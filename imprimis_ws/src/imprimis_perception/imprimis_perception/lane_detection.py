"""Backward-compatible executable for the new segmentation-only lane node.

The projection stage now lives in lane_projection.py. Use lane_detection.launch.py
to start four per-camera segmenters and the shared projector.
"""
from imprimis_perception.segmentation import main

class LaneDetection(Node):
    def __init__(self):
        super().__init__('lane_detection')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('process_rate_hz',10.0)
        self.declare_parameter('camera_height', 0.675) #meters
        self.declare_parameter('camera_angle', 77.0) #0 = pointing straight down, 90 = looking out to the horizon
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('lower_white', (0,0, 200))
        self.declare_parameter('upper_white', (179, 60, 255))
        self.declare_parameter('min_area', 50.0)
        self.declare_parameter('max_area', 60000.0)


        self.theta = self.get_parameter('camera_angle').value # remember to use to calculate the distance in z and x ranges
        self.height = self.get_parameter('camera_height').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.process_period = 1.0 / self.get_parameter('process_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.lower_white = self.get_parameter('lower_white').value
        self.upper_white = self.get_parameter('upper_white').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.kernel = np.ones((3, 3), np.uint8)
        
        
        self.bridge = CvBridge() #converts Ros message into OpenCv numpy matrix
        self.maskBridge = CvBridge()

        self.camera_matrix = None
        self.have_intrinsics = False
        self.maskPublish = True #should we publish the mask view, keep true for testing


       
        self.last_process_time = self.get_clock().now()
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            camera_info_topic, 
            self.camera_info_cb, 
            1)

        self.image_sub = self.create_subscription( 
            Image, 
            image_topic,
            self.image_cb,
            1  # 1 is queue size, should stay at 1 to stay real time
        )


        self.camera_mask = self.create_publisher(
            Image,
            'perception/mask',
            1
        )

        self.raw_mask_pub = self.create_publisher(
            Image,
            'perception/mask_raw',
             1
             )

        self.lane_pointcloud = self.create_publisher(
            PointCloud2,
            'perception/lane_points',
            1
        )

        self.get_logger().info(
            f'Lane detector started. Waiting on {image_topic} and {camera_info_topic}'
        )


    def camera_info_cb(self, msg: CameraInfo):
        if self.have_intrinsics:
            return
        
        self.camera_matrix= np.array(msg.k).reshape(3,3)
        self.image_width = msg.width
        self.image_height = msg.height
        self.have_intrinsics = True
        self.get_logger().info(f'Cached Camera intrinsics:\n{self.camera_matrix}')
        self.recompute()
   
   
    def parameter_callback(self, params):
        for param in params:
            if (param.name == 'process_rate_hz'):
                if param.value <= 0:
                    return SetParametersResult(successful = False, reason='process_rate_hz was 0 or lower')
                self.process_period = 1.0/param.value
           
            if(param.name == 'camera_angle'):
                if param.value >= 90 or param.value <=0:
                    return SetParametersResult(successful = False, reason='Angle cannot = 90 or 0')
                self.theta = param.value
                self.recompute()

            if(param.name == 'camera_height'):
                self.height = param.value
                self.recompute()

            if(param.name == 'lower_white'):
                self.lower_white = param.value
           
            if (param.name == 'upper_white'):
                self.upper_white = param.value

            if param.name == 'min_area':
                self.min_area = param.value
            
            if param.name == 'max_area':
                self.max_area = param.value
        
        return SetParametersResult(successful=True)


    def image_cb(self, msg: Image):
        if not self.have_intrinsics:
            return #need to have intrinsics before we can really do much

        now = self.get_clock().now()
       # self.raw_mask_pub.publish(self.raw_msg)
        
        
        elapsed = (now - self.last_process_time).nanoseconds / 1e9
        if elapsed < self.process_period:
            return
        self.last_process_time = now

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.mask = self.segment_lane_pixels(cv_image)


        lane_pixel_count = cv2.countNonZero(self.mask)
        if self.maskPublish == True:
            self.publishMask(self.mask)
            self.raw_mask_pub.publish(self.maskBridge.cv2_to_imgmsg(self.raw_mask,encoding='mono8'))
        
        lane_points = self.raycast(self.mask)
        self.publishPointCloud(lane_points)
        #self.get_logger().info(f'LanePoints: {lane_points}')
        self.get_logger().info(f'Lane pixels detected: {lane_pixel_count}', throttle_duration_sec=1.0)# for testing


    def publishMask(self, cv_image):
        cv_image = self.maskBridge.cv2_to_imgmsg(cv_image,encoding='mono8')
        self.camera_mask.publish(cv_image)


    def segment_lane_pixels(self, cv_image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        val_enhanced = clahe.apply(val)
        blurred = cv2.GaussianBlur(val_enhanced, (5, 5), 0) #0 controls the spread of the gaussian distribution, 0 is set to auto instead of choosing our own
        hsv_enhanced = cv2.merge([hue, sat, blurred])
        mask = cv2.inRange(hsv_enhanced, self.lower_white, self.upper_white)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        self.raw_mask = mask.copy() #only for comparison, get rid of later when we no longer need

        filter_bin_mask = np.zeros_like(mask)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            if self.min_area <= cv2.contourArea(contour) <= self.max_area:
                cv2.drawContours(filter_bin_mask, [contour], -1, 255, cv2.FILLED)

        return filter_bin_mask


    def raycast(self, mask: np.ndarray):
        hits = mask[self.v_valid, self.u_valid] == 255
        if not np.any(hits):
            return []

        zg = self.Zg[hits]                  # ground distance forward
        xg = self.Xg[hits]                  # ground distance to the right
        xy = np.column_stack((zg, -xg))     # top view: x forward, y left

        cells = thin_structures(xy, cell=0.05, x_range=(0.0, 4.0), half_width=3.5, max_width=0.45)
        if len(cells) == 0:
            return []

        zg = cells[:, 0]                    # back to forward distance
        xg = -cells[:, 1]                   # back to right distance

        th = math.radians(self.theta)
        y_cam = self.height * math.sin(th) - zg * math.cos(th)
        z_cam = self.height * math.cos(th) + zg * math.sin(th)

        return np.column_stack((xg, y_cam, z_cam)).astype(np.float32).tolist()

    def publishPointCloud(self, lane_points):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()

        cloud_msg = point_cloud2.create_cloud(header, fields, lane_points)
        self.lane_pointcloud.publish(cloud_msg)        


    def recompute(self):
        if not self.have_intrinsics:
            return

        fx = self.camera_matrix[0][0] #focal length horizontal
        fy = self.camera_matrix[1][1] #focal length vertical
        cx = self.camera_matrix[0][2] #principle point horizontal
        cy = self.camera_matrix[1][2] #principle point vertical

        th = math.radians(self.theta)

        # ground grid for inverse mapping 
        z_max = min(self.height * math.tan(th), 4.0)
        z_values = np.arange(0.5, z_max, 0.01)
        x_values = np.arange(-3.0, 3.0, 0.01)
        X, Z = np.meshgrid(x_values, z_values)

        y_cam = self.height * math.sin(th) - Z * math.cos(th)
        z_cam = self.height * math.cos(th) + Z * math.sin(th)

        u = np.round(fx * (X / z_cam) + cx).astype(int)
        v = np.round(fy * (y_cam / z_cam) + cy).astype(int)

        valid = (u >= 0) & (u < self.image_width) & (v >= 0) & (v < self.image_height)
        
        self.u_valid = u[valid]
        self.v_valid = v[valid]
        self.Xg = X[valid]   
        self.Zg = Z[valid]   

    def morphological_skeleton(image):
        """Reduce every mark in a binary image to a line about one cell wide (its middle)."""
        cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        skeleton = np.zeros_like(image)
        work = image.copy()
        for _ in range(64):
            eroded = cv2.erode(work, cross)
            opened = cv2.dilate(eroded, cross)
            skeleton = cv2.bitwise_or(skeleton, cv2.subtract(work, opened))
            work = eroded
            if cv2.countNonZero(work) == 0:
                break
        return skeleton


    def thin_structures(xy, cell=0.05, x_range=(0.0, 6.0), half_width=4.0, max_width=0.45, min_cells=5, skeleton=True):
        """Keep only narrow marks, such as 4 inch tape, and drop wide white areas.

        The points are drawn on a top-view grid. Anything that survives erosion by max_width is a wide
        area; it is removed together with a border around it. Specks smaller than min_cells are removed
        too. With skeleton set, each remaining mark is reduced to its middle line, so that tape seen in
        many frames does not build up into a wide band in the cost map. Returns the centers of the
        remaining cells.
        """
        if len(xy) == 0:
            return np.zeros((0, 2))
        nx = int(round((x_range[1] - x_range[0]) / cell))
        ny = int(round(2 * half_width / cell))
        ix = ((xy[:, 0] - x_range[0]) / cell).astype(int)
        iy = ((xy[:, 1] + half_width) / cell).astype(int)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        grid = np.zeros((nx, ny), np.uint8)
        grid[ix[ok], iy[ok]] = 255
        closed = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        k = max(3, int(round(max_width / cell)))
        wide = cv2.morphologyEx(closed, cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
        wide = cv2.dilate(wide, np.ones((k, k), np.uint8))
        thin = cv2.bitwise_and(closed, cv2.bitwise_not(wide))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(thin, connectivity=8)
        small = np.nonzero(stats[:, cv2.CC_STAT_AREA] < min_cells)[0]
        if small.size:
            thin[np.isin(labels, small)] = 0
        if skeleton:
            thin = morphological_skeleton(thin)
        gx, gy = np.nonzero(thin)
        return np.stack([x_range[0] + (gx + 0.5) * cell, -half_width + (gy + 0.5) * cell], axis=-1)


    def find_white_discs(xy, cell=0.05, x_range=(0.0, 3.5), half_width=4.0, min_diameter=0.40, max_diameter=0.85,
                        tape_width=0.30, min_fill=0.70, min_aspect=0.72, alone_margin=0.25):
        """Painted potholes: round white areas on the ground. In the IGVC a pothole is a solid white circle 2 ft across.

        xy are white pixels placed on the ground (from mask_to_ground_points, so things that stand up are
        already gone). Lane tape is removed by an opening wider than the tape; what is left and is about
        as long as it is wide, and filled, is a pothole. Returns a list of (x, y, diameter, cells), where
        cells are the centers of the grid cells the pothole covers.

        Three things keep lane tape from being taken for a pothole, which happened in a first version where
        the tape curves or the robot stands close and askew to it. The shape must be nearly as wide as it is
        long. Its size must be near that of the painted circle (0.61 m). And it must stand alone: the white
        area it belongs to, before the tape was removed, may not reach more than alone_margin beyond it. A
        piece of lane line always has more line attached. The price is that a pothole touching a lane line,
        or one only half in view, is not reported until it is seen whole.
        """
        if cv2 is None or len(xy) == 0:
            return []
        grid = _top_view(xy, cell, x_range, half_width)
        # far away the rows of pixels land more than a cell apart: close the gaps before judging the shape
        closed = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        k = max(3, int(round(tape_width / cell)))
        wide = cv2.morphologyEx(closed, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
        found = []
        count, labels, stats, _ = cv2.connectedComponentsWithStats(closed, connectivity=8)
        for hx, hy, diameter, cells in _round_blobs(wide, cell, x_range, half_width, min_diameter, max_diameter, min_fill, min_aspect):
            # the white area this blob was part of before the opening
            ix = int((hx - x_range[0]) / cell)
            iy = int((hy + half_width) / cell)
            label = labels[min(max(ix, 0), labels.shape[0] - 1), min(max(iy, 0), labels.shape[1] - 1)]
            if label == 0:
                continue
            whole = max(stats[label, cv2.CC_STAT_WIDTH], stats[label, cv2.CC_STAT_HEIGHT]) * cell
            if whole > diameter + alone_margin:
                continue                    # more white is attached: a lane line, not a pothole
            # not cut off by the edge of what the camera can place on the ground
            if hx + diameter / 2 > x_range[1] - cell or hx - diameter / 2 < x_range[0] + cell:
                continue
            found.append((hx, hy, diameter, cells))
        return found



def main(args=None):
    rclpy.init(args=args)
    node = LaneDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
