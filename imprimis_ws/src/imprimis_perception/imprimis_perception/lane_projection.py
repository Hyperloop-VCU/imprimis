
import math
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from imprimis_perception import surface_tools as st



def scaled_intrinsics(k, info_width, info_height, mask_width, mask_height):
    """fx, fy, cx, cy of the camera for a mask that was resized from the CameraInfo resolution."""
    sx = mask_width / float(info_width)
    sy = mask_height / float(info_height)
    return k[0][0] * sx, k[1][1] * sy, k[0][2] * sx, k[1][2] * sy


def build_ground_lookup(intr, width, height, r_base_cam, t_base_cam, ground_z,
                        min_range=0.5, max_range=4.0, cell=0.01):
    
    fx, fy, cx, cy = intr
    gx = np.arange(t_base_cam[0] - max_range, t_base_cam[0] + max_range, cell)
    gy = np.arange(t_base_cam[1] - max_range, t_base_cam[1] + max_range, cell)
    X, Y = np.meshgrid(gx, gy)
    X, Y = X.ravel(), Y.ravel()
    reach = np.hypot(X - t_base_cam[0], Y - t_base_cam[1])
    near = (reach >= min_range) & (reach <= max_range)
    X, Y = X[near], Y[near]

    ground = np.stack([X, Y, np.full_like(X, ground_z)], axis=-1)
    p_cam = (ground - t_base_cam) @ r_base_cam        # r.T @ (p - t), row by row
    ahead = p_cam[:, 2] > 0.05
    p_cam, X, Y = p_cam[ahead], X[ahead], Y[ahead]
    u = np.round(fx * p_cam[:, 0] / p_cam[:, 2] + cx).astype(int)
    v = np.round(fy * p_cam[:, 1] / p_cam[:, 2] + cy).astype(int)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    return u[inside], v[inside], np.stack([X[inside], Y[inside]], axis=-1)


def mask_to_ground_xy(mask, u, v, xy):
    hits = mask[v, u] > 0
    return xy[hits]


def grid_bounds(xy, cell=0.05):
    x0 = math.floor(xy[:, 0].min() / cell) * cell
    x1 = math.ceil(xy[:, 0].max() / cell) * cell + cell
    half = math.ceil(np.abs(xy[:, 1]).max() / cell) * cell + cell
    return (x0, x1), half


def camera_height_and_pitch(r_base_cam, t_base_cam, ground_z):
    forward = r_base_cam[:, 2]
    return t_base_cam[2] - ground_z, math.degrees(math.asin(max(-1.0, min(1.0, -forward[2]))))


def rotation_and_translation(transform):
    # p_parent = r @ p_child + t
    t = transform.transform.translation
    q = transform.transform.rotation
    r = st.quat_to_matrix(q.x, q.y, q.z, q.w)
    return r, np.array([t.x, t.y, t.z])



def fit_ground_plane(points, iterations=150, distance_threshold=0.05, min_inlier_fraction=0.3):
    #basically draw a plane from three randomly selected points, and check how many points have less than the distance_threshold from that plane and use that to find the best plane
    n = len(points)
    if n < 50:
        return None

    rng = np.random.default_rng()
    best_count, best_normal, best_d = 0, None, None
    for _ in range(iterations):
        p0, p1, p2 = points[rng.choice(n, 3, replace=False)]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal = normal / norm
        d = -normal @ p0
        count = int((np.abs(points @ normal + d) < distance_threshold).sum())
        if count > best_count:
            best_count, best_normal, best_d = count, normal, d

    if best_normal is None or best_count < min_inlier_fraction * n:
        return None

    # refine with a least-squares fit over the inliers
    inliers = points[np.abs(points @ best_normal + best_d) < distance_threshold]
    centroid = inliers.mean(axis=0)
    _, _, vh = np.linalg.svd(inliers - centroid)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    return normal, -normal @ centroid


def obstacle_cells(points_lidar, r_base_lidar, t_base_lidar, ground_z, obstacle_height=0.12, max_height=1.0,
                   plane_distance=0.05, cell=0.1):
    if len(points_lidar) == 0:
        return np.zeros((0, 3))
    points = points_lidar @ r_base_lidar.T + t_base_lidar
    plane = fit_ground_plane(points_lidar, distance_threshold=plane_distance)
    if plane is None or (r_base_lidar @ plane[0])[2] < 0.95:     # none found, or it is a wall: tilted over 18 deg
        height = points[:, 2] - ground_z
    else:
        height = points_lidar @ plane[0] + plane[1]
    keep = (height > obstacle_height) & (height <= max_height)
    if not keep.any():
        return np.zeros((0, 3))
    xy, height = points[keep, :2], height[keep]
    keys = np.floor(xy / cell).astype(int)
    order = np.argsort(-height)
    _, first = np.unique(keys[order], axis=0, return_index=True)
    idx = order[first]
    return np.column_stack([xy[idx], height[idx]])


def reject_near_obstacles(xy, obstacle_xy, radius=0.2):
    if len(xy) == 0 or len(obstacle_xy) == 0:
        return xy
    d2 = ((xy[:, None, :] - obstacle_xy[None, :, :]) ** 2).sum(axis=2)
    return xy[d2.min(axis=1) >= radius ** 2]


def reject_hidden(xy, camera_xy, camera_height, obstacles, cell=0.1):
    if len(xy) == 0 or len(obstacles) == 0:
        return xy
    d = xy - camera_xy
    r = np.hypot(d[:, 0], d[:, 1])
    bearing = np.arctan2(d[:, 1], d[:, 0])
    o = obstacles[:, :2] - camera_xy
    r_o = np.hypot(o[:, 0], o[:, 1])
    bearing_o = np.arctan2(o[:, 1], o[:, 0])
    h = np.minimum(obstacles[:, 2], 0.98 * camera_height)
    shadow_end = r_o * camera_height / (camera_height - h)
    half_width = np.arctan2(0.75 * cell, r_o)
    turn = np.abs((bearing[:, None] - bearing_o[None, :] + np.pi) % (2 * np.pi) - np.pi)
    hidden = (turn <= half_width) & (r[:, None] > r_o) & (r[:, None] <= shadow_end)
    return xy[~hidden.any(axis=1)]


class Camera:
    def __init__(self, name):
        self.name = name
        self.info = None            # CameraInfo, the first one received
        self.mount = None           # (r, t) of the optical frame in the base frame
        self.frame_id = None
        self.lookup = None          # (u, v, xy, mask shape)
        self.stamp = None           # seconds, of the newest mask
        self.lanes = np.zeros((0, 2))
        self.holes = np.zeros((0, 2))


class LaneProjection(Node):
    def __init__(self):
        super().__init__('lane_projection')

        self.declare_parameter('camera_names', ['d435i'])
        # {name} is replaced by each camera name
        self.declare_parameter('mask_topic_pattern', '/perception/{name}/lane_mask/compressed')
        self.declare_parameter('camera_info_topic_pattern', '/cameras/{name}/color/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ground_frame', 'base_footprint')  # the ground plane is z of this frame
        self.declare_parameter('output_height', 0.0)              # m above the ground the points are published at, see the docstring
        self.declare_parameter('min_range', 0.5)                  # m along the ground from the camera
        self.declare_parameter('max_range', 4.0)                  # beyond about 4 m one pixel covers too much ground
        self.declare_parameter('max_line_width', 0.45)            # wider white areas are not lane tape
        self.declare_parameter('max_camera_age', 0.5)             # s; another camera's points older than this are left out
        self.declare_parameter('detect_potholes', True)
        self.declare_parameter('lidar_topic', 'velodyne_points')
        self.declare_parameter('lidar_frame_id', 'velodyne')
        self.declare_parameter('ground_plane_distance', 0.05)   # RANSAC inlier threshold, meters
        self.declare_parameter('obstacle_height', 0.12)         # above the plane counts as an obstacle
        self.declare_parameter('obstacle_max_height', 1.0)      # higher returns (a ceiling) do not stand on the ground
        self.declare_parameter('obstacle_reject_radius', 0.2)   # veto lane points this close to one
        self.declare_parameter('ground_plane_range', 6.0)       # only fit the plane to points within this far, meters
        self.declare_parameter('lidar_min_range', 0.5)          # drop closer returns -- the sim LiDAR reports
                                                                  # degenerate (0,0,0) self-hits inside this radius

        names = list(self.get_parameter('camera_names').value)
        mask_pattern = self.get_parameter('mask_topic_pattern').value
        info_pattern = self.get_parameter('camera_info_topic_pattern').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ground_frame = self.get_parameter('ground_frame').value
        self.output_height = self.get_parameter('output_height').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.max_line_width = self.get_parameter('max_line_width').value
        self.max_camera_age = self.get_parameter('max_camera_age').value
        self.detect_potholes = self.get_parameter('detect_potholes').value
        lidar_topic = self.get_parameter('lidar_topic').value
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value
        self.ground_plane_distance = self.get_parameter('ground_plane_distance').value
        self.obstacle_height = self.get_parameter('obstacle_height').value
        self.obstacle_max_height = self.get_parameter('obstacle_max_height').value
        self.obstacle_reject_radius = self.get_parameter('obstacle_reject_radius').value
        self.ground_plane_range = self.get_parameter('ground_plane_range').value
        self.lidar_min_range = self.get_parameter('lidar_min_range').value

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf = {}
        self.ground_z = None
        self.obstacles = np.zeros((0, 3))

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.cameras = []
        for name in names:
            camera = Camera(name)
            self.cameras.append(camera)
            mask_topic = mask_pattern.format(name=name)
            info_topic = info_pattern.format(name=name)
            self.create_subscription(CameraInfo, info_topic, lambda m, c=camera: self.camera_info_cb(c, m), 1)
            self.create_subscription(CompressedImage, mask_topic, lambda m, c=camera: self.mask_cb(c, m), 1)
            self.get_logger().info(f'Camera {name}: waiting on {mask_topic} and {info_topic}')
        self.create_subscription(PointCloud2, lidar_topic, self.lidar_cb, 1)

        self.lane_pointcloud = self.create_publisher(PointCloud2, 'perception/lane_points', 1)
        self.pothole_pointcloud = self.create_publisher(PointCloud2, 'perception/pothole_points', 1)

    def lookup_static(self, child):
        """(r, t) of a frame fixed to the robot, in the base frame, or None until TF has it. Cached."""
        if child in self.static_tf:
            return self.static_tf[child]
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, child, Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().info(f'Waiting on TF {self.base_frame} -> {child}: {ex}', throttle_duration_sec=5.0)
            return None
        self.static_tf[child] = rotation_and_translation(transform)
        return self.static_tf[child]

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'detect_potholes':
                self.detect_potholes = param.value
        return SetParametersResult(successful=True)

    def camera_info_cb(self, camera, msg: CameraInfo):
        if camera.info is None:
            camera.info = msg

    def lidar_cb(self, msg: PointCloud2):
        mount = self.lookup_static(self.lidar_frame_id)
        if mount is None or not self.know_ground():
            return
        points = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        if len(points) == 0:
            return
        points = points[np.isfinite(points).all(axis=1)]
        r = np.linalg.norm(points, axis=1)
        points = points[(r > self.lidar_min_range) & (r < self.ground_plane_range)]
        self.obstacles = obstacle_cells(points, mount[0], mount[1], self.ground_z, self.obstacle_height,
                                        self.obstacle_max_height, self.ground_plane_distance)

    def know_ground(self):
        if self.ground_z is None:
            ground = self.lookup_static(self.ground_frame)
            if ground is None:
                return False
            self.ground_z = float(ground[1][2])
        return True

    def ready(self, camera, msg, mask):
        if camera.info is None:
            self.get_logger().info(f'Camera {camera.name}: waiting on CameraInfo', throttle_duration_sec=5.0)
            return False
        if not self.know_ground():
            return False
        if camera.mount is None or camera.frame_id != msg.header.frame_id:
            mount = self.lookup_static(msg.header.frame_id)
            if mount is None:
                return False
            camera.mount, camera.frame_id, camera.lookup = mount, msg.header.frame_id, None
            height, pitch = camera_height_and_pitch(mount[0], mount[1], self.ground_z)
            self.get_logger().info(f'Camera {camera.name}: {camera.frame_id} is {height:.3f} m above the ground, '
                                   f'looking {pitch:.1f} deg down')
        if camera.lookup is None or camera.lookup[3] != mask.shape:
            intr = scaled_intrinsics(np.array(camera.info.k).reshape(3, 3), camera.info.width, camera.info.height,
                                     mask.shape[1], mask.shape[0])
            u, v, xy = build_ground_lookup(intr, mask.shape[1], mask.shape[0], camera.mount[0], camera.mount[1],
                                           self.ground_z, self.min_range, self.max_range)
            camera.lookup = (u, v, xy, mask.shape)
        return True

    def lidar_veto(self, camera, xy):
        obstacles = self.obstacles
        xy = reject_near_obstacles(xy, obstacles[:, :2], self.obstacle_reject_radius)
        t = camera.mount[1]
        return reject_hidden(xy, t[:2], t[2] - self.ground_z, obstacles)

    def mask_cb(self, camera, msg: CompressedImage):
        mask = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if mask.ndim == 3:
            mask = mask[:, :, 0]
        if not self.ready(camera, msg, mask):
            return

        u, v, xy, _ = camera.lookup
        white = mask_to_ground_xy(mask, u, v, xy)
        camera.lanes = np.zeros((0, 2))
        camera.holes = np.zeros((0, 2))
        if len(white):
            x_range, half_width = grid_bounds(xy)
            camera.lanes = self.lidar_veto(camera, st.thin_structures(
                white, cell=0.05, x_range=x_range, half_width=half_width, max_width=self.max_line_width))
            if self.detect_potholes:
                discs = st.find_white_discs(white, cell=0.05, x_range=x_range, half_width=half_width)
                centers = np.array([[hx, hy] for hx, hy, _, _ in discs]).reshape(-1, 2)
                camera.holes = self.lidar_veto(camera, centers)
        camera.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        # every camera with a recent enough mask contributes to one cloud
        fresh = [c for c in self.cameras
                 if c.stamp is not None and abs(camera.stamp - c.stamp) <= self.max_camera_age]
        self.publish_point_cloud(self.lane_pointcloud, np.concatenate([c.lanes for c in fresh]), msg.header.stamp)
        if self.detect_potholes:
            self.publish_point_cloud(self.pothole_pointcloud, np.concatenate([c.holes for c in fresh]),
                                     msg.header.stamp)

    def publish_point_cloud(self, publisher, xy, stamp):
        points = np.zeros((len(xy), 3), np.float32)
        points[:, :2] = xy
        points[:, 2] = self.ground_z + self.output_height   # ground_z is known: no mask is used before it is
        header = Header()
        header.frame_id = self.base_frame
        header.stamp = stamp
        publisher.publish(point_cloud2.create_cloud_xyz32(header, points))


def main(args=None):
    rclpy.init(args=args)
    node = LaneProjection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
