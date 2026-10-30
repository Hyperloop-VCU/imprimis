"""Project four camera lane masks onto the robot's ground plane.

Each mask is paired with that camera's CameraInfo and TF transform. The projector
combines the latest fresh per-camera points into one PointCloud2 in base_frame.
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros

from imprimis_perception import surface_tools as st


def build_ground_grid(camera_matrix, image_width, image_height, height, angle_deg, max_range):
    fx = camera_matrix[0][0]
    fy = camera_matrix[1][1]
    cx = camera_matrix[0][2]
    cy = camera_matrix[1][2]

    th = math.radians(angle_deg)
    z_max = min(height * math.tan(th), max_range)
    z_values = np.arange(0.5, z_max, 0.01)
    x_values = np.arange(-3.0, 3.0, 0.01)
    X, Z = np.meshgrid(x_values, z_values)

    y_cam = height * math.sin(th) - Z * math.cos(th)
    z_cam = height * math.cos(th) + Z * math.sin(th)

    u = np.round(fx * (X / z_cam) + cx).astype(int)
    v = np.round(fy * (y_cam / z_cam) + cy).astype(int)
    valid = (u >= 0) & (u < image_width) & (v >= 0) & (v < image_height)

    return u[valid], v[valid], X[valid], Z[valid]


def mask_to_ground_xy(mask, u_valid, v_valid, Xg, Zg):
    hits = mask[v_valid, u_valid] == 255
    if not np.any(hits):
        return np.zeros((0, 2))

    zg = Zg[hits]
    xg = Xg[hits]
    return np.column_stack((zg, -xg))   # top view: x forward, y left


def ground_to_camera(zg, xg, height, angle_deg):
    th = math.radians(angle_deg)
    y_cam = height * math.sin(th) - zg * math.cos(th)
    z_cam = height * math.cos(th) + zg * math.sin(th)
    return xg, y_cam, z_cam


def raycast(xy, max_range, height, angle_deg):
    if len(xy) == 0:
        return []

    cells = st.thin_structures(xy, cell=0.05, x_range=(0.0, max_range), half_width=3.5, max_width=0.45)
    if len(cells) == 0:
        return []

    zg = cells[:, 0]
    xg = -cells[:, 1]
    xg, y_cam, z_cam = ground_to_camera(zg, xg, height, angle_deg)
    return np.column_stack((xg, y_cam, z_cam)).astype(np.float32).tolist()


def find_potholes(xy, max_range, height, angle_deg):
    if len(xy) == 0:
        return []

    discs = st.find_white_discs(xy, cell=0.05, x_range=(0.0, max_range), half_width=3.5)
    if not discs:
        return []

    zg = np.array([hx for hx, hy, diameter, cells in discs])
    xg = -np.array([hy for hx, hy, diameter, cells in discs])
    xg, y_cam, z_cam = ground_to_camera(zg, xg, height, angle_deg)
    return np.column_stack((xg, y_cam, z_cam)).astype(np.float32).tolist()


def rotation_and_translation(transform):
    # p_parent = r @ p_child + t
    t = transform.transform.translation
    q = transform.transform.rotation
    r = st.quat_to_matrix(q.x, q.y, q.z, q.w)
    return r, np.array([t.x, t.y, t.z])


def height_and_angle_from_transform(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    r = st.quat_to_matrix(q.x, q.y, q.z, q.w)
    forward_z = r[2, 2]  # ground-frame z of the optical z-axis (forward)
    angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, -forward_z))))
    return t.z, angle_deg


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


def classify_obstacles(points, plane, obstacle_height=0.12):
    """(x, y) in the LiDAR's own frame of points standing clearly above the fitted ground plane."""
    normal, d = plane
    height_above = points @ normal + d
    obstacles = points[height_above > obstacle_height]
    if len(obstacles) == 0:
        return np.zeros((0, 2))
    # voxel-downsample so the proximity check below stays cheap
    keys = np.round(obstacles[:, :2] / 0.1).astype(int)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return obstacles[idx, :2]


def reject_near_obstacles(points_cam, r_lidar_from_cam, t_lidar_from_cam, obstacle_xy, radius=0.2):
    """points_cam (camera-frame x,y,z lists), dropping the ones near an obstacle seen by the LiDAR --
    e.g. a wall-mounted sign the 2D mask mistook for tape."""
    if len(points_cam) == 0 or len(obstacle_xy) == 0:
        return points_cam

    p_lidar = np.array(points_cam) @ r_lidar_from_cam.T + t_lidar_from_cam
    d2 = ((p_lidar[:, None, :2] - obstacle_xy[None, :, :]) ** 2).sum(axis=2)
    near = d2.min(axis=1) < radius ** 2
    return [p for p, bad in zip(points_cam, near) if not bad]


class LaneProjection(Node):
    def __init__(self):
        super().__init__('lane_projection')
        self.declare_parameter('camera_names', ['d455', 'd435i', 'd435_1', 'd435_2'])
        self.declare_parameter('mask_topics', [
            'perception/d455/lane_mask', 'perception/d435i/lane_mask',
            'perception/d435_1/lane_mask', 'perception/d435_2/lane_mask'])
        self.declare_parameter('camera_info_topics', [
            'cameras/d455/color/camera_info', 'cameras/d435i/color/camera_info',
            'cameras/d435_1/color/camera_info', 'cameras/d435_2/color/camera_info'])
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_topic', 'perception/lane_points')
        self.declare_parameter('ground_frame', 'base_footprint')
        self.declare_parameter('ground_z', -0.1277)
        self.declare_parameter('min_forward', 0.5)
        self.declare_parameter('max_forward', 4.0)
        self.declare_parameter('half_width', 3.5)
        self.declare_parameter('pixel_step', 2)
        self.declare_parameter('grid_cell', 0.05)
        self.declare_parameter('max_line_width', 0.45)
        self.declare_parameter('max_camera_age', 0.35)
        self.declare_parameter('output_z', 0.5)

        self.declare_parameter('mask_topic', 'perception/mask/compressed')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('detect_potholes', True)
        self.declare_parameter('lidar_topic', 'velodyne_points')
        self.declare_parameter('lidar_frame_id', 'velodyne')
        self.declare_parameter('ground_plane_distance', 0.05)   # RANSAC inlier threshold, meters
        self.declare_parameter('obstacle_height', 0.12)         # above the plane counts as an obstacle
        self.declare_parameter('obstacle_reject_radius', 0.2)   # veto lane points this close to one
        self.declare_parameter('ground_plane_range', 6.0)       # only fit the plane to points within this far, meters
        self.declare_parameter('lidar_min_range', 0.5)          # drop closer returns -- the sim LiDAR reports
                                                                  # degenerate (0,0,0) self-hits inside this radius

        mask_topic = self.get_parameter('mask_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.detect_potholes = self.get_parameter('detect_potholes').value
        lidar_topic = self.get_parameter('lidar_topic').value
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value
        self.ground_plane_distance = self.get_parameter('ground_plane_distance').value
        self.obstacle_height = self.get_parameter('obstacle_height').value
        self.obstacle_reject_radius = self.get_parameter('obstacle_reject_radius').value
        self.ground_plane_range = self.get_parameter('ground_plane_range').value
        self.lidar_min_range = self.get_parameter('lidar_min_range').value
        self.max_range = 4.0

        self.bridge = CvBridge()

        self.camera_matrix = None
        self.have_intrinsics = False
        self.height = None
        self.theta = None
        self.geometry_ready = False
        self.u_valid = self.v_valid = self.Xg = self.Zg = None

        self.r_lidar_from_cam = None
        self.t_lidar_from_cam = None
        self.lidar_extrinsic_ready = False
        self.obstacle_xy = np.zeros((0, 2))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_info = [None] * CAMERA_COUNT
        self.camera_points = [None] * CAMERA_COUNT
        self.camera_stamps = [None] * CAMERA_COUNT
        self.latest_stamp = None
        self.latest_stamp_msg = None
        self.ground_frame_checked = False
        self.info_subs = []
        self.mask_subs = []
        for index, (info_topic, mask_topic) in enumerate(zip(self.info_topics, self.mask_topics)):
            self.info_subs.append(self.create_subscription(
                CameraInfo, info_topic,
                lambda msg, i=index: self.camera_info_cb(i, msg),
                qos_profile_sensor_data))
            self.mask_subs.append(self.create_subscription(
                Image, mask_topic,
                lambda msg, i=index: self.mask_cb(i, msg),
                qos_profile_sensor_data))
        self.points_pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 10)
        self.get_logger().info(
            f'Projecting masks from {", ".join(self.camera_names)} into {self.base_frame}')

    def camera_info_cb(self, index, msg):
        k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        if msg.width <= 0 or msg.height <= 0 or k[0, 0] <= 0 or k[1, 1] <= 0:
            self.get_logger().warning(
                f'Ignoring invalid CameraInfo from {self.camera_names[index]}',
                throttle_duration_sec=5.0)
            return
        if msg.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
            self.get_logger().error(
                f'{self.camera_names[index]} uses unsupported distortion model '
                f'{msg.distortion_model!r}; expected a pinhole/plumb_bob model.')
            self.camera_info[index] = None
            return
        d = np.asarray(msg.d, dtype=np.float64)
        if d.size == 0:
            d = np.zeros(5, dtype=np.float64)
        self.camera_info[index] = {
            'k': k, 'd': d, 'width': int(msg.width), 'height': int(msg.height),
            'frame_id': msg.header.frame_id,
        }

    @staticmethod
    def _stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec / 1e9

    @staticmethod
    def _rotation_from_quaternion(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm < 1e-12:
            raise ValueError('TF contains a zero quaternion')
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ], dtype=np.float64)

    def mask_cb(self, index, msg):
        if not self.ground_frame_checked:
            try:
                ground_tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.ground_frame, Time())
                self.ground_z = float(ground_tf.transform.translation.z)
                self.ground_frame_checked = True
                self.get_logger().info(
                    f'Ground plane from {self.ground_frame}: z={self.ground_z:.3f} m in {self.base_frame}')
            except tf2_ros.TransformException:
                self.get_logger().warning(
                    f'No transform from {self.base_frame} to {self.ground_frame}; '
                    f'using configured ground_z={self.ground_z:.3f} m.',
                    throttle_duration_sec=10.0)
        info = self.camera_info[index]
        if info is None:
            self.get_logger().warning(
                f'Waiting for valid CameraInfo on {self.info_topics[index]}',
                throttle_duration_sec=5.0)
            return
        if msg.encoding != 'mono8' or msg.width <= 0 or msg.height <= 0:
            self.get_logger().warning(
                f'Ignoring malformed mask on {self.mask_topics[index]} (expected mono8)',
                throttle_duration_sec=5.0)
            return
        if info['frame_id'] and msg.header.frame_id and info['frame_id'] != msg.header.frame_id:
            self.get_logger().warning(
                f'{self.camera_names[index]} mask frame {msg.header.frame_id!r} does not match '
                f'CameraInfo frame {info["frame_id"]!r}', throttle_duration_sec=5.0)
            return
        expected_bytes = msg.step * msg.height
        if msg.step < msg.width or len(msg.data) < expected_bytes:
            self.get_logger().warning(
                f'Ignoring truncated mask from {self.camera_names[index]}',
                throttle_duration_sec=5.0)
            return
        mask = np.frombuffer(msg.data, dtype=np.uint8, count=expected_bytes)
        mask = mask.reshape(msg.height, msg.step)[:, :msg.width]
        rows, cols = np.nonzero(mask[::self.pixel_step, ::self.pixel_step])
        stamp = self._stamp_seconds(msg.header.stamp)
        if self.latest_stamp is None or stamp >= self.latest_stamp:
            self.latest_stamp = stamp
            self.latest_stamp_msg = msg.header.stamp
        self.camera_stamps[index] = stamp
        if rows.size == 0:
            self.camera_points[index] = np.zeros((0, 2), dtype=np.float32)
            self.publish_combined()
            return

        rows = rows.astype(np.float64) * self.pixel_step
        cols = cols.astype(np.float64) * self.pixel_step
        # Scale intrinsics if a camera publishes a mask at a resized resolution.
        sx = msg.width / info['width']
        sy = msg.height / info['height']
        k = info['k'].copy()
        k[0, :] *= sx
        k[1, :] *= sy
        pixels = np.column_stack((cols, rows)).astype(np.float32).reshape(-1, 1, 2)
        try:
            normalized = cv2.undistortPoints(pixels, k, info['d']).reshape(-1, 2)
            rays_camera = np.column_stack((normalized, np.ones(len(normalized))))
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id,
                Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.05))
            rotation = self._rotation_from_quaternion(tf.transform.rotation)
            origin = np.array([
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ], dtype=np.float64)
            rays_base = rays_camera @ rotation.T
            downward = rays_base[:, 2] < -1e-6
            ray_z = rays_base[:, 2]
            scale = np.full(len(ray_z), np.nan, dtype=np.float64)
            scale[downward] = (self.ground_z - origin[2]) / ray_z[downward]
            forward = scale > 0
            points = origin + rays_base * np.nan_to_num(scale, nan=0.0)[:, None]
            keep = (downward & forward
                    & (points[:, 0] >= self.min_forward)
                    & (points[:, 0] <= self.max_forward)
                    & (np.abs(points[:, 1]) <= self.half_width))
            self.camera_points[index] = points[keep, :2].astype(np.float32)
        except (cv2.error, ValueError, tf2_ros.TransformException) as exc:
            self.camera_points[index] = None
            self.camera_stamps[index] = None
            self.get_logger().warning(
                f'Could not project {self.camera_names[index]} mask: {exc}',
                throttle_duration_sec=5.0)
            self.publish_combined()
            return
        self.publish_combined()

        self.height, self.theta = height_and_angle_from_transform(transform)
        self.geometry_ready = True
        self.get_logger().info(f'Camera geometry from TF: height={self.height:.3f} m, angle={self.theta:.2f} deg')
        self.try_build_ground_grid()
        self.geometry_timer.cancel()

    def try_build_ground_grid(self):
        if self.have_intrinsics and self.geometry_ready:
            self.recompute()

    def try_update_lidar_geometry(self):
        if self.lidar_extrinsic_ready:
            self.lidar_geometry_timer.cancel()
            return
        try:
            transform = self.tf_buffer.lookup_transform(self.lidar_frame_id, self.frame_id, Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().info(
                f'Waiting on TF {self.lidar_frame_id} -> {self.frame_id}: {ex}', throttle_duration_sec=5.0)
            return

        self.r_lidar_from_cam, self.t_lidar_from_cam = rotation_and_translation(transform)
        self.lidar_extrinsic_ready = True
        self.lidar_geometry_timer.cancel()

    def lidar_cb(self, msg: PointCloud2):
        if not self.lidar_extrinsic_ready:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        if len(points) == 0:
            return
        # skip_nans only drops NaN rows -- a no-return ray (out of range, e.g. sky) comes back as
        # +/-Inf instead, so drop those too, and only fit the plane to nearby points: a point far
        # enough away that the ground has curved or sloped out from under it shouldn't have to be
        # coplanar with the ground right around the robot, and letting it count would just dilute
        # the inlier fraction and make a good nearby plane look like a bad one.
        points = points[np.isfinite(points).all(axis=1)]
        r = np.linalg.norm(points, axis=1)
        points = points[(r > self.lidar_min_range) & (r < self.ground_plane_range)]
        if len(points) == 0:
            return
        plane = fit_ground_plane(points, distance_threshold=self.ground_plane_distance)
        if plane is None:
            return
        self.obstacle_xy = classify_obstacles(points, plane, self.obstacle_height)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'detect_potholes':
                self.detect_potholes = param.value
        return SetParametersResult(successful=True)

    def mask_cb(self, msg: CompressedImage):
        if self.u_valid is None:
            return

        mask = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        xy = mask_to_ground_xy(mask, self.u_valid, self.v_valid, self.Xg, self.Zg)

        lane_points = raycast(xy, self.max_range, self.height, self.theta)
        if self.lidar_extrinsic_ready:
            lane_points = reject_near_obstacles(
                lane_points, self.r_lidar_from_cam, self.t_lidar_from_cam,
                self.obstacle_xy, self.obstacle_reject_radius)
        self.publish_point_cloud(self.lane_pointcloud, lane_points, msg.header.stamp)

        if self.detect_potholes:
            pothole_points = find_potholes(xy, self.max_range, self.height, self.theta)
            if self.lidar_extrinsic_ready:
                pothole_points = reject_near_obstacles(
                    pothole_points, self.r_lidar_from_cam, self.t_lidar_from_cam,
                    self.obstacle_xy, self.obstacle_reject_radius)
            self.publish_point_cloud(self.pothole_pointcloud, pothole_points, msg.header.stamp)

    def publish_point_cloud(self, publisher, points, stamp):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self.points_pub.publish(point_cloud2.create_cloud(header, fields, xyz.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = LaneProjection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
