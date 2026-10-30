import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, PointField, PointCloud2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
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

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_cb, 1)
        self.mask_sub = self.create_subscription(CompressedImage, mask_topic, self.mask_cb, 1)
        self.lidar_sub = self.create_subscription(PointCloud2, lidar_topic, self.lidar_cb, 1)

        self.lane_pointcloud = self.create_publisher(PointCloud2, 'perception/lane_points', 1)
        self.pothole_pointcloud = self.create_publisher(PointCloud2, 'perception/pothole_points', 1)

        # the mounts are fixed, so these only have to succeed once; they retry because TF
        # catches up after the drivers finish starting, which can be later than this node
        self.geometry_timer = self.create_timer(0.5, self.try_update_geometry)
        self.lidar_geometry_timer = self.create_timer(0.5, self.try_update_lidar_geometry)

        self.get_logger().info(
            f'Lane projection started. Waiting on {mask_topic}, {camera_info_topic}, '
            f'and TF {self.base_frame_id} -> {self.frame_id}'
        )

    def camera_info_cb(self, msg: CameraInfo):
        if self.have_intrinsics:
            return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.image_width = msg.width
        self.image_height = msg.height
        self.have_intrinsics = True
        self.try_build_ground_grid()

    def try_update_geometry(self):
        if self.geometry_ready:
            self.geometry_timer.cancel()
            return
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame_id, self.frame_id, Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().info(
                f'Waiting on TF {self.base_frame_id} -> {self.frame_id}: {ex}', throttle_duration_sec=5.0)
            return

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
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = stamp
        publisher.publish(point_cloud2.create_cloud(header, fields, points))

    def recompute(self):
        self.u_valid, self.v_valid, self.Xg, self.Zg = build_ground_grid(
            self.camera_matrix, self.image_width, self.image_height,
            self.height, self.theta, self.max_range)


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
