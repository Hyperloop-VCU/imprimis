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


CAMERA_COUNT = 4


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

        self.camera_names = list(self.get_parameter('camera_names').value)
        self.mask_topics = list(self.get_parameter('mask_topics').value)
        self.info_topics = list(self.get_parameter('camera_info_topics').value)
        if not (len(self.camera_names) == len(self.mask_topics) == len(self.info_topics) == CAMERA_COUNT):
            raise ValueError('camera_names, mask_topics, and camera_info_topics must each have exactly four entries')
        if len(set(self.camera_names)) != CAMERA_COUNT:
            raise ValueError('camera_names must be unique')
        self.base_frame = self.get_parameter('base_frame').value
        self.ground_frame = self.get_parameter('ground_frame').value
        self.ground_z = float(self.get_parameter('ground_z').value)
        self.min_forward = float(self.get_parameter('min_forward').value)
        self.max_forward = float(self.get_parameter('max_forward').value)
        self.half_width = float(self.get_parameter('half_width').value)
        self.pixel_step = int(self.get_parameter('pixel_step').value)
        self.grid_cell = float(self.get_parameter('grid_cell').value)
        self.max_line_width = float(self.get_parameter('max_line_width').value)
        self.max_camera_age = float(self.get_parameter('max_camera_age').value)
        self.output_z = float(self.get_parameter('output_z').value)
        if (not all(math.isfinite(value) for value in (
                self.ground_z, self.min_forward, self.max_forward, self.half_width,
                self.grid_cell, self.max_line_width, self.max_camera_age, self.output_z))
                or self.min_forward < 0 or self.max_forward <= self.min_forward
                or self.half_width <= 0 or self.pixel_step < 1
                or self.grid_cell <= 0 or self.max_line_width <= 0
                or self.max_camera_age <= 0):
            raise ValueError('invalid projection distance, width, cell, sample-step, or freshness parameter')

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

    def publish_combined(self):
        fresh = []
        for points, seen in zip(self.camera_points, self.camera_stamps):
            if points is not None and seen is not None and self.latest_stamp - seen <= self.max_camera_age:
                if len(points):
                    fresh.append(points)
        if fresh:
            xy = np.concatenate(fresh, axis=0)
            points = st.thin_structures(
                xy,
                cell=self.grid_cell,
                x_range=(self.min_forward, self.max_forward),
                half_width=self.half_width,
                max_width=self.max_line_width,
            )
            xyz = np.column_stack((points, np.full(len(points), self.output_z)))
        else:
            xyz = np.zeros((0, 3), dtype=np.float32)
        header = Header()
        header.stamp = self.latest_stamp_msg
        header.frame_id = self.base_frame
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
