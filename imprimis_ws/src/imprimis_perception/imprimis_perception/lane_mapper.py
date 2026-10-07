"""Lane mapper: white tape on the ground, published as points the cost maps can use.

This is the navigation-facing companion of lane_detection.py (the team's prototype, left as it is).
The differences:
  - Each white pixel is placed with the depth image and kept only if it lies on the ground, so a
    pale ramp or the white bands on a barrel are not mistaken for tape. Without a depth image the
    node falls back to intersecting the pixel's ray with the ground plane.
  - Wide white areas are dropped; only narrow marks survive.
  - The camera pose comes from TF and the tilt from the IMU, instead of fixed numbers.
  - Output is in the base frame, a little above the ground (output_z), which is what the Nav2
    obstacle layer needs. Add the topic to a cost map as an observation source.

Topics out:
  perception/lane_points (PointCloud2, base frame)
  perception/lane_mask (Image, mono8)   the white mask, for checking the thresholds
  perception/lane_info (String)         JSON: cells, left_offset, right_offset (meters to each line)
"""
import json
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import Empty, String
import tf2_ros

from imprimis_perception import surface_tools as st

LINK_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
# Sensor topics: best effort, and only the newest message is kept. A frame that had to wait is worthless.
LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


class LaneMapper(Node):
    def __init__(self):
        super().__init__('lane_mapper')
        p = self.declare_parameter
        self.base_frame = p('base_frame', 'base_link').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.ground_frame = p('ground_frame', 'base_footprint').value
        self.ground_z = p('ground_z', -0.1277).value
        self.imu_yaw_in_base = p('imu_yaw_in_base', 0.0).value
        self.rate = p('process_rate_hz', 5.0).value
        self.horizontal_fov = p('horizontal_fov', 1.52).value
        self.camera_offset_x = p('camera_offset_x', 0.05).value
        self.optical_frame = p('image_in_optical_frame', False).value
        self.work_width = p('work_width', 640).value
        self.max_saturation = p('max_saturation', 60).value
        self.min_value = p('min_value', 200).value
        self.use_depth = p('use_depth', True).value
        self.height_tolerance = p('height_tolerance', 0.07).value
        self.max_range = p('max_range', 4.0).value   # beyond about 4 m one pixel covers too much ground
        self.max_width = p('max_line_width', 0.45).value
        self.max_pitch_deg = p('max_pitch_deg', 3.0).value
        self.output_z = p('output_z', 0.5).value
        self.publish_mask = p('publish_mask', True).value
        self.max_age = p('max_frame_age', 0.5).value   # seconds; older camera frames are skipped
        # Potholes. In the IGVC they are white circles 2 ft across painted on the ground; this node also looks for
        # real holes, where the depth camera sees the ground farther away than flat ground would be.
        self.detect_potholes = bool(p('detect_potholes', True).value)
        self.detect_drops = bool(p('detect_drop_offs', True).value)
        self.pothole_range = p('pothole_range', 3.5).value      # m ahead; farther away the pixels are too coarse
        self.min_drop = p('min_drop', 0.12).value               # m of depth that makes a hole
        self.candidates = []        # potholes seen in recent frames: (time, x, y in the odom frame, kind)
        self.image_age = None
        self.stale = 0

        self.roll = 0.0
        self.pitch = 0.0
        self.depth = None
        self.depth_stamp = None
        self.depth_intr = None
        self.zone = None
        self.static_tf = {}
        self.last_time = None
        self.sensor_time = 0.0      # newest stamp seen on the IMU; see ramp_detector.now_seconds

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.points_pub = self.create_publisher(PointCloud2, 'perception/lane_points', 5)
        self.mask_pub = self.create_publisher(Image, 'perception/lane_mask', 2)
        self.info_pub = self.create_publisher(String, 'perception/lane_info', 5)
        self.hole_points_pub = self.create_publisher(PointCloud2, 'perception/pothole_points', 5)
        self.hole_info_pub = self.create_publisher(String, 'perception/pothole_info', 5)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Imu, p('imu_topic', 'imu/data').value, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Image, p('depth_topic', 'cameras/front/depth/image_raw').value, self.depth_cb, LATEST)
        self.create_subscription(Image, p('image_topic', 'cameras/front/color/image_raw').value, self.image_cb, LATEST)
        self.create_subscription(PolygonStamped, 'ramp/zone', self.zone_cb, latched)
        self.create_subscription(Empty, 'lap/reset', self.reset_cb, 5)
        self.get_logger().info('Lane mapper started.')

    def lookup_static(self, child):
        if child in self.static_tf:
            return self.static_tf[child]
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, child, Time())
        except Exception:
            return None
        q, v = t.transform.rotation, t.transform.translation
        result = (st.quat_to_matrix(q.x, q.y, q.z, q.w), np.array([v.x, v.y, v.z]))
        self.static_tf[child] = result
        if child == self.ground_frame:
            self.ground_z = float(v.z)
        return result

    def imu_cb(self, msg):
        self.sensor_time = max(self.sensor_time, msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        q = msg.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) < 1e-6:
            return
        self.roll, self.pitch = st.roll_pitch_of_base((q.x, q.y, q.z, q.w), self.imu_yaw_in_base)

    def depth_cb(self, msg):
        if msg.encoding == '32FC1':
            self.depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == '16UC1':
            d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            d[d == 0] = np.nan
            self.depth = d
        else:
            return
        self.depth_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.depth_intr is None:
            self.depth_intr = st.intrinsics_from_fov(msg.width, msg.height, self.horizontal_fov)

    def zone_cb(self, msg):
        if len(msg.polygon.points) < 3:
            self.zone = None        # the ramp detector was reset and no longer knows where the ramp is
            return
        self.zone = (msg.header.frame_id, np.array([[pt.x, pt.y] for pt in msg.polygon.points]))

    def reset_cb(self, _msg):
        """A new run starts: drop everything held over from the last one."""
        self.depth = None
        self.depth_stamp = None
        self.zone = None
        self.last_time = None
        self.stale = 0
        self.candidates = []

    def seen_before(self, hx, hy, kind, now):
        """True if a pothole of this kind was seen at this place in an earlier frame of the last two seconds.
        One frame alone marks nothing in the cost maps, because a mark there stays."""
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except Exception:
            return False
        q, v = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        ox = v.x + math.cos(yaw) * hx - math.sin(yaw) * hy
        oy = v.y + math.sin(yaw) * hx + math.cos(yaw) * hy
        self.candidates = [c for c in self.candidates if 0.0 <= now - c[0] <= 2.0]
        known = any(c[3] == kind and math.hypot(c[1] - ox, c[2] - oy) < 0.45 for c in self.candidates)
        self.candidates.append((now, ox, oy, kind))
        return known

    def cloud_of(self, cells, stamp):
        pts = np.zeros((len(cells), 3), np.float32)
        if len(cells):
            pts[:, :2] = cells
        pts[:, 2] = self.output_z
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.base_frame
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate('xyz')]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(pts)
        cloud.is_dense = True
        cloud.data = pts.tobytes()
        return cloud

    def zone_in_base(self):
        """The ramp area as a polygon in the base frame, or None."""
        if self.zone is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.zone[0], Time())
        except Exception:
            return None
        q, v = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        pts = self.zone[1]
        return np.stack([c * pts[:, 0] - s * pts[:, 1] + v.x, s * pts[:, 0] + c * pts[:, 1] + v.y], axis=-1)

    def image_cb(self, msg):

        # Guard clauses 
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        now = max(self.sensor_time, stamp)
        if self.last_time is not None and 0 <= now - self.last_time < 1.0 / self.rate:
            return
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        self.image_age = now - stamp
        if self.image_age > self.max_age:
            self.stale += 1        # the robot has moved on; these points would be marked in the wrong place
            return
        tf = self.lookup_static(msg.header.frame_id)
        self.lookup_static(self.ground_frame)
        if tf is None:
            return

        # 
        self.last_time = now
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        if msg.encoding == 'bgr8':
            image = image[:, :, ::-1]
        scale = min(1.0, self.work_width / float(msg.width))
        if scale < 1.0:
            image = cv2.resize(image, (int(msg.width * scale), int(msg.height * scale)), interpolation=cv2.INTER_AREA)
        intr = st.intrinsics_from_fov(image.shape[1], image.shape[0], self.horizontal_fov)
        mask = st.white_mask(np.ascontiguousarray(image), self.max_saturation, self.min_value)
        if self.publish_mask and self.mask_pub.get_subscription_count() > 0:
            out = Image()
            out.header = msg.header
            out.height, out.width = mask.shape
            out.encoding = 'mono8'
            out.step = mask.shape[1]
            out.data = mask.tobytes()
            self.mask_pub.publish(out)

        cells = np.zeros((0, 2))
        holes = []
        hole_cells = np.zeros((0, 2))
        if abs(math.degrees(self.pitch)) <= self.max_pitch_deg:  # on the ramp the ground ahead is not the wheel plane
            depth = None
            if self.use_depth and self.depth is not None and abs(stamp - self.depth_stamp) < 0.5:
                depth = self.depth
            r_cam = tf[0] @ LINK_FROM_OPTICAL if self.optical_frame else tf[0]
            t_cam = tf[1] + tf[0] @ np.array([self.camera_offset_x, 0.0, 0.0])
            r_level = st.level_matrix(self.roll, self.pitch)
            with np.errstate(invalid='ignore'):
                xy = st.mask_to_ground_points(mask, intr, depth, self.depth_intr, r_cam, t_cam, r_level,
                                              self.ground_z, self.height_tolerance, self.max_range)
            cells = st.thin_structures(xy, x_range=(0.0, self.max_range + 0.5), max_width=self.max_width)
            zone = self.zone_in_base()
            self.get_logger().info(f'mask px={int(np.count_nonzero(mask))}  ground pts={len(xy)}  thin={len(cells)}  zone={zone is not None}')
            if len(cells):
                cells = cells[st.points_outside_polygon(cells, zone)]
            if self.detect_potholes:
                found = [('painted', hole) for hole in st.find_white_discs(xy, x_range=(0.0, self.pothole_range))]
                if self.detect_drops and depth is not None:
                    with np.errstate(invalid='ignore'):
                        dropped = st.ground_drop_points(depth, self.depth_intr, r_cam, t_cam, r_level, self.ground_z,
                                                        x_range=(0.7, self.pothole_range), min_drop=self.min_drop)
                    found += [('drop', hole) for hole in st.cluster_drops(dropped, x_range=(0.0, self.pothole_range))]
                for kind, (hx, hy, diameter, covered) in found:
                    if not st.points_outside_polygon(np.array([[hx, hy]]), zone)[0]:
                        continue            # on the ramp, where the ground is not the wheel plane
                    if not self.seen_before(hx, hy, kind, now):
                        continue            # first sighting: wait for the next frame to agree
                    holes.append({'x': round(hx, 2), 'y': round(hy, 2), 'diameter': round(diameter, 2), 'kind': kind})
                    hole_cells = np.concatenate([hole_cells, covered[::2]])

        pts = np.zeros((len(cells), 3), np.float32)
        pts[:, :2] = cells
        pts[:, 2] = self.output_z
        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = self.base_frame
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate('xyz')]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(pts)
        cloud.is_dense = True
        cloud.data = pts.tobytes()
        self.points_pub.publish(cloud)

        self.hole_points_pub.publish(self.cloud_of(hole_cells, msg.header.stamp))
        self.hole_info_pub.publish(String(data=json.dumps({'potholes': holes})))

        left = cells[cells[:, 1] > 0.3][:, 1] if len(cells) else np.zeros(0)
        right = cells[cells[:, 1] < -0.3][:, 1] if len(cells) else np.zeros(0)
        info = {'cells': int(len(cells)), 'image_age_s': round(self.image_age, 2), 'stale_frames': self.stale,
                'left_offset': round(float(np.min(left)), 2) if left.size else None,
                'right_offset': round(float(-np.max(right)), 2) if right.size else None}
        self.info_pub.publish(String(data=json.dumps(info)))


def main(args=None):
    rclpy.init(args=args)
    node = LaneMapper()
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
