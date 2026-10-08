"""Ramp detection and ramp-aware LiDAR filtering.

Why this node exists. The odometry filter never estimates roll or pitch, so the navigation stack
treats base_link as level at all times. On the ramp the robot is tilted about 8.5 degrees and the
LiDAR's ground returns come out half a meter "high". The cost maps mark them as a wall across the
lane, and because the cost maps never clear, the wall stays. The ramp surface itself is also marked
near its ridge.

What it does.
  1. Reads roll and pitch from the IMU.
  2. Rotates every LiDAR cloud into a true-vertical frame, removes ground and ramp-surface returns,
     and publishes what is left (the obstacles) on cloud_out. The cost maps read that topic.
  3. Looks ahead with the depth camera for a steady climb, which is how a ramp differs from a barrel.
  4. Publishes the ramp state, grade and the area of the ramp, for the lane mapper, the control
     window and the lap records.

Topics out:
  cloud_out (PointCloud2, base frame)  obstacles only
  ramp/state (String)   flat | ramp_ahead | climbing | descending
  ramp/detected (Bool)  true in every state except flat
  ramp/info (String)    JSON: state, pitch_deg, roll_deg, grade, distance
  ramp/zone (PolygonStamped, odom frame, latched)  the ramp area
"""
import json
import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import Bool, Empty, String
import tf2_ros

from imprimis_perception import surface_tools as st

LINK_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
# Sensor topics: best effort, and only the newest message is kept. A frame that had to wait is worthless.
LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


def stamp_from_seconds(seconds):
    """A ROS time stamp from seconds on the sensors' clock."""
    stamp = TimeMsg()
    stamp.sec = int(seconds)
    stamp.nanosec = int((seconds - int(seconds)) * 1e9)
    return stamp


class RampDetector(Node):
    def __init__(self):
        super().__init__('ramp_detector')
        p = self.declare_parameter
        self.base_frame = p('base_frame', 'base_link').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.ground_frame = p('ground_frame', 'base_footprint').value
        self.ground_z = p('ground_z', -0.1277).value  # used until the ground frame is found in TF
        self.imu_yaw_in_base = p('imu_yaw_in_base', 0.0).value
        self.imu_time_constant = p('imu_time_constant', 0.08).value
        # LiDAR filter
        self.min_height = p('min_height', 0.15).value
        self.max_height = p('max_height', 2.0).value
        self.surface_max_height = p('surface_max_height', 0.60).value
        self.max_surface_grade = p('max_surface_grade', 0.35).value
        self.lidar_rings = p('lidar_rings', 16).value
        self.lidar_columns = p('lidar_columns', 1800).value
        # depth look-ahead
        self.depth_rate = p('depth_rate_hz', 5.0).value
        self.horizontal_fov = p('horizontal_fov', 1.52).value
        self.camera_offset_x = p('camera_offset_x', 0.05).value
        self.optical_frame = p('depth_in_optical_frame', False).value
        self.min_grade = p('min_grade', 0.07).value
        self.max_grade = p('max_grade', 0.30).value
        # state machine
        self.on_ramp_deg = p('on_ramp_pitch_deg', 4.0).value
        self.off_ramp_deg = p('off_ramp_pitch_deg', 2.0).value
        self.settle_time = p('settle_time', 0.5).value
        self.zone_length = p('default_zone_length', 6.5).value
        self.zone_half_width = p('default_zone_half_width', 2.6).value
        self.max_age = p('max_frame_age', 0.5).value   # seconds; older camera frames are skipped

        self.roll = 0.0
        self.pitch = 0.0
        self.have_imu = False
        self.last_imu_time = None
        self.state = 'flat'
        self.flat_since = None
        self.ahead = None            # latest depth detection
        self.ahead_time = None
        self.zone = None             # dict x, y, heading, length, half_width, seen_from, confirmed
        self.static_tf = {}
        self.last_depth_time = None
        self.depth_intr = None
        self.clouds = 0
        self.sensor_time = 0.0      # newest stamp seen on the IMU or the LiDAR
        self.depth_age = None
        self.cloud_age = None
        self.stale = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2, p('cloud_out', 'velodyne_points_nav').value, 5)
        self.state_pub = self.create_publisher(String, 'ramp/state', latched)
        self.detected_pub = self.create_publisher(Bool, 'ramp/detected', latched)
        self.info_pub = self.create_publisher(String, 'ramp/info', 5)
        self.zone_pub = self.create_publisher(PolygonStamped, 'ramp/zone', latched)

        self.create_subscription(Imu, p('imu_topic', 'imu/data').value, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, p('cloud_in', 'velodyne_points').value, self.cloud_cb, LATEST)
        self.create_subscription(Image, p('depth_topic', 'cameras/front/depth/image_raw').value, self.depth_cb, LATEST)
        self.create_subscription(Empty, 'lap/reset', self.reset_cb, 5)
        self.create_timer(0.1, self.update_state)
        self.publish_state()
        self.get_logger().info('Ramp detector started.')

    # ------------------------------------------------------------------ helpers
    def reset_cb(self, _msg):
        """A new run starts: forget where the ramp was seen. Its place was kept in the odometry frame, which has
        drifted since; the ramp will be found again when the robot comes to it."""
        self.ahead = None
        self.ahead_time = None
        self.zone = None
        self.last_depth_time = None
        self.stale = 0
        empty = PolygonStamped()
        empty.header.frame_id = self.odom_frame
        empty.header.stamp = stamp_from_seconds(self.sensor_time)
        self.zone_pub.publish(empty)

    def lookup_static(self, child):
        """Rotation matrix and translation of a frame fixed to the robot, in the base frame. Cached."""
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

    def robot_pose(self):
        """x, y, yaw of the base frame in the odom frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except Exception:
            return None
        q, v = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return v.x, v.y, yaw

    def now_seconds(self):
        """The time on the sensors' clock, simulated or real: the newest stamp on the IMU or the LiDAR.
        Taking it from the data keeps this node off the simulator's clock topic, which arrives several
        hundred times a second and costs a Python node about half a processor."""
        return self.sensor_time

    # ------------------------------------------------------------------ IMU
    def imu_cb(self, msg):
        self.sensor_time = max(self.sensor_time, msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        q = msg.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) < 1e-6:
            return  # this IMU does not report orientation
        roll, pitch = st.roll_pitch_of_base((q.x, q.y, q.z, q.w), self.imu_yaw_in_base)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if not self.have_imu or self.last_imu_time is None or t <= self.last_imu_time:
            self.roll, self.pitch = roll, pitch
        else:
            dt = t - self.last_imu_time
            a = dt / (self.imu_time_constant + dt)
            self.roll += a * (roll - self.roll)
            self.pitch += a * (pitch - self.pitch)
        self.last_imu_time = t
        self.have_imu = True

    # ------------------------------------------------------------------ LiDAR
    def cloud_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.sensor_time = max(self.sensor_time, stamp)
        self.cloud_age = self.sensor_time - stamp
        tf = self.lookup_static(msg.header.frame_id)
        self.lookup_static(self.ground_frame)
        if tf is None:
            return
        offsets = {f.name: f.offset for f in msg.fields}
        if not all(k in offsets for k in ('x', 'y', 'z')):
            return
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4', '<f4', '<f4'],
                          'offsets': [offsets['x'], offsets['y'], offsets['z']], 'itemsize': msg.point_step})
        raw = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
        pts = np.stack([raw['x'], raw['y'], raw['z']], axis=-1)
        if msg.height > 1:
            organized = pts.reshape(msg.height, msg.width, 3)
        elif 'ring' in offsets:
            ring_dtype = np.dtype({'names': ['ring'], 'formats': ['<u2'], 'offsets': [offsets['ring']],
                                   'itemsize': msg.point_step})
            ring = np.frombuffer(msg.data, dtype=ring_dtype, count=msg.width)['ring'].astype(int)
            organized = st.organize_by_ring(pts, ring, self.lidar_rings, self.lidar_columns)
        else:
            self.get_logger().warn('LiDAR cloud is neither organized nor carries a ring field; passing it through.',
                                   throttle_duration_sec=10.0)
            self.cloud_pub.publish(msg)
            return
        r_level = st.level_matrix(self.roll, self.pitch)
        rot = r_level @ tf[0]
        with np.errstate(invalid='ignore'):
            level = (organized.reshape(-1, 3).astype(np.float64) @ rot.T + r_level @ tf[1]).reshape(organized.shape)
        obstacle, _ = st.classify_lidar(level, self.ground_z, self.min_height, self.max_height,
                                        self.surface_max_height, self.max_surface_grade)
        out = level[obstacle].astype(np.float32)
        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = self.base_frame
        cloud.height = 1
        cloud.width = len(out)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate('xyz')]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(out)
        cloud.is_dense = True
        cloud.data = out.tobytes()
        self.cloud_pub.publish(cloud)
        self.clouds += 1

    # ------------------------------------------------------------------ depth camera
    def depth_cb(self, msg):
        now = max(self.now_seconds(), msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        if self.last_depth_time is not None and 0 <= now - self.last_depth_time < 1.0 / self.depth_rate:
            return
        if msg.encoding not in ('32FC1', '16UC1'):
            return
        self.depth_age = now - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        if self.depth_age > self.max_age:
            self.stale += 1        # the ramp may be somewhere else by now
            return
        tf = self.lookup_static(msg.header.frame_id)
        if tf is None:
            return
        self.last_depth_time = now
        if msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        else:  # RealSense depth in millimeters
            depth = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            depth[depth == 0] = np.nan
        if self.depth_intr is None:
            self.depth_intr = st.intrinsics_from_fov(msg.width, msg.height, self.horizontal_fov)
        rows, cols = np.mgrid[0:msg.height:4, 0:msg.width:4]
        with np.errstate(invalid='ignore'):
            link, ok = st.depth_to_link_points(depth, self.depth_intr, rows.ravel(), cols.ravel())
        r_cam = tf[0] @ LINK_FROM_OPTICAL if self.optical_frame else tf[0]
        t_cam = tf[1] + tf[0] @ np.array([self.camera_offset_x, 0.0, 0.0])
        r_level = st.level_matrix(self.roll, self.pitch)
        pts = link[ok] @ (r_level @ r_cam).T + r_level @ t_cam
        centers, heights = st.ground_profile(pts, self.ground_z)
        ramp = st.find_ramp(centers, heights, self.min_grade, self.max_grade)
        if ramp is None:
            return
        half = st.ramp_half_width(pts, self.ground_z, ramp['distance'], ramp['distance'] + ramp['length'] + 0.5)
        ramp['half_width'] = half
        self.ahead = ramp
        self.ahead_time = now
        if abs(math.degrees(self.pitch)) < self.off_ramp_deg:
            self.note_ramp_start(ramp['distance'], half, seen_from=ramp['distance'])

    # ------------------------------------------------------------------ ramp area
    def note_ramp_start(self, distance, half_width, seen_from):
        pose = self.robot_pose()
        if pose is None:
            return
        x, y, yaw = pose
        sx, sy = x + distance * math.cos(yaw), y + distance * math.sin(yaw)
        half = self.zone_half_width if half_width is None else max(1.0, half_width + 0.25)
        z = self.zone
        if z is not None and math.hypot(sx - z['x'], sy - z['y']) < 3.0:
            if z['confirmed'] or seen_from >= z['seen_from']:
                return  # the estimate made from closer by is the better one
            length = z['length']
        else:
            length = self.zone_length
            self.get_logger().info('Ramp found %.1f m ahead.' % distance)
        self.zone = {'x': sx, 'y': sy, 'heading': yaw, 'length': length, 'half_width': half,
                     'seen_from': seen_from, 'confirmed': False}
        self.publish_zone()

    def publish_zone(self):
        z = self.zone
        if z is None:
            return
        c, s = math.cos(z['heading']), math.sin(z['heading'])
        msg = PolygonStamped()
        msg.header.frame_id = self.odom_frame
        msg.header.stamp = stamp_from_seconds(self.sensor_time)
        for along, across in ((-0.3, -z['half_width']), (z['length'], -z['half_width']),
                              (z['length'], z['half_width']), (-0.3, z['half_width'])):
            msg.polygon.points.append(Point32(x=float(z['x'] + along * c - across * s),
                                              y=float(z['y'] + along * s + across * c), z=0.0))
        self.zone_pub.publish(msg)

    # ------------------------------------------------------------------ state
    def update_state(self):
        now = self.now_seconds()
        pitch_deg = math.degrees(self.pitch)
        previous = self.state
        if pitch_deg <= -self.on_ramp_deg:
            self.state = 'climbing'
            self.flat_since = None
        elif pitch_deg >= self.on_ramp_deg:
            self.state = 'descending'
            self.flat_since = None
        elif abs(pitch_deg) < self.off_ramp_deg:
            if self.state in ('climbing', 'descending'):
                if self.flat_since is None:
                    self.flat_since = now
                elif now - self.flat_since >= self.settle_time or now < self.flat_since:
                    if self.state == 'descending':
                        self.close_zone()
                    self.state = 'flat'
            if self.state in ('flat', 'ramp_ahead'):
                recent = self.ahead_time is not None and 0 <= now - self.ahead_time < 1.0
                self.state = 'ramp_ahead' if recent else 'flat'
        if self.state == 'climbing' and previous in ('flat', 'ramp_ahead'):
            pose = self.robot_pose()
            near = False
            if pose is not None and self.zone is not None:
                near = math.hypot(pose[0] - self.zone['x'], pose[1] - self.zone['y']) < 4.0
            if not near:
                self.note_ramp_start(-0.5, None, seen_from=0.0)  # climbed a ramp the camera did not announce
        if self.state != previous:
            self.get_logger().info('Ramp state: %s (pitch %.1f deg)' % (self.state, pitch_deg))
            self.publish_state()
        info = {'state': self.state, 'pitch_deg': round(pitch_deg, 2), 'roll_deg': round(math.degrees(self.roll), 2),
                'grade': None, 'distance': None, 'imu': self.have_imu, 'clouds': self.clouds,
                'depth_age_s': None if self.depth_age is None else round(self.depth_age, 2),
                'cloud_age_s': None if self.cloud_age is None else round(self.cloud_age, 2), 'stale_frames': self.stale}
        if self.state == 'ramp_ahead' and self.ahead is not None:
            info['grade'] = round(self.ahead['grade'], 3)
            info['distance'] = round(self.ahead['distance'], 2)
        elif self.state in ('climbing', 'descending'):
            info['grade'] = round(abs(math.tan(self.pitch)), 3)
        self.info_pub.publish(String(data=json.dumps(info)))

    def close_zone(self):
        """The robot has come off the far side: the ramp ends about where the robot is now."""
        pose = self.robot_pose()
        z = self.zone
        if pose is None or z is None:
            return
        along = (pose[0] - z['x']) * math.cos(z['heading']) + (pose[1] - z['y']) * math.sin(z['heading'])
        if 1.0 < along < 15.0:
            z['length'] = along + 0.3
            z['confirmed'] = True
            self.get_logger().info('Ramp crossed. Measured length %.1f m.' % z['length'])
            self.publish_zone()

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))
        self.detected_pub.publish(Bool(data=self.state != 'flat'))


def main(args=None):
    rclpy.init(args=args)
    node = RampDetector()
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
