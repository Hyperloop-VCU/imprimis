"""Speed governor: as fast as the rules allow where the way is clear, slower where it is not.

The IGVC limit is 5 mph (2.235 m/s). Nav2's controller is allowed that speed, and this node tells it,
ten times a second, how much of it to use. It looks along the path Nav2 has planned and asks four
questions:

  Obstacles.  How close do barrels and lane lines come to the path ahead? With plenty of room the limit
              is the top speed. In a tight gap it is the slow speed. In between it is in between.
  Turns.      How sharp is the path ahead? The limit keeps the sideways acceleration modest.
  Ramp.       Is the ramp just ahead, or is the robot on it? The ramp is taken at ramp speed.
  Path end.   Is the path about to run out? Then the robot must be able to stop.

Each answer is a speed the robot must be down to when it gets there, so the limit now is that speed
plus what the brakes can take off on the way. The lowest of the four wins. The limit may fall at once;
it rises gradually.

Topics in:   plan (Path)                         the path Nav2 is following
             odometry/filtered/local             where the robot is
             velodyne_points_nav (PointCloud2)   LiDAR obstacle points from the ramp detector, base frame
             perception/lane_points (PointCloud2) lane line points from the lane mapper, base frame
             ramp/info (String, JSON)
Topics out:  speed_limit (nav2_msgs/SpeedLimit)   read by Nav2's controller server
             speed_governor/info (String, JSON)   limit and the reason, for the control window and the records
"""
import json
import math

import numpy as np
import rclpy
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, String

LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
MPH = 2.237


class SpeedGovernor(Node):
    def __init__(self):
        super().__init__('speed_governor')
        p = self.declare_parameter
        self.top = p('top_speed', 2.1).value                 # m/s on a clear stretch. 5 mph is 2.235; at 2.2 the true speed touched 5.06 mph
        # Tight gap and turn limits were 1.2 m/s and 1.5 m/s^2 at first. Laps were then fast but touched a lane line
        # in the same narrow place three times and stalled in the chicane of the north arm. Time lost in a stall is
        # far more than time lost by entering slowly.
        self.slow = p('tight_speed', 0.9).value              # m/s through a tight gap
        self.gap_clear = p('clear_gap', 1.0).value           # m between the robot's side and an obstacle: no slowing
        self.gap_tight = p('tight_gap', 0.25).value          # m: slow speed
        self.half_width = p('robot_half_width', 0.49).value
        self.brake = p('braking', 0.9).value                 # m/s^2 the governor counts on (the controller answers about half a second late)
        self.margin = p('braking_margin', 0.9).value         # m: be down to the speed this far before the place
        self.lateral = p('lateral_acceleration', 1.1).value  # m/s^2 allowed in a turn
        self.ramp_speed = p('ramp_speed', 1.2).value
        self.end_speed = p('path_end_speed', 0.8).value
        self.look = p('look_ahead', 7.0).value               # m of path examined
        self.rise = p('limit_rise', 1.0).value               # m/s^2: how fast the limit may come back up
        self.no_path_speed = p('no_path_speed', 1.0).value

        self.pose = None
        self.clock = 0.0
        self.path = None            # (n, 2) in the odom frame
        self.path_time = 0.0
        self.obstacles = np.zeros((0, 2))
        self.lanes = np.zeros((0, 2))
        self.lane_time = 0.0
        self.ramp = {}
        self.limit = self.slow
        self.last_tick = None

        self.pub = self.create_publisher(SpeedLimit, 'speed_limit', 5)
        self.info_pub = self.create_publisher(String, 'speed_governor/info', 5)
        self.create_subscription(Path, 'plan', self.path_cb, 5)
        self.create_subscription(Odometry, p('odom_topic', 'odometry/filtered/local').value, self.odom_cb, 20)
        self.create_subscription(PointCloud2, 'velodyne_points_nav', lambda m: self.cloud_cb(m, 'obstacles'), LATEST)
        self.create_subscription(PointCloud2, 'perception/lane_points', lambda m: self.cloud_cb(m, 'lanes'), LATEST)
        self.create_subscription(String, 'ramp/info', self.ramp_cb, 5)
        self.create_subscription(Empty, 'lap/reset', self.reset_cb, 5)
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Speed governor: top speed %.2f m/s (%.1f mph), %.1f m/s in tight gaps.' % (self.top, self.top * MPH, self.slow))

    # ------------------------------------------------------------------ inputs
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.twist.twist.linear.x)
        self.clock = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def path_cb(self, msg):
        if len(msg.poses) >= 2:
            self.path = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses])
            self.path_time = self.clock

    def cloud_cb(self, msg, kind):
        if self.pose is None or msg.point_step < 12:
            return
        pts = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)[:, :2]
        if len(pts) > 1200:
            pts = pts[::len(pts) // 1200 + 1]
        x, y, yaw, _ = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], axis=-1)
        if kind == 'obstacles':
            self.obstacles = world
        else:
            # lane points are only seen ahead of the camera; keep the recent ones so a line beside the robot still counts
            keep = self.lanes[-600:] if self.clock - self.lane_time < 5.0 else np.zeros((0, 2))
            self.lanes = np.concatenate([keep, world]) if len(world) else keep
            self.lane_time = self.clock

    def reset_cb(self, _msg):
        """A new run starts: drop the path, the obstacle points and the lane points kept from the last one."""
        self.path = None
        self.obstacles = np.zeros((0, 2))
        self.lanes = np.zeros((0, 2))
        self.ramp = {}

    def ramp_cb(self, msg):
        try:
            self.ramp = json.loads(msg.data)
        except ValueError:
            pass

    # ------------------------------------------------------------------ the limit
    def allowed_now(self, speed_there, distance):
        """Speed allowed here if the robot must be down to speed_there after distance (with a short margin)."""
        return math.sqrt(speed_there ** 2 + 2.0 * self.brake * max(0.0, distance - self.margin))

    def path_ahead(self):
        """The planned path from the robot onward, resampled every 0.25 m, with the distance along it."""
        if self.path is None or self.pose is None or self.clock - self.path_time > 3.0:
            return None
        d = np.hypot(self.path[:, 0] - self.pose[0], self.path[:, 1] - self.pose[1])
        start = int(np.argmin(d))
        pts = self.path[start:]
        if len(pts) < 2:
            return None
        seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(s[-1])
        if total < 0.3:
            return None
        grid = np.arange(0.0, min(total, self.look) + 1e-6, 0.25)
        return np.stack([np.interp(grid, s, pts[:, 0]), np.interp(grid, s, pts[:, 1])], axis=-1), grid, total

    def tick(self):
        if self.pose is None:
            return
        limit, reason = self.top, 'clear'
        ahead = self.path_ahead()
        if ahead is None:
            limit, reason = self.no_path_speed, 'no path yet'
        else:
            pts, grid, total = ahead
            # obstacles and lane lines beside the path
            cloud = np.concatenate([self.obstacles, self.lanes]) if len(self.lanes) else self.obstacles
            if len(cloud):
                near = np.hypot(cloud[:, 0] - self.pose[0], cloud[:, 1] - self.pose[1]) < self.look + 2.0
                cloud = cloud[near]
            if len(cloud):
                d = np.hypot(cloud[:, None, 0] - pts[None, :, 0], cloud[:, None, 1] - pts[None, :, 1])   # (points, path samples)
                nearest = d.argmin(axis=1)
                gap = d.min(axis=1) - self.half_width
                where = grid[nearest]
                frac = np.clip((gap - self.gap_tight) / (self.gap_clear - self.gap_tight), 0.0, 1.0)
                there = self.slow + frac * (self.top - self.slow)
                now = np.sqrt(there ** 2 + 2.0 * self.brake * np.maximum(0.0, where - self.margin))
                i = int(now.argmin())
                if now[i] < limit:
                    limit, reason = float(now[i]), 'obstacle %.1f m from the side of the robot, %.1f m ahead' % (max(0.0, gap[i]), where[i])
            # turns
            if len(pts) >= 5:
                heading = np.unwrap(np.arctan2(np.diff(pts[:, 1]), np.diff(pts[:, 0])))
                span = 4                                    # 1 m
                if len(heading) > span:
                    curvature = np.abs(heading[span:] - heading[:-span]) / (span * 0.25)
                    there = np.sqrt(self.lateral / np.maximum(curvature, 1e-3))
                    now = np.sqrt(np.minimum(there, self.top) ** 2 + 2.0 * self.brake * np.maximum(0.0, grid[:len(there)] - self.margin))
                    i = int(now.argmin())
                    if now[i] < limit:
                        limit, reason = float(now[i]), 'turn of %.1f m radius, %.1f m ahead' % (1.0 / max(curvature[i], 1e-3), grid[i])
            # the end of the path
            if total < self.look:
                now = self.allowed_now(self.end_speed, total)
                if now < limit:
                    limit, reason = now, 'path ends in %.1f m' % total
        # the ramp
        state = self.ramp.get('state', 'flat')
        if state in ('climbing', 'descending'):
            if self.ramp_speed < limit:
                limit, reason = self.ramp_speed, 'on the ramp'
        elif state == 'ramp_ahead' and self.ramp.get('distance') is not None:
            now = self.allowed_now(self.ramp_speed, float(self.ramp['distance']))
            if now < limit:
                limit, reason = now, 'ramp %.1f m ahead' % self.ramp['distance']
        limit = max(0.4, min(self.top, limit))
        # fall at once, rise gradually
        now_t = self.clock
        dt = 0.1 if self.last_tick is None else max(0.0, min(0.5, now_t - self.last_tick))
        self.last_tick = now_t
        if limit > self.limit:
            limit = min(limit, self.limit + self.rise * dt)
        self.limit = limit
        msg = SpeedLimit()
        msg.header.stamp.sec = int(self.clock)
        msg.header.stamp.nanosec = int((self.clock - int(self.clock)) * 1e9)
        msg.percentage = False
        msg.speed_limit = float(limit)
        self.pub.publish(msg)
        self.info_pub.publish(String(data=json.dumps({'limit_mps': round(limit, 2), 'limit_mph': round(limit * MPH, 1),
                                                      'reason': reason, 'top_mph': round(self.top * MPH, 1),
                                                      'speed_mph': round(self.pose[3] * MPH, 1)})))


def main(args=None):
    rclpy.init(args=args)
    node = SpeedGovernor()
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
