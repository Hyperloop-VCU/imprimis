"""Lap manager: drive mode, finish line, lap records, and the automatic lap driver.

Drive mode. "manual" means a person drives (the control window publishes velocity commands).
"automatic" means this node leads Nav2 along the remembered route.

How the robot is led. The route is the path of the best lap on record. The node keeps a goal a few
meters ahead of the robot on that route and moves it forward as the robot advances, so Nav2 always
has several meters of path in front of it and can hold its speed. The route is a guide, not a rail:
Nav2 plans its own way to the goal around whatever the LiDAR and the camera see. If the place where
the goal would go is occupied (a barrel was moved onto the route), the goal slides forward along the
route to the next free place. That is what lets a remembered lap work on a course that has changed.

Finish line. The lap ends when the robot crosses the finish line in the direction of travel after
passing every checkpoint in order. The node then cancels navigation and holds the robot still. A new
lap starts only when someone asks for one, so the robot never loops on its own.

Lap memory. Every run is saved. A completed lap that ranks ahead of the best on record becomes the
route for the next automatic run (see lap_core.RouteMemory).

Topics in:   drive_mode_request (String: manual | automatic)
             lap/command (String: new_lap | stop)
Topics out:  drive_mode (String, latched)
             lap/status (String, JSON, 5 Hz)
             lap/finished (Bool, latched)
             lap/route (Path, latched)  the remembered route
             lap/reset (Empty)          a new run begins: perception nodes drop what they hold
             lap/notice (String, JSON)  a line of text for the control window
"""
import json
import math
import os

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField
from std_msgs.msg import Bool, Empty, String
from tf2_msgs.msg import TFMessage

from imprimis_mission.lap_core import Course, LapTracker, RouteMemory, distance_to_loop, gps_to_course, route_points

LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
ROUTE_STEP = 0.5     # meters between route points


def stamp_from_seconds(seconds):
    """A ROS time stamp from seconds on the sensors' clock."""
    stamp = TimeMsg()
    stamp.sec = int(seconds)
    stamp.nanosec = int((seconds - int(seconds)) * 1e9)
    return stamp


class LapManager(Node):
    def __init__(self):
        super().__init__('lap_manager')
        p = self.declare_parameter
        self.course_file = p('course_file', '').value
        memory_dir = os.path.expanduser(p('memory_dir', '~/imprimis_lap_memory').value)
        self.lookahead = p('goal_lookahead', 7.0).value          # meters of route between the robot and its goal
        self.resend = p('goal_resend_distance', 2.5).value       # move the goal on after this much progress
        self.goal_clearance = p('goal_clearance', 0.9).value     # a goal this close to an obstacle point is "occupied"
        # The corridor can be narrower where the route is free than where something stands on it. That was tried with
        # 1.5 m (run 20), to keep the planner off a lane line the camera cannot see beside one barrel in the north arm.
        # The robot then could not get through the chicane of that arm: the route of one lap and the odometry of the
        # next differ by 0.3 to 0.5 m, and that is too much of 1.5 m. Both widths are therefore 2.6 m.
        self.corridor_half = p('corridor_half_width', 2.6).value        # m to each side of a free stretch of route
        self.corridor_wide = p('corridor_wide_half_width', 2.6).value   # m to each side where the route is blocked
        self.block_clearance = p('route_block_clearance', 0.6).value    # an obstacle this near a route point blocks it
        self.blocked = set()             # route indices found blocked during this lap
        # Remembered lane lines are stored in the odometry frame of the lap that saw them. Odometry drifts by
        # 1 to 2 m over a lap and not the same way twice, so in a narrow passage the remembered line can lie
        # across the gap the robot must take. They are therefore off unless asked for; the camera marks the
        # true lines as it sees them, and the route corridor keeps the planner near the route.
        self.use_lane_memory = bool(p('use_lane_memory', False).value)
        self.off_route_limit = p('off_route_limit', 4.5).value   # m from the route at which a lap is stopped
        self.lane_setback = p('lane_memory_setback', 0.35).value # remembered lane lines are moved this far away from the route
        self.stuck_timeout = p('stuck_timeout', 75.0).value
        self.max_lap_time = p('max_lap_time', 1200.0).value
        self.max_goal_failures = p('max_goal_failures', 3).value
        self.frame = p('goal_frame', 'odom').value
        self.mode = p('start_mode', 'manual').value
        self.autostart = p('autostart', False).value
        self.exit_when_done = p('exit_when_done', False).value
        # Arming check: before an automatic run sets off, the robot must show that it is where the course is.
        # Without it the robot drives the remembered route from wherever it happens to stand.
        self.arming_check = bool(p('arming_check', True).value)
        self.arming_window = p('arming_window', 8.0).value          # s the checks may take before the run is refused
        self.arming_require_gps = bool(p('arming_require_gps', True).value)
        # GPS boundary: while an automatic run is under way the GPS position must stay near the course line.
        self.boundary_check = bool(p('boundary_check', True).value)
        self.gps_loss_timeout = p('gps_loss_timeout', 5.0).value    # s without a fix that stops an automatic run; 0 turns this off

        self.course = None
        self.course_mtime = None
        self.tracker = None
        self.memory = None
        if self.course_file and os.path.isfile(self.course_file):
            self.course = Course.load(self.course_file)
            self.course_mtime = os.stat(self.course_file).st_mtime
            self.tracker = LapTracker(self.course)
            self.memory = RouteMemory(memory_dir)
            self.get_logger().info('Course "%s", layout %s; lap memory in %s' % (self.course.name, self.course.layout_id, memory_dir))
        else:
            self.get_logger().info('No course file: drive mode switching only, no lap logic.')

        self.pose = None            # x, y, yaw, speed
        self.truth = None           # x, y, yaw from the simulator, with the time it arrived
        self.truth_ok = None        # None until the first reading has been checked against the odometry
        self.truth_name = p('truth_model_name', 'imprimis').value
        self.sim_time = 0.0
        self.route = []             # (x, y, yaw) every ROUTE_STEP meters
        self.progress = 0           # index of the route point the robot has reached
        self.boost = 0.0            # extra lookahead after repeated failures, meters
        self.obstacles = None       # latest LiDAR obstacle points in the odom frame, (n, 2)
        self.lane_memory = np.zeros((0, 2))   # lane line cells remembered from an earlier lap, odom frame
        self.corridor = None        # the corridor points, built once per route
        self.goals_moved = 0        # how often the goal had to slide off an occupied place
        self.route_description = 'none'
        self.best_time = None
        self.goal_handle = None
        self.sent_index = None
        self.goal_failures = 0
        self.skipped = 0
        self.last_progress = None   # (time, x, y)
        self.headway = None         # (time, route index) when the robot last gained 2 m along the route
        self.route_distance = 0.0   # m from the robot to the nearest point of the route ahead
        self.hold_until = 0.0
        self.message = ''
        self.last_result = None
        self.new_best = False
        self.ramp_state = 'flat'
        self.lane_info = {}
        self.ticks = 0
        self.done = False
        self.run_reset_time = None  # when the short-lived data was last cleared
        self.run_reset_done = False  # true if that happened before the lap now running or armed
        self.potholes = []          # potholes detected on this run: dicts x, y (odom frame), diameter, kind, seen
        self.detours = 0            # plans refused on this lap because they went the long way round
        self.arming = 'off'         # off | checking | armed | refused
        self.arming_checks = []     # [name, passed, what was found] of the latest check
        self.arming_deadline = None
        self.arming_changed = 0.0
        self.arming_ticks = 0
        self.gps_fix = None         # (time, x, y in course coordinates, valid)
        self.boundary_distance = None   # m from the GPS position to the course line
        self.boundary_strikes = 0
        self.max_boundary = 0.0
        self.cloud_time = None      # when LiDAR points last arrived
        self.lane_time = None       # when the lane mapper last reported (the camera is alive)
        self.lane_sides = None      # (time, lane points to the left, lane points to the right)
        # The course frame. Odometry turns by about one degree per lap within a session, so its frame slowly
        # rotates away from the course. Everything here (route, corridor, lap logic, records) is kept in a
        # course frame instead: course pose = odometry pose turned by anchor[2] and shifted by anchor[0:2].
        # It is lined up again with the two lane lines of the start straight whenever a new lap is armed.
        self.anchor = (0.0, 0.0, 0.0)
        self.anchor_source = 'fresh start'
        self.anchor_wanted_until = 0.0   # a new lap was armed: take the next good look at the lane lines
        self.lane_fit = None             # (time, heading of the robot in the lane, its distance left of the lane's middle)
        self.lane_fits = []              # the fits collected while a new lap waits to be lined up
        self.pose_odom = None
        self.laps_armed = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, 'drive_mode', latched)
        self.status_pub = self.create_publisher(String, 'lap/status', 5)
        self.finished_pub = self.create_publisher(Bool, 'lap/finished', latched)
        self.route_pub = self.create_publisher(Path, 'lap/route', latched)
        self.corridor_pub = self.create_publisher(PointCloud2, 'lap/corridor', 5)
        self.reset_pub = self.create_publisher(Empty, 'lap/reset', 5)
        self.notice_pub = self.create_publisher(String, 'lap/notice', 5)
        self.clear_clients = [(self.create_client(ClearEntireCostmap, name), label) for name, label in (
            ('local_costmap/clear_entirely_local_costmap', 'local cost map'),
            ('global_costmap/clear_entirely_global_costmap', 'global cost map'))]
        self.cmd_pub = self.create_publisher(TwistStamped, p('cmd_topic', 'diffbot_base_controller/cmd_vel').value, 5)
        self.create_subscription(Odometry, p('odom_topic', 'odometry/filtered/local').value, self.odom_cb, 20)
        self.create_subscription(String, 'drive_mode_request', self.mode_cb, 5)
        self.create_subscription(String, 'lap/command', self.command_cb, 5)
        self.create_subscription(String, 'ramp/state', self.ramp_cb, latched)
        self.create_subscription(String, 'perception/lane_info', self.lane_cb, 5)
        self.create_subscription(PointCloud2, p('obstacle_topic', 'velodyne_points_nav').value, self.obstacle_cb, LATEST)
        self.create_subscription(PointCloud2, 'perception/lane_points', self.lane_points_cb, LATEST)
        self.create_subscription(String, 'perception/pothole_info', self.pothole_cb, 5)
        self.create_subscription(Path, 'plan', self.plan_cb, 5)
        self.create_subscription(NavSatFix, p('gps_topic', 'gps/fix').value, self.gps_cb, 5)
        # Simulator only: Gazebo's own record of where the robot is, used to score a lap honestly.
        self.create_subscription(TFMessage, p('truth_topic', 'sim/true_poses').value, self.truth_cb, 5)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.load_route()
        self.mode_pub.publish(String(data=self.mode))
        self.finished_pub.publish(Bool(data=False))
        self.create_timer(0.2, self.tick)
        self.create_timer(0.05, self.hold_tick)
        if self.autostart:
            self.mode = 'automatic'
            self.mode_pub.publish(String(data=self.mode))
            self.begin_arming()

    # ------------------------------------------------------------------ route
    def load_route(self):
        self.route = []
        self.progress = 0
        self.boost = 0.0
        self.blocked = set()
        if self.memory is None:
            return
        route = self.memory.route()
        cells = self.memory.load_lane_memory(self.course.surface_fingerprint) if self.use_lane_memory else []
        self.lane_memory = np.array(cells, dtype=float).reshape(-1, 2)
        self.route_description, self.best_time = route['description'], route['best_time']
        if route['points']:
            self.route = route_points(route['points'], ROUTE_STEP, self.course)
        self.build_corridor()
        path = Path()
        path.header.frame_id = self.frame
        for gx, gy, gyaw in self.route[::2]:
            path.poses.append(self.pose_msg(gx, gy, gyaw))
        self.route_pub.publish(path)
        self.get_logger().info('Route: %s, %.0f m; %d remembered lane line cells' % (self.route_description, len(self.route) * ROUTE_STEP, len(self.lane_memory)))

    def reload_course(self):
        """The course file was rewritten (a barrel was moved in Blender): score against the new layout."""
        try:
            mtime = os.stat(self.course_file).st_mtime
        except OSError:
            return
        if mtime == self.course_mtime:
            return
        try:
            course = Course.load(self.course_file)
        except (ValueError, KeyError):
            return                  # caught the file half written; the next look will get it
        self.course_mtime = mtime
        old = self.course.layout_id
        self.course = course
        self.tracker.course = course
        if course.layout_id != old:
            self.get_logger().info('The course changed: layout %s is now %s (%d barrels).' % (old, course.layout_id, len(course.barrels)))
            if self.tracker.state == 'running':
                self.tracker.note(self.sim_time, 'course_changed', layout=course.layout_id)

    def pose_msg(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = self.frame
        m.header.stamp = stamp_from_seconds(self.sim_time)   # the odometry's clock; see ramp_detector.now_seconds
        x, y, yaw = self.to_odom(x, y, yaw)
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        return m

    def to_course(self, x, y, yaw):
        dx, dy, dyaw = self.anchor
        c, s = math.cos(dyaw), math.sin(dyaw)
        return c * x - s * y + dx, s * x + c * y + dy, yaw + dyaw

    def to_odom(self, x, y, yaw):
        dx, dy, dyaw = self.anchor
        c, s = math.cos(dyaw), math.sin(dyaw)
        return c * (x - dx) + s * (y - dy), -s * (x - dx) + c * (y - dy), yaw - dyaw

    @staticmethod
    def fit_lane(pts):
        """From lane points in the base frame: the robot's heading in the lane and its distance left of the lane's
        middle, or None unless both lines are seen straight, parallel and a lane's width apart."""
        ahead = pts[(pts[:, 0] > 0.8) & (pts[:, 0] < 4.5)]
        fits = []
        for side in (ahead[ahead[:, 1] > 0.6], ahead[ahead[:, 1] < -0.6]):
            # From 2 m to the side a line enters the picture about 2.2 m ahead, and lane points end at 4 m:
            # there is never much more than 1.5 m of each line to see. One meter is asked for.
            if len(side) < 8 or side[:, 0].max() - side[:, 0].min() < 1.0:
                return None
            a, b = np.polyfit(side[:, 0], side[:, 1], 1)
            if float(np.sqrt(np.mean((side[:, 1] - (a * side[:, 0] + b)) ** 2))) > 0.07:
                return None
            fits.append((float(a), float(b)))
        (al, bl), (ar, br) = fits
        # the two lines of the start straight are not quite parallel in the map (they close by about 0.05 m per meter)
        if abs(al - ar) > 0.12 or not (3.2 < bl - br < 5.0) or abs(al + ar) / 2.0 > 0.35:
            return None
        slope = (al + ar) / 2.0
        return -math.atan(slope), -(bl + br) / 2.0 * math.cos(math.atan(slope))

    def standing_on_start_straight(self):
        if self.pose is None or self.pose_odom is None:
            return False
        x, y, _, speed = self.pose
        return abs(speed) < 0.15 and -2.0 < x < 12.0 and abs(y) < 2.5

    def align_to_start_straight(self):
        """Line the course frame up with the lane lines, from the middle value of the camera frames collected
        since the new lap was armed. Only on the start straight, standing still."""
        if len(self.lane_fits) < 2 or not self.standing_on_start_straight():
            return False
        heading = float(np.median([f[0] for f in self.lane_fits]))
        left_of_middle = float(np.median([f[1] for f in self.lane_fits]))
        frames = len(self.lane_fits)
        self.lane_fits = []
        x, y, yaw, speed = self.pose
        if abs(heading) > 0.5:
            return False
        xo, yo, yaw_o, _ = self.pose_odom
        dyaw = heading - yaw_o
        c, s = math.cos(dyaw), math.sin(dyaw)
        old = self.anchor
        # the place along the straight is kept as odometry has it; heading and side position come from the lines
        self.anchor = (x - (c * xo - s * yo), left_of_middle - (s * xo + c * yo), dyaw)
        self.anchor_source = 'lane lines'
        self.anchor_wanted_until = 0.0
        self.hold_until = min(self.hold_until, self.sim_time + 1.0)
        self.pose = self.to_course(xo, yo, yaw_o) + (speed,)
        if self.tracker is not None and self.tracker.state == 'ready':
            self.tracker.reset()            # the pose just moved a little; that is not the robot setting off
        self.last_progress = (self.sim_time, self.pose[0], self.pose[1])
        self.get_logger().info('Lined up with the start straight from %d camera frames: heading corrected by %+.1f degrees, side position by %+.2f m.'
                               % (frames, math.degrees(dyaw - old[2]), self.pose[1] - y))
        return True

    # ------------------------------------------------------------------ inputs
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        speed = msg.twist.twist.linear.x
        self.pose_odom = (x, y, yaw, speed)
        x, y, yaw = self.to_course(x, y, yaw)
        self.pose = (x, y, yaw, speed)
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.tracker is None:
            return
        truth = None
        if self.truth is not None and abs(self.sim_time - self.truth[3]) < 1.0:
            truth = self.truth[:3]
        for event in self.tracker.update(self.sim_time, x, y, yaw, speed, truth):
            if event == 'started':
                self.get_logger().info('Lap started (%s).' % self.mode)
                self.last_progress = (self.sim_time, x, y)
            elif event == 'checkpoint':
                self.get_logger().info('Checkpoint %d of %d.' % (self.tracker.next_checkpoint, len(self.course.checkpoints)))
            elif event == 'finished':
                self.finish_lap()

    def obstacle_cb(self, msg):
        """LiDAR obstacle points (already leveled and stripped of ground by the ramp detector), kept in the odom frame."""
        self.cloud_time = self.sim_time
        if self.pose is None or msg.point_step < 12:
            return
        pts = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)[:, :2]
        if len(pts) > 1500:
            pts = pts[::len(pts) // 1500 + 1]
        x, y, yaw, _ = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        self.obstacles = np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], axis=-1)

    def lane_points_cb(self, msg):
        """Lane points from the lane mapper (base frame): remembered, in the odom frame, for the laps to come."""
        if self.pose is None or self.tracker is None or msg.point_step < 12:
            return
        if msg.width == 0:
            self.lane_sides = (self.sim_time, 0, 0)
            return
        pts = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)[:, :2]
        ahead = pts[(pts[:, 0] > 0.5) & (pts[:, 0] < 4.5)]
        self.lane_sides = (self.sim_time, int((ahead[:, 1] > 0.6).sum()), int((ahead[:, 1] < -0.6).sum()))
        fit = self.fit_lane(pts)
        if fit is not None:
            self.lane_fit = (self.sim_time, fit[0], fit[1])
            if self.sim_time < self.anchor_wanted_until and self.standing_on_start_straight():
                self.lane_fits.append(fit)
                if len(self.lane_fits) >= 5:
                    self.align_to_start_straight()
        x, y, yaw, _ = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        self.tracker.add_lane_points(zip((x + c * pts[:, 0] - s * pts[:, 1]).tolist(), (y + s * pts[:, 0] + c * pts[:, 1]).tolist()))

    def gps_cb(self, msg):
        """A GPS fix, turned into course coordinates with the datum in the course file."""
        if self.course is None or not self.course.gps:
            return
        valid = msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
        if not valid:
            self.gps_fix = (self.sim_time, 0.0, 0.0, False)
            return
        x, y = gps_to_course(msg.latitude, msg.longitude, self.course.gps)
        self.gps_fix = (self.sim_time, x, y, True)
        if self.course.centerline:
            self.boundary_distance = distance_to_loop((x, y), self.course.centerline)
        else:
            self.boundary_distance = math.hypot(x, y)      # no course line surveyed: distance from the start line

    def begin_arming(self):
        """An automatic run was asked for. It sets off only after the arming check has passed."""
        self.arming_checks = []
        self.arming_deadline = None
        self.arming_changed = self.sim_time
        self.arming = 'checking' if self.arming_check else 'armed'
        self.arming_ticks = 0
        if self.message.startswith('NOT ARMED'):
            self.message = ''               # the refusal before this one is history now

    def arming_evaluate(self, full):
        """The list of checks: [name, passed, what was found].

        Before a lap (full) the robot must be in the start area of this course, by GPS, with a lane line in view.
        When automatic driving is taken up again in the middle of a lap, it must be within the course boundary.
        In both cases the sensors must be alive and GPS and odometry must agree on where the robot is.
        """
        now = self.sim_time
        checks = []

        def fresh(when, limit):
            return when is not None and 0.0 <= now - when <= limit

        ready = self.nav.server_is_ready()
        checks.append(['Navigation', ready, 'ready' if ready else 'not ready yet'])
        ok = fresh(self.cloud_time, 1.5)
        checks.append(['LiDAR', ok, 'points arriving' if ok else 'no points for more than 1.5 s'])
        ok = fresh(self.lane_time, 2.0)
        checks.append(['Camera', ok, 'pictures arriving' if ok else 'no picture processed for more than 2 s'])
        gps = self.course.gps if self.course is not None else None
        if gps:
            fix = self.gps_fix
            have = fix is not None and fresh(fix[0], 3.0) and fix[3]
            checks.append(['GPS', have, 'position fix' if have else 'no position fix'])
            if have:
                limit = gps.get('boundary_half_width_m', 6.0)
                d = self.boundary_distance
                inside = d is not None and d <= limit
                checks.append(['Course boundary', inside, 'GPS puts the robot %.1f m from the course line (limit %.0f m)' % (d if d is not None else -1.0, limit)])
                if full:
                    x0, x1, half = gps.get('start_box', [-4.0, 14.0, 3.0])
                    at_start = x0 <= fix[1] <= x1 and abs(fix[2]) <= half
                    checks.append(['Start area', at_start, 'on the start straight' if at_start
                                   else 'GPS puts the robot %.0f m from the start line' % math.hypot(fix[1], fix[2])])
                if self.pose is not None:
                    gap = math.hypot(fix[1] - self.pose[0], fix[2] - self.pose[1])
                    checks.append(['Position agreement', gap <= 5.0, 'GPS and odometry %s by %.1f m' % ('agree, differing' if gap <= 5.0 else 'differ', gap)])
        elif self.arming_require_gps:
            checks.append(['GPS', False, 'the course file has no GPS datum, so the place cannot be checked'])
        if full:
            sides = self.lane_sides
            seen = 0
            if sides is not None and fresh(sides[0], 1.5):
                seen = int(sides[1] >= 8) + int(sides[2] >= 8)
            checks.append(['Lane lines', seen >= 1, ('no lane line in view', 'one line of the lane in view', 'both lines of the lane in view')[seen]])
        return checks

    def run_arming(self, state):
        """Called every tick while an automatic run waits to be armed."""
        self.arming_checks = self.arming_evaluate(full=(state == 'ready'))
        if self.arming_deadline is None and self.nav.server_is_ready() and self.pose is not None:
            self.arming_deadline = self.sim_time + self.arming_window      # the clock starts once navigation is up
        failed = [c for c in self.arming_checks if not c[1]]
        if not failed:
            self.arming = 'armed'
            self.arming_changed = self.sim_time
            self.get_logger().info('Armed. ' + '; '.join('%s: %s' % (c[0], c[2]) for c in self.arming_checks) + '.')
            self.notice_pub.publish(String(data=json.dumps({'text': 'ARMED   ·   all checks passed'})))
        elif self.arming_deadline is not None and self.sim_time > self.arming_deadline:
            self.refuse_arming(failed)

    def refuse_arming(self, failed):
        self.arming = 'refused'
        self.arming_changed = self.sim_time
        self.cancel_goal()
        self.mode = 'manual'
        self.mode_pub.publish(String(data=self.mode))
        self.message = 'NOT ARMED: ' + '; '.join(c[2] for c in failed)
        self.get_logger().warn('Automatic run refused. ' + '; '.join('%s: %s' % (c[0], c[2]) for c in failed)
                               + '. The robot stays in manual mode.')
        if self.exit_when_done:
            self.done = True

    def plan_cb(self, msg):
        """Refuse a plan that goes the long way round.

        The corridor around the route is a closed loop. When the planner finds the way ahead shut, the only
        other way to the goal it can see is backwards around the whole course, and it takes it: the robot
        turns round and drives off the wrong way. The goal is never more than about 13 m ahead on the route,
        so a plan several times that long is such a detour. It is cancelled, the cost maps are cleared (what
        shut the way is most often a stale or misplaced mark), and the goal is sent again after a moment.
        """
        if self.mode != 'automatic' or self.sent_index is None or self.tracker is None or self.tracker.state != 'running':
            return
        if len(msg.poses) < 2 or not self.route:
            return
        length = 0.0
        last = msg.poses[0].pose.position
        for ps in msg.poses[1:]:
            length += math.hypot(ps.pose.position.x - last.x, ps.pose.position.y - last.y)
            last = ps.pose.position
        along = max(0, self.sent_index - self.progress) * ROUTE_STEP
        if length <= 3.0 * along + 12.0:
            return
        self.detours += 1
        self.tracker.note(self.sim_time, 'detour_refused', planned_m=round(length, 1), ahead_m=round(along, 1))
        self.get_logger().warn('Nav2 planned %.0f m to a goal %.0f m ahead on the route: the long way round. Refused (%d).'
                               % (length, along, self.detours))
        self.cancel_goal()
        for client, _label in self.clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
        self.hold_until = self.sim_time + 1.5
        if self.detours >= 5:
            self.fail_lap('the way ahead stayed shut: Nav2 planned the long way round %d times' % self.detours)

    def pothole_cb(self, msg):
        """Potholes seen by the lane mapper (base frame), kept in the odom frame for this run."""
        if self.pose is None:
            return
        try:
            holes = json.loads(msg.data).get('potholes', [])
        except ValueError:
            return
        x, y, yaw, _ = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        for hole in holes:
            hx = x + c * hole['x'] - s * hole['y']
            hy = y + s * hole['x'] + c * hole['y']
            for known in self.potholes:
                if math.hypot(known['x'] - hx, known['y'] - hy) < 0.7 and known['kind'] == hole.get('kind'):
                    n = known['seen']
                    known['x'] = (known['x'] * n + hx) / (n + 1)
                    known['y'] = (known['y'] * n + hy) / (n + 1)
                    known['diameter'] = max(known['diameter'], hole.get('diameter', 0.0))
                    known['seen'] = n + 1
                    break
            else:
                self.potholes.append({'x': hx, 'y': hy, 'diameter': hole.get('diameter', 0.0), 'kind': hole.get('kind', 'painted'), 'seen': 1})
                if self.tracker is not None and self.tracker.state == 'running':
                    self.tracker.note(self.sim_time, 'pothole_seen', x=round(hx, 2), y=round(hy, 2), pothole=hole.get('kind', 'painted'))

    def pothole_ahead(self):
        """Distance to the nearest pothole seen at least twice that lies ahead of the robot, or None."""
        if self.pose is None:
            return None
        x, y, yaw, _ = self.pose
        best = None
        for hole in self.potholes:
            if hole['seen'] < 2:
                continue
            dx, dy = hole['x'] - x, hole['y'] - y
            d = math.hypot(dx, dy)
            bearing = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
            if d < 6.0 and abs(bearing) < 1.0:
                best = d if best is None else min(best, d)
        return best

    def reset_run_data(self, why):
        """A run begins with a blank slate for everything short-lived, and with everything learned kept.

        Short-lived: the two cost maps, the obstacle points and blocked route points held here, the potholes
        seen, and what the perception nodes and the speed governor hold (they listen on lap/reset). All of it
        describes the surroundings as the sensors last saw them, in the odometry frame, which drifts by a few
        tenths of a meter per lap. Left in place, the marks of the last lap sit beside the real barrels on the
        next one and close passages that are open.
        Kept: the route of the best lap and every run record. Those are what the robot has learned.
        """
        if self.run_reset_time is not None and 0.0 <= self.sim_time - self.run_reset_time < 2.0:
            self.run_reset_done = True
            return                          # just done (a new lap and a mode switch arrive together)
        cleared = []
        for client, label in self.clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
                cleared.append(label)
        self.obstacles = None
        self.potholes = []
        self.lane_info = {}
        if self.blocked:
            self.blocked = set()
            self.build_corridor()
        self.goal_failures = 0
        self.skipped = 0
        self.boost = 0.0
        self.sent_index = None
        self.headway = None
        self.detours = 0
        self.reset_pub.publish(Empty())
        cleared += ['obstacle cache', 'perception buffers']
        # give the sensors and the corridor a moment to fill the empty cost maps before the first plan
        self.hold_until = max(self.hold_until, self.sim_time + 1.5)
        self.run_reset_time = self.sim_time
        self.run_reset_done = True
        runs = self.memory.run_count() if self.memory is not None else 0
        self.get_logger().info('Run data cleared (%s): %s. Kept: route %s; %d runs on record.'
                               % (why, ', '.join(cleared), self.route_description, runs))
        self.notice_pub.publish(String(data=json.dumps({'text': 'RUN DATA CLEARED   ·   ROUTE KEPT'})))

    def truth_cb(self, msg):
        if not msg.transforms or self.truth_ok is False:
            return
        # The bridge leaves the names empty. The model itself is the first entry; its links follow.
        tf = msg.transforms[0]
        for candidate in msg.transforms:
            if candidate.child_frame_id == self.truth_name:
                tf = candidate
                break
        q, v = tf.transform.rotation, tf.transform.translation
        if self.truth_ok is None:
            if self.pose is None:
                return
            # The robot starts where its odometry starts, so the two must agree at the first reading.
            self.truth_ok = math.hypot(v.x - self.pose[0], v.y - self.pose[1]) < 2.0
            if not self.truth_ok:
                self.get_logger().warn('The simulator pose topic does not match the odometry at the start; scoring with odometry.')
                return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.truth = (v.x, v.y, yaw, self.sim_time)

    def ramp_cb(self, msg):
        if self.tracker is not None and msg.data != self.ramp_state and self.tracker.state == 'running':
            self.tracker.note(self.sim_time, 'ramp', state=msg.data)
        self.ramp_state = msg.data

    def lane_cb(self, msg):
        self.lane_time = self.sim_time
        try:
            self.lane_info = json.loads(msg.data)
        except ValueError:
            pass

    def mode_cb(self, msg):
        wanted = msg.data.strip().lower()
        if wanted not in ('manual', 'automatic') or wanted == self.mode:
            return
        self.mode = wanted
        self.mode_pub.publish(String(data=self.mode))
        self.get_logger().info('Drive mode: %s' % self.mode)
        if self.tracker is not None and self.tracker.state == 'running':
            self.tracker.note(self.sim_time, 'mode', mode=self.mode)
        if wanted == 'manual':
            self.cancel_goal()
            self.arming = 'off'
        else:
            self.goal_failures = 0
            self.sent_index = None
            if self.tracker is not None and self.tracker.state == 'ready':
                self.reset_run_data('automatic run about to begin')
            self.begin_arming()
            if self.pose is not None:
                self.last_progress = (self.sim_time, self.pose[0], self.pose[1])

    def command_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'new_lap':
            self.new_lap()
        elif cmd == 'stop':
            self.cancel_goal()
            self.hold_until = self.sim_time + 1.0

    def new_lap(self):
        """Arm the tracker for another lap, starting from where the robot stands."""
        if self.tracker is None:
            return
        if self.tracker.state == 'running':
            self.save_run()          # an abandoned lap is still a record
        self.cancel_goal()
        self.tracker.reset()
        self.load_route()
        self.sent_index = None
        self.goal_failures = 0
        self.skipped = 0
        self.goals_moved = 0
        self.headway = None
        self.run_reset_done = False
        self.reset_run_data('new lap armed')
        self.laps_armed += 1
        # Line the frame up with the start straight. The lane mapper was just reset: collect its next frames
        # for up to three seconds, and do not set off before that.
        self.lane_fits = []
        self.anchor_wanted_until = self.sim_time + 3.0
        self.hold_until = max(self.hold_until, self.sim_time + 3.0)
        self.anchor_source = 'kept from the lap before'     # until the lane lines say otherwise
        self.boundary_strikes = 0
        self.max_boundary = 0.0
        if self.mode == 'automatic':
            self.begin_arming()
        else:
            self.arming = 'off'
        self.message = ''
        self.new_best = False
        self.finished_pub.publish(Bool(data=False))
        if self.pose is not None:
            self.last_progress = (self.sim_time, self.pose[0], self.pose[1])
        self.get_logger().info('New lap armed.')

    # ------------------------------------------------------------------ finish and failure
    def finish_lap(self):
        t = self.tracker
        self.cancel_goal()
        self.hold_until = self.sim_time + 2.0
        self.finished_pub.publish(Bool(data=True))
        name, self.new_best = self.save_run()
        self.arming = 'off'
        self.message = 'LAP COMPLETE in %.1f s' % t.lap_time
        self.get_logger().info('FINISH LINE CROSSED. Lap complete in %.1f s, %.1f m, average %.1f mph, mode %s. Barrel contacts %d, '
                               'lane line touches %d, pothole touches %d (scored with %s). Potholes detected %d. Saved as %s.%s'
                               % (t.lap_time, t.distance, 2.237 * t.distance / max(t.lap_time, 1e-6), self.mode, t.barrel_contacts,
                                  t.lane_touches, t.pothole_touches, 'the simulator true pose' if t.scored_with_truth else 'odometry',
                                  sum(1 for h in self.potholes if h['seen'] >= 2), name,
                                  ' This is the new best route; the next automatic run will drive it.' if self.new_best else ''))
        self.done = True

    def fail_lap(self, reason):
        if self.tracker is None or self.tracker.state != 'running':
            return
        self.tracker.fail(self.sim_time, reason)
        self.cancel_goal()
        self.arming = 'off'
        self.hold_until = self.sim_time + 1.0
        name, _ = self.save_run()
        self.message = 'RUN STOPPED: ' + reason
        self.get_logger().warn('Run stopped: %s. Saved as %s.' % (reason, name))
        self.done = True

    def save_run(self):
        extra = {'route_used': self.route_description, 'layout_id': self.course.layout_id,
                 'surface_fingerprint': self.course.surface_fingerprint, 'lane_memory_used': int(len(self.lane_memory)),
                 'route_progress_m': round(self.progress * ROUTE_STEP, 1), 'route_length_m': round(len(self.route) * ROUTE_STEP, 1),
                 'goals_skipped': self.skipped, 'goals_moved_off_obstacles': self.goals_moved, 'route_points_blocked': len(self.blocked),
                 'run_data_cleared_at_start': bool(self.run_reset_done), 'detours_refused': self.detours,
                 'arming_checks': self.arming_checks, 'max_boundary_distance_m': round(self.max_boundary, 2),
                 'lap_in_session': self.laps_armed + 1,
                 'frame_anchor': {'source': self.anchor_source, 'yaw_deg': round(math.degrees(self.anchor[2]), 2),
                                  'dx_m': round(self.anchor[0], 3), 'dy_m': round(self.anchor[1], 3)},
                 'potholes_detected': [[round(h['x'], 2), round(h['y'], 2), round(h['diameter'], 2), h['kind'], h['seen']]
                                       for h in self.potholes if h['seen'] >= 2]}
        record = self.tracker.record(self.mode, extra)
        self.last_result = {k: record[k] for k in ('result', 'lap_time_s', 'distance_m', 'min_barrel_clearance_m', 'mode',
                                                   'barrel_contacts', 'lane_line_touches', 'pothole_touches', 'clean', 'scored_with', 'average_speed_mph')}
        name, new_best = self.memory.save_run(record)
        if new_best:
            self.best_time = record['lap_time_s']
        return name, new_best

    # ------------------------------------------------------------------ navigation goals
    def cancel_goal(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.goal_handle = None
        self.sent_index = None

    def update_progress(self):
        """Move the progress mark to the nearest route point within the next 15 meters. It never moves back."""
        x, y = self.pose[:2]
        best, best_d = self.progress, None
        for i in range(self.progress, min(len(self.route), self.progress + int(15.0 / ROUTE_STEP))):
            d = math.hypot(self.route[i][0] - x, self.route[i][1] - y)
            if best_d is None or d < best_d:
                best, best_d = i, d
        if best > self.progress + int(3.0 / ROUTE_STEP):
            self.boost = 0.0        # real progress since the last trouble
        self.progress = best
        # distance to the route, counting the 8 m behind the progress mark too: a robot that backs up is still on it
        lo = max(0, self.progress - int(8.0 / ROUTE_STEP))
        hi = min(len(self.route), self.progress + int(15.0 / ROUTE_STEP))
        self.route_distance = min(math.hypot(self.route[i][0] - x, self.route[i][1] - y) for i in range(lo, hi)) if hi > lo else 0.0

    def build_corridor(self):
        """Two rows of points, one to each side of the whole route, plus the lane lines an earlier lap saw.

        The cost maps only know the lane lines the camera has already seen, and No Man's Land has none.
        A robot that finds its way blocked could otherwise leave the course, drive around the outside,
        and never touch a line. The remembered route says where the course is. These points keep the
        planner near it, while leaving the whole lane, and more, free for going around a barrel that
        has been moved. If there is no way through inside the corridor, the robot stops; it does not
        go looking outside.
        """
        self.corridor = None
        if self.corridor_half <= 0 or len(self.route) < 3:
            return
        centers = np.array([[q[0], q[1]] for q in self.route])
        reach = int(6.0 / ROUTE_STEP)       # the corridor is wide for 6 m before and after a blocked point
        wide = np.zeros(len(self.route), bool)
        for b in self.blocked:
            wide[max(0, b - reach):b + reach + 1] = True
        pts = []
        for side in (1.0, -1.0):
            row = []
            for k, (px, py, yaw) in enumerate(self.route):
                half = self.corridor_wide if wide[k] else self.corridor_half
                wx = px - side * half * math.sin(yaw)
                wy = py + side * half * math.cos(yaw)
                # on the inside of a bend the offset row folds back toward the route: drop those points
                if np.min((centers[:, 0] - wx) ** 2 + (centers[:, 1] - wy) ** 2) >= (half - 0.3) ** 2:
                    row.append((wx, wy))
            # A wall must be a line of cells without a break. Nav2 tests the outline of the robot and the cell
            # under its center, and the center is only 0.3 m behind the front edge, so the planner can thread
            # the robot between two single cells half a meter apart. It did, and the robot left the course.
            # The row is therefore filled in every 0.12 m, across the dropped points of a bend, and from the
            # end of the route to its start.
            if len(row) > 1:
                row.append(row[0])
            for a, b in zip(row[:-1], row[1:]):
                gap = math.hypot(b[0] - a[0], b[1] - a[1])
                if gap > 4.0:
                    pts.append((a[0], a[1], 0.5))
                    continue                # not one wall: leave it open
                steps = max(1, int(math.ceil(gap / 0.12)))
                for k in range(steps):
                    pts.append((a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps, 0.5))
        # The remembered lane lines come from another lap's odometry, which drifts by a few tenths of a
        # meter. Used as they are they would narrow the tight passages beside a lane line. Each is
        # therefore set back, away from the route, by lane_setback. The camera still marks the true line.
        for q in self.lane_memory:
            d = np.hypot(centers[:, 0] - q[0], centers[:, 1] - q[1])
            i = int(np.argmin(d))
            if d[i] < 0.3 or d[i] > self.corridor_wide + 1.0:
                continue                # on top of the route (a wrong point), or nowhere near it
            ux, uy = (q[0] - centers[i, 0]) / d[i], (q[1] - centers[i, 1]) / d[i]
            pts.append((float(q[0] + self.lane_setback * ux), float(q[1] + self.lane_setback * uy), 0.5))
        if pts:
            self.corridor = np.array(pts, np.float32)

    def publish_corridor(self):
        if self.corridor is None:
            return
        cloud = PointCloud2()
        cloud.header.frame_id = self.frame
        cloud.header.stamp = stamp_from_seconds(self.sim_time)
        cloud.height = 1
        cloud.width = len(self.corridor)
        cloud.fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1) for k, n in enumerate('xyz')]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(self.corridor)
        cloud.is_dense = True
        dx, dy, dyaw = self.anchor
        c, s = math.cos(dyaw), math.sin(dyaw)
        in_odom = self.corridor.copy()
        in_odom[:, 0] = c * (self.corridor[:, 0] - dx) + s * (self.corridor[:, 1] - dy)
        in_odom[:, 1] = -s * (self.corridor[:, 0] - dx) + c * (self.corridor[:, 1] - dy)
        cloud.data = in_odom.astype(np.float32).tobytes()
        self.corridor_pub.publish(cloud)

    def occupied(self, index, clearance=None):
        if self.obstacles is None or len(self.obstacles) == 0:
            return False
        gx, gy = self.route[index][:2]
        d2 = (self.obstacles[:, 0] - gx) ** 2 + (self.obstacles[:, 1] - gy) ** 2
        return bool(d2.min() < (clearance or self.goal_clearance) ** 2)

    def widen_where_blocked(self):
        """Look 12 m ahead along the route. Where an obstacle stands on it, the corridor becomes wide."""
        found = False
        for i in range(self.progress, min(len(self.route), self.progress + int(12.0 / ROUTE_STEP))):
            if i not in self.blocked and self.occupied(i, self.block_clearance):
                self.blocked.add(i)
                found = True
        if found:
            self.build_corridor()

    def goal_index(self):
        """Where the goal belongs now: the lookahead distance along the route, slid forward off any obstacle."""
        last = len(self.route) - 1
        wanted = min(last, self.progress + int((self.lookahead + self.boost) / ROUTE_STEP))
        for i in range(wanted, min(last, wanted + int(6.0 / ROUTE_STEP)) + 1):
            if not self.occupied(i):
                if i != wanted:
                    return i, True
                return i, False
        return wanted, False

    def send_goal(self, index):
        if not self.nav.server_is_ready():
            return
        gx, gy, gyaw = self.route[index]
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_msg(gx, gy, gyaw)
        self.sent_index = index
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(lambda f, i=index: self.goal_response(f, i))

    def goal_response(self, future, index):
        handle = future.result()
        if handle is None or not handle.accepted:
            if self.sent_index == index:
                self.sent_index = None
            return
        if self.sent_index != index or self.mode != 'automatic':
            handle.cancel_goal_async()   # superseded while the request was on its way
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f, i=index: self.goal_result(f, i))

    def goal_result(self, future, index):
        if self.sent_index != index:
            return                        # an older goal that was replaced
        status = future.result().status
        self.goal_handle = None
        self.sent_index = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_failures = 0
            if index >= len(self.route) - 1 and self.tracker is not None and self.tracker.state == 'running':
                # the end of the route, and the finish line did not count: a checkpoint was missed on the way
                self.fail_lap('the end of the route was reached with %d of %d checkpoints passed'
                              % (self.tracker.next_checkpoint, len(self.course.checkpoints)))
        elif status == GoalStatus.STATUS_ABORTED and self.mode == 'automatic':
            self.goal_failures += 1
            if self.tracker is not None:
                self.tracker.note(self.sim_time, 'goal_aborted', route_m=round(index * ROUTE_STEP, 1), attempt=self.goal_failures)
            self.get_logger().warn('Nav2 gave up on the goal at %.0f m of the route (attempt %d).' % (index * ROUTE_STEP, self.goal_failures))
            if self.goal_failures >= self.max_goal_failures:
                self.goal_failures = 0
                if self.skipped < 2 and index < len(self.route) - 1:
                    self.skipped += 1
                    self.boost += 3.0     # aim further along the route
                    self.get_logger().warn('Aiming %.0f m further along the route.' % self.boost)
                else:
                    self.fail_lap('Nav2 could not reach the route at %.0f m' % (index * ROUTE_STEP))

    # ------------------------------------------------------------------ periodic work
    def tick(self):
        self.ticks += 1
        if self.tracker is not None and self.ticks % 5 == 0:
            self.reload_course()
        if self.pose is not None and self.route:
            self.update_progress()
        state = self.tracker.state if self.tracker is not None else 'none'
        if self.mode == 'automatic' and self.arming == 'checking' and (self.pose is None or not self.route):
            # nothing can be checked, and nothing can be driven, without a position and a route: say so, and
            # give up after 20 seconds by the wall clock (the robot's own clock is not running yet)
            self.arming_ticks += 1
            self.arming_checks = [['Odometry', self.pose is not None, 'position arriving' if self.pose is not None else 'no position from the robot'],
                                  ['Route', bool(self.route), 'route on record' if self.route else 'no route on record for this course']]
            if self.arming_ticks > 100:
                self.refuse_arming([c for c in self.arming_checks if not c[1]])
        if self.mode == 'automatic' and self.pose is not None and self.route and state in ('ready', 'running'):
            if 0.0 < self.anchor_wanted_until <= self.sim_time and self.align_to_start_straight():
                pass                        # lined up with the frames that did come
            elif 0.0 < self.anchor_wanted_until <= self.sim_time:
                self.anchor_wanted_until = 0.0
                self.get_logger().warn('Could not line up with the start straight (its two lane lines were not seen clearly from here). '
                                       'The frame of the lap before is kept; this lap will not be used as a route.')
            if state == 'ready' and not self.run_reset_done and self.nav.server_is_ready():
                self.reset_run_data('first automatic run')
            if self.arming == 'checking':
                self.run_arming(state)
            self.widen_where_blocked()
            self.publish_corridor()     # every tick: the walls must be back at once after a cost map is cleared
            if self.sim_time >= self.hold_until and self.mode == 'automatic' and self.arming == 'armed':
                wanted, slid = self.goal_index()
                stale = self.sent_index is None or wanted - self.sent_index >= int(self.resend / ROUTE_STEP) \
                    or (wanted == len(self.route) - 1 and self.sent_index != wanted) \
                    or (self.sent_index < len(self.route) and self.occupied(self.sent_index) and wanted != self.sent_index)
                if stale:
                    if slid:
                        self.goals_moved += 1
                    self.send_goal(wanted)
            if state == 'running':
                t0, x0, y0 = self.last_progress or (self.sim_time, self.pose[0], self.pose[1])
                if math.hypot(self.pose[0] - x0, self.pose[1] - y0) > 0.5 or self.sim_time < t0:
                    self.last_progress = (self.sim_time, self.pose[0], self.pose[1])
                elif self.sim_time - t0 > self.stuck_timeout:
                    self.fail_lap('no progress for %.0f s' % self.stuck_timeout)
                # moving is not enough: the robot must also get further along the route. Without this
                # rule a robot whose way is blocked can wander, and wandering has taken it off the course.
                if self.headway is None or self.sim_time < self.headway[0] or self.progress >= self.headway[1] + int(2.0 / ROUTE_STEP):
                    self.headway = (self.sim_time, self.progress)
                elif self.sim_time - self.headway[0] > self.stuck_timeout and self.progress < len(self.route) - 2:
                    self.fail_lap('no headway along the route for %.0f s (the way may be blocked)' % self.stuck_timeout)
                # a robot this far from the route has left the corridor, whatever let it through
                if self.mode == 'automatic' and self.off_route_limit > 0 and self.route_distance > self.off_route_limit \
                        and self.progress < len(self.route) - 2:
                    self.fail_lap('the robot left the route (%.1f m from it)' % self.route_distance)
                # the GPS boundary: the robot must stay near the course line, wherever its odometry believes it is
                gps = self.course.gps
                if gps and self.boundary_check and self.mode == 'automatic' and self.tracker.state == 'running':
                    fix = self.gps_fix
                    limit = gps.get('boundary_half_width_m', 6.0)
                    if fix is not None and fix[3] and 0.0 <= self.sim_time - fix[0] <= 1.5 and self.boundary_distance is not None:
                        self.max_boundary = max(self.max_boundary, self.boundary_distance)
                        self.boundary_strikes = self.boundary_strikes + 1 if self.boundary_distance > limit else 0
                        if self.boundary_strikes >= 5:      # one second outside
                            self.fail_lap('the robot left the course: GPS puts it %.1f m from the course line (limit %.0f m)'
                                          % (self.boundary_distance, limit))
                    elif self.gps_loss_timeout > 0 and (fix is None or self.sim_time - fix[0] > self.gps_loss_timeout):
                        self.fail_lap('no GPS fix for more than %.0f s' % self.gps_loss_timeout)
                if self.tracker.state == 'running' and self.tracker.lap_time > self.max_lap_time:
                    self.fail_lap('lap time limit of %.0f s' % self.max_lap_time)
        status = {'mode': self.mode, 'state': state, 'message': self.message, 'ramp': self.ramp_state,
                  'lane': self.lane_info, 'route': self.route_description, 'best_time': self.best_time,
                  'goal': self.progress, 'goals': len(self.route), 'nav_ready': self.nav.server_is_ready(),
                  'last_result': self.last_result, 'new_best': self.new_best,
                  'arming': self.arming, 'arming_checks': self.arming_checks,
                  'arming_age': round(max(0.0, self.sim_time - self.arming_changed), 1),
                  'boundary_m': None if self.boundary_distance is None else round(self.boundary_distance, 1),
                  'boundary_limit': (self.course.gps or {}).get('boundary_half_width_m') if self.course is not None else None}
        if self.tracker is not None:
            t = self.tracker
            status.update({'lap_time': round(t.lap_time, 1), 'distance': round(t.distance, 1),
                           'checkpoint': t.next_checkpoint, 'checkpoints': len(self.course.checkpoints),
                           'runs': self.memory.run_count(), 'barrel_contacts': t.barrel_contacts,
                           'lane_touches': t.lane_touches, 'pothole_touches': t.pothole_touches,
                           'potholes_seen': [[round(h['x'], 2), round(h['y'], 2)] for h in self.potholes if h['seen'] >= 2],
                           'pothole_ahead': (lambda d: None if d is None else round(d, 1))(self.pothole_ahead()),
                           'truth': t.scored_with_truth, 'layout': self.course.layout_id,
                           'average_mph': round(2.237 * t.distance / t.lap_time, 2) if t.lap_time > 1.0 else 0.0})
        if self.pose is not None:
            status.update({'x': round(self.pose[0], 2), 'y': round(self.pose[1], 2), 'yaw': round(self.pose[2], 3),
                           'speed': round(self.pose[3], 2)})
        self.status_pub.publish(String(data=json.dumps(status)))
        if self.done and self.exit_when_done and self.sim_time >= self.hold_until:
            raise SystemExit

    def hold_tick(self):
        """After the finish line (or a stop) command zero speed for a moment, so the robot halts at once."""
        if self.sim_time < self.hold_until and self.mode == 'automatic':
            msg = TwistStamped()
            msg.header.stamp = stamp_from_seconds(self.sim_time)
            msg.header.frame_id = 'base_link'
            self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LapManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
