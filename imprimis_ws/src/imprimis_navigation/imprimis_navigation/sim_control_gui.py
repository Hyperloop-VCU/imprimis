"""Simulation control window: a first-person view, WASD teleop, the IGVC waypoints, and a reset button.

Driving
  W / S   forward / reverse        Shift   boost, held
  A / D   turn left / right        Space   brake
  F11     full screen              Esc     leave full screen

W A S D publishes straight to diffbot_base_controller/cmd_vel, the topic the Nav2 velocity smoother
also publishes on, so driving by hand while Nav2 navigates means the two fight over the wheels. This
window is for simulation testing, where that is the operator's business; there is no mode switch.

Send IGVC Waypoints
  Publishes the four course waypoints as a GeoPath on gps_waypoint_goal_array. map_goal_to_odom
  converts them with navsat_transform's /fromLL service and feeds them to Nav2 one at a time.

Reset / Reset to First Turn
  Cancels every Nav2 goal, stops the wheels, teleports the robot in Gazebo to the start or to the
  first turn, resets both EKFs to that pose, and clears the local and global costmaps. Each step
  reports into the log at the bottom of the window.

  The costmaps are cleared twice with a pause between, and the pauses either side matter: every
  observation source in Course2027.yaml has clearing: False, so nothing is ever raytraced away once
  marked, and at update_frequency 50 Hz the obstacle layer re-marks the newest cloud still sitting in
  its observation buffer within a tick of the clear. Clearing too soon after the teleport therefore
  stamps pre-teleport points back onto the fresh map at the robot's new pose. The settle_* parameters
  set the pauses; the log reports how much of each costmap is marked when the dust settles.

Topics out: diffbot_base_controller/cmd_vel (TwistStamped), gps_waypoint_goal_array (GeoPath)
Topics in:  cameras/front/color/image_raw, gps/fix, odometry/filtered/local,
            navigate_to_pose/_action/status
Services:   navigate_to_pose/_action/cancel_goal, ekf_local/set_pose, ekf_global/set_pose,
            local_costmap/clear_entirely_local_costmap, global_costmap/clear_entirely_global_costmap
Gazebo:     /world/<world>/set_pose, through the gz command line tool; Gazebo is asked
            for the world's name unless the world_name parameter is set
"""
import math
import signal
import shutil
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from geometry_msgs.msg import TwistStamped
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from robot_localization.srv import SetPose
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus

from PyQt5.QtCore import QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QFontMetrics, QImage, QPainter, QPen
from PyQt5.QtWidgets import (QApplication, QFrame, QHBoxLayout, QLabel, QPlainTextEdit, QPushButton,
                             QSizePolicy, QSplitter, QVBoxLayout, QWidget)

# palette
RED = QColor(218, 41, 42)
TEAL = QColor(46, 211, 198)
AMBER = QColor(255, 180, 0)
WHITE = QColor(255, 255, 255)
GREY = QColor(140, 148, 160)
DARK = QColor(11, 15, 20)
PANEL = QColor(11, 15, 20, 195)
SUNKEN = '#12161c'
EDGE = '#262c36'

NAV_STATE = {
    GoalStatus.STATUS_UNKNOWN: ('idle', GREY),
    GoalStatus.STATUS_ACCEPTED: ('accepted', AMBER),
    GoalStatus.STATUS_EXECUTING: ('navigating', TEAL),
    GoalStatus.STATUS_CANCELING: ('canceling', AMBER),
    GoalStatus.STATUS_SUCCEEDED: ('succeeded', TEAL),
    GoalStatus.STATUS_CANCELED: ('canceled', GREY),
    GoalStatus.STATUS_ABORTED: ('aborted', RED),
}

GPS_STATE = {
    NavSatStatus.STATUS_NO_FIX: ('no fix', RED),
    NavSatStatus.STATUS_FIX: ('fix', TEAL),
    NavSatStatus.STATUS_SBAS_FIX: ('SBAS fix', TEAL),
    NavSatStatus.STATUS_GBAS_FIX: ('GBAS fix', TEAL),
}

# The four course waypoints, in order, as latitude and longitude. Numbers 2 and 3 are only about
# three metres apart, which is still clear of map_goal_to_odom's one metre capture radius.
IGVC_WAYPOINTS = ((42.6675857, -83.2194531),
                  (42.6675744, -83.2196290),
                  (42.6675802, -83.2196689),
                  (42.6675771, -83.2198280))

WGS84_A = 6378137.0             # equatorial radius, m
WGS84_E2 = 6.69437999014e-3     # first eccentricity squared


def metres_per_degree(latitude):
    """Metres per degree of latitude and of longitude on the WGS84 ellipsoid at this latitude."""
    lat = math.radians(latitude)
    s = math.sin(lat)
    denominator = 1.0 - WGS84_E2 * s * s
    per_degree_latitude = math.pi * WGS84_A * (1.0 - WGS84_E2) / (180.0 * denominator ** 1.5)
    per_degree_longitude = math.pi * WGS84_A * math.cos(lat) / (180.0 * math.sqrt(denominator))
    return per_degree_latitude, per_degree_longitude


def offset_to_latlon(latitude, longitude, north, east):
    """The point `north` and `east` metres from (latitude, longitude). Good to millimetres over a lap."""
    per_latitude, per_longitude = metres_per_degree(latitude)
    return latitude + north / per_latitude, longitude + east / per_longitude


def latlon_to_offset(latitude, longitude, to_latitude, to_longitude):
    """How far north and east (metres) the second point lies from the first."""
    per_latitude, per_longitude = metres_per_degree(latitude)
    return (to_latitude - latitude) * per_latitude, (to_longitude - longitude) * per_longitude


class SimControlNode(Node):
    """Everything this window says to, and hears from, the rest of the system."""

    def __init__(self):
        super().__init__('sim_control_gui')
        p = self.declare_parameter

        # Stamps from this node reach robot_localization, which compares them against the sensor
        # stamps it has already fused, so the window has to be on the simulator's clock.
        if p('sim_time', True).value:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # driving
        self.v_normal = p('speed_normal', 1.0).value
        self.v_boost = p('speed_boost', 2.1).value          # 4.7 mph; the IGVC limit of 5 mph is 2.235
        self.v_reverse = p('speed_reverse', 0.6).value
        self.w_normal = p('turn_rate', 1.0).value
        self.w_boost = p('turn_rate_boost', 1.4).value

        # The two places Reset can put the robot back to, and the waypoints the course button sends.
        self.reference_latitude = p('reference_latitude', 42.66790675650037).value
        self.reference_longitude = p('reference_longitude', -83.219581829959).value
        self.first_turn_latitude = p('first_turn_latitude', 42.6679100).value
        self.first_turn_longitude = p('first_turn_longitude', -83.2194443).value
        flat = p('waypoints', [value for point in IGVC_WAYPOINTS for value in point]).value
        if len(flat) < 2 or len(flat) % 2:
            self.get_logger().error(f'waypoints needs an even number of values (latitude, longitude, ...), '
                                    f'got {len(flat)}; using the built-in course')
            flat = [value for point in IGVC_WAYPOINTS for value in point]
        self.waypoints = list(zip(flat[0::2], flat[1::2]))

        # How long the reset waits between its steps. The costmaps are the reason these exist: see
        # the note at the top of this file.
        self.settle_after_cancel = p('settle_after_cancel', 0.4).value
        self.brake_seconds = p('brake_seconds', 0.5).value
        self.settle_after_teleport = p('settle_after_teleport', 1.0).value
        # Six times the period of the slowest obstacle source (the lane and pothole clouds run at
        # about 4 Hz), so every observation buffer holds a cloud from the new pose before the clear.
        self.settle_after_filters = p('settle_after_filters', 1.5).value
        self.settle_between_clears = p('settle_between_clears', 1.0).value

        # Gazebo. The world's own latitude and longitude (its <spherical_coordinates>) is the origin
        # of its ENU coordinates, which is also where the robot was spawned, so it is the origin of
        # the odom and map frames as well.
        self.world_name = p('world_name', '').value    # empty: ask Gazebo which world it has open
        self.robot_name = p('robot_name', 'imprimis').value
        self.world_latitude = p('world_datum_latitude', 42.66791).value
        self.world_longitude = p('world_datum_longitude', -83.21958).value
        self.spawn_height = p('spawn_height', 0.1).value    # the -z given to ros_gz_sim create
        self.spawn_yaw = p('spawn_yaw', 0.0).value          # radians counter-clockwise from east
        self.gz_timeout = p('gz_service_timeout', 3.0).value

        self.frame = None
        self.frame_count = 0
        self.frame_times = []
        self.gps = None
        self.gps_arrival = 0.0
        self.odom = None
        self.nav_status = GoalStatus.STATUS_UNKNOWN

        self.cmd_pub = self.create_publisher(TwistStamped, p('cmd_topic', 'diffbot_base_controller/cmd_vel').value, 5)
        self.waypoint_topic = p('waypoint_topic', 'gps_waypoint_goal_array').value
        self.waypoint_pub = self.create_publisher(GeoPath, self.waypoint_topic, 10)

        self.image_topic = p('image_topic', 'cameras/front/color/image_raw').value
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.gps_topic = p('gps_topic', 'gps/fix').value
        self.create_subscription(NavSatFix, self.gps_topic, self.gps_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, p('odom_topic', 'odometry/filtered/local').value, self.odom_cb, 5)
        self.create_subscription(GoalStatusArray, p('nav_status_topic', 'navigate_to_pose/_action/status').value,
                                 self.nav_status_cb, 10)

        # Services used by Reset. They are called from a worker thread, so they need a callback group
        # that the executor is free to serve while that thread waits.
        group = ReentrantCallbackGroup()
        self.cancel_client = self.create_client(
            CancelGoal, p('nav_cancel_service', 'navigate_to_pose/_action/cancel_goal').value, callback_group=group)
        self.local_ekf_client = self.create_client(
            SetPose, p('local_ekf_service', 'ekf_local/set_pose').value, callback_group=group)
        self.global_ekf_client = self.create_client(
            SetPose, p('global_ekf_service', 'ekf_global/set_pose').value, callback_group=group)
        self.local_costmap_client = self.create_client(
            ClearEntireCostmap, p('local_costmap_service', 'local_costmap/clear_entirely_local_costmap').value,
            callback_group=group)
        self.global_costmap_client = self.create_client(
            ClearEntireCostmap, p('global_costmap_service', 'global_costmap/clear_entirely_global_costmap').value,
            callback_group=group)

        # Read-back for the clear. The costmap topics are no use here: with always_send_full_costmap
        # false the full grid is published once and only deltas after that, so a late subscriber
        # hears nothing. This service answers with the whole grid whenever it is asked.
        self.local_costmap_read = self.create_client(
            GetCostmap, p('local_costmap_read_service', 'local_costmap/get_costmap').value, callback_group=group)
        self.global_costmap_read = self.create_client(
            GetCostmap, p('global_costmap_read_service', 'global_costmap/get_costmap').value, callback_group=group)

        self.local_frame = p('local_frame', 'odom').value
        self.global_frame = p('global_frame', 'map').value

    # ---------------------------------------------------------------- subscriptions

    def image_cb(self, msg):
        if msg.encoding in ('rgb8', 'bgr8'):
            self.frame = msg
            self.frame_count += 1
            self.frame_times.append(time.monotonic())
            del self.frame_times[:-40]

    def gps_cb(self, msg):
        self.gps = msg
        self.gps_arrival = time.monotonic()     # wall clock: this is about the fix being live, not the sim's time

    def gps_age(self):
        return float('inf') if self.gps is None else time.monotonic() - self.gps_arrival

    def odom_cb(self, msg):
        self.odom = msg

    def nav_status_cb(self, msg):
        if msg.status_list:
            newest = max(msg.status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
            self.nav_status = newest.status

    def camera_fps(self):
        if len(self.frame_times) < 2:
            return 0.0
        span = self.frame_times[-1] - self.frame_times[0]
        return 0.0 if span <= 0.0 else (len(self.frame_times) - 1) / span

    # ---------------------------------------------------------------- driving

    def drive(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def brake(self, seconds):
        """Hold a zero command long enough that the simulator cannot be left with an older one."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.drive(0.0, 0.0)
            time.sleep(0.05)

    # ---------------------------------------------------------------- waypoints

    def send_igvc_waypoints(self):
        """Publish the course waypoints, in order. GPS points carry no heading, so map_goal_to_odom
        points each one along the route for us."""
        path = GeoPath()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'wgs84'
        for latitude, longitude in self.waypoints:
            waypoint = GeoPoseStamped()
            waypoint.header = path.header
            waypoint.pose.position.latitude = float(latitude)
            waypoint.pose.position.longitude = float(longitude)
            waypoint.pose.position.altitude = 0.0
            path.poses.append(waypoint)

        self.waypoint_pub.publish(path)
        return list(self.waypoints), self.waypoint_pub.get_subscription_count()

    def leg_lengths(self):
        """How far apart the waypoints are, in metres, starting with the reference point."""
        legs = []
        previous = (self.reference_latitude, self.reference_longitude)
        for point in self.waypoints:
            north, east = latlon_to_offset(previous[0], previous[1], point[0], point[1])
            legs.append(math.hypot(north, east))
            previous = point
        return legs

    # ---------------------------------------------------------------- reset

    def point_in_world(self, latitude, longitude):
        """A latitude and longitude as (x, y, yaw) in the Gazebo world, which is also odom and map.

        The world is ENU with its origin at the world's own latitude and longitude, and the robot was
        spawned at that origin, so this one pose serves the teleport and both filter resets.
        """
        north, east = latlon_to_offset(self.world_latitude, self.world_longitude, latitude, longitude)
        return east, north, self.spawn_yaw

    def reset(self, report, latitude, longitude, name):
        """Put the simulation back to a known place. Runs on a worker thread; `report` logs one line.

        The order and the waiting both matter. Nav2 is stopped first so nothing is driving during the
        teleport; the filters are told where the robot is only once Gazebo has actually moved it; and
        the costmaps are cleared only once the sensors have had time to produce clouds from the new
        pose, then cleared a second time to catch whatever was marked while that was settling.
        """
        x, y, yaw = self.point_in_world(latitude, longitude)
        report(f'--- reset to {name} ---')
        report(f'{latitude:.7f}, {longitude:.7f} is x {x:+.2f} y {y:+.2f} in {self.global_frame}')

        self.cancel_navigation(report)
        self.pause(self.settle_after_cancel)
        self.brake(self.brake_seconds)
        report('wheels: stopped')

        if not self.teleport(x, y, yaw, report):
            report('--- reset abandoned: the robot was not moved ---')
            return
        self.pause(self.settle_after_teleport, report, 'letting Gazebo settle the robot')

        self.reset_filters(x, y, yaw, report)
        self.pause(self.settle_after_filters, report, 'waiting for sensor data from the new pose')

        self.clear_costmaps(report)
        self.pause(self.settle_between_clears, report, 'letting the obstacle layers refill')
        self.clear_costmaps(report, again=True)

        self.report_costmap_fill(report)
        report('--- reset complete ---')

    def pause(self, seconds, report=None, why=None):
        if seconds <= 0.0:
            return
        if report is not None and why is not None:
            report(f'waiting {seconds:.1f} s: {why}')
        time.sleep(seconds)

    def cancel_navigation(self, report):
        # Asked for unconditionally: the status this window has seen is not a reliable account of
        # what Nav2 is doing, since a goal preempted by the next waypoint reports itself aborted.
        # A zero goal id with a zero stamp means "every goal".
        self.call(self.cancel_client, CancelGoal.Request(), 'navigation: every goal canceled', report)

    def gz_service(self, service, request_type, reply_type, request):
        """Call a Gazebo service with the gz command line tool. Returns its stdout, or None.

        --req goes last: the tool hangs if an empty request string is followed by another flag.
        """
        gz = shutil.which('gz') or shutil.which('ign')
        if gz is None:
            return None
        command = [gz, 'service', '-s', service, '--reqtype', request_type, '--reptype', reply_type,
                   '--timeout', str(int(self.gz_timeout * 1000.0)), '--req', request]
        try:
            return subprocess.run(command, capture_output=True, text=True, timeout=self.gz_timeout + 2.0).stdout
        except (subprocess.TimeoutExpired, OSError):
            return None

    def gz_world(self):
        """The name of the world Gazebo has open, which the world services are named after."""
        if self.world_name:
            return self.world_name
        answer = self.gz_service('/gazebo/worlds', 'gz.msgs.Empty', 'gz.msgs.StringMsg_V', '')
        if not answer or '"' not in answer:
            return None
        self.world_name = answer.split('"')[1]      # data: "<world>"
        return self.world_name

    def teleport(self, x, y, yaw, report):
        """Move the robot with Gazebo's own set_pose service, through the gz command line tool.

        The blocking form of the service is the one worth calling: it answers once the pose has
        actually been applied, where the plain form answers true as soon as the request is queued,
        even for a model that does not exist.
        """
        world = self.gz_world()
        if world is None:
            report('teleport: FAILED, no answer from Gazebo. Is the simulator running, and gz on the path?')
            return False
        request = (f'name: "{self.robot_name}", '
                   f'position: {{x: {x:.4f}, y: {y:.4f}, z: {self.spawn_height:.4f}}}, '
                   f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2.0):.9f}, w: {math.cos(yaw / 2.0):.9f}}}')
        answer = self.gz_service(f'/world/{world}/set_pose/blocking', 'gz.msgs.Pose', 'gz.msgs.Boolean', request)
        if not answer or 'true' not in answer.lower():
            report(f'teleport: FAILED, /world/{world}/set_pose did not move "{self.robot_name}". '
                   'Is the simulator unpaused, and is that the model name?')
            return False
        report(f'teleport: {self.robot_name} moved to x {x:+.2f} y {y:+.2f} in world "{world}"')
        return True

    def reset_filters(self, x, y, yaw, report):
        for client, frame, label in ((self.local_ekf_client, self.local_frame, 'local EKF'),
                                     (self.global_ekf_client, self.global_frame, 'global EKF')):
            request = SetPose.Request()
            request.pose.header.frame_id = frame
            request.pose.header.stamp = self.get_clock().now().to_msg()
            request.pose.pose.pose.position.x = x
            request.pose.pose.pose.position.y = y
            request.pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
            request.pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
            for i in range(0, 36, 7):
                request.pose.pose.covariance[i] = 1e-6
            self.call(client, request, f'{label}: reset in {frame}', report)

    def clear_costmaps(self, report, again=False):
        pass_name = 'cleared again' if again else 'cleared'
        self.call(self.local_costmap_client, ClearEntireCostmap.Request(), f'local costmap: {pass_name}', report)
        self.call(self.global_costmap_client, ClearEntireCostmap.Request(), f'global costmap: {pass_name}', report)

    def report_costmap_fill(self, report):
        """Read each costmap back and say how much of it carries a cost, so a clear that did not
        take can be seen rather than guessed at. Whatever the robot can see right now comes straight
        back, so a few per cent here is the view from the new pose, not a failed clear."""
        for client, label in ((self.local_costmap_read, 'local costmap'),
                              (self.global_costmap_read, 'global costmap')):
            filled, failure = self.costmap_fill(client)
            if failure is not None:
                report(f'{label}: cannot be read back, {failure}')
            else:
                obstacles, costed, total = filled
                report(f'{label}: {obstacles} obstacle cells of {total}, '
                       f'{100.0 * costed / total:.0f}% of the grid carrying cost')

    def costmap_fill(self, client):
        """Read a costmap through its get_costmap service: (obstacle cells, cells with any cost, size).

        Anything at or above the inscribed cost is something the robot was told is there; everything
        else with a cost is the inflation layer's margin around it, which with inflation_radius 1.3 m
        covers far more of the grid than the obstacles themselves.
        """
        response, failure = self.request(client, GetCostmap.Request())
        if failure is not None:
            return None, failure
        cells = np.frombuffer(response.map.data, dtype=np.uint8)    # rclpy hands these over as an array.array
        return (int(np.count_nonzero(cells >= 253)), int(np.count_nonzero(cells)), int(cells.size)), None

    def request(self, client, message, timeout=5.0):
        """Send a service request from a worker thread. Returns (response, reason it failed)."""
        if not client.wait_for_service(timeout_sec=2.0):
            return None, f'{client.srv_name} is not up'
        future = client.call_async(message)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            return None, f'{client.srv_name} timed out'
        if future.exception() is not None:
            return None, str(future.exception())
        return future.result(), None

    def call(self, client, message, label, report, timeout=5.0):
        """Call a service from a worker thread and say in one line how it went."""
        _, failure = self.request(client, message, timeout)
        report(label if failure is None else f'{label}: FAILED, {failure}')
        return failure is None


class Readout(QWidget):
    """One cell of the bar along the top: a caption with a value under it."""

    def __init__(self, caption, width):
        super().__init__()
        self.caption = QLabel(caption)
        self.caption.setFont(QFont('DejaVu Sans', 7, QFont.Bold))
        self.caption.setStyleSheet(f'color: {GREY.name()};')
        self.value = QLabel('\u2014')
        self.value.setFont(QFont('DejaVu Sans Mono', 10))
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)
        layout.addWidget(self.caption)
        layout.addWidget(self.value)
        self.setMinimumWidth(width)
        self.set('\u2014', GREY)

    def set(self, text, color=WHITE):
        self.value.setText(text)
        self.value.setStyleSheet(f'color: {color.name()};')


class Telemetry(QFrame):
    """The bar along the top of the window: the live GPS fix, and what the robot is doing with it."""

    CELLS = (('fix', 'GPS FIX', 72),
             ('latitude', 'LATITUDE', 100),
             ('longitude', 'LONGITUDE', 104),
             ('altitude', 'ALTITUDE', 70),
             ('accuracy', 'ACCURACY', 70),
             ('offset', 'FROM REFERENCE', 136),
             ('speed', 'SPEED', 76),
             ('nav', 'NAV2', 86),
             ('camera', 'CAMERA', 62))

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setObjectName('telemetry')
        self.setStyleSheet(f'#telemetry {{ background: {SUNKEN}; border: 1px solid {EDGE}; border-radius: 6px; }}')
        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 9, 16, 11)
        layout.setSpacing(14)

        self.cells = {}
        for name, caption, width in self.CELLS:
            if self.cells:
                rule = QFrame()
                rule.setFixedWidth(1)
                rule.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
                rule.setStyleSheet(f'background: {EDGE};')
                layout.addWidget(rule)
            self.cells[name] = Readout(caption, width)
            layout.addWidget(self.cells[name])
        layout.addStretch(1)

    def refresh(self):
        fix, age = self.node.gps, self.node.gps_age()
        if fix is None:
            self.cells['fix'].set('waiting', GREY)
            for name in ('latitude', 'longitude', 'altitude', 'accuracy', 'offset'):
                self.cells[name].set('\u2014', GREY)
        else:
            stale = age > 2.0
            color = GREY if stale else WHITE
            state, state_color = GPS_STATE.get(fix.status.status, ('unknown', AMBER))
            self.cells['fix'].set(f'{age:.0f} s old' if stale else state, GREY if stale else state_color)
            self.cells['latitude'].set(f'{fix.latitude:.7f}', color)        # 7 places is about a centimetre
            self.cells['longitude'].set(f'{fix.longitude:.7f}', color)
            self.cells['altitude'].set(f'{fix.altitude:.1f} m', color)
            sigma = math.sqrt(max(fix.position_covariance[0], fix.position_covariance[4]))
            self.cells['accuracy'].set(f'\u00b1{sigma:.2f} m' if sigma > 0.0 else 'unknown', color)
            north, east = latlon_to_offset(self.node.reference_latitude, self.node.reference_longitude,
                                           fix.latitude, fix.longitude)
            self.cells['offset'].set(f'N {north:+.2f}  E {east:+.2f}', color)

        if self.node.odom is None:
            self.cells['speed'].set('\u2014', GREY)
        else:
            twist = self.node.odom.twist.twist
            self.cells['speed'].set(f'{math.hypot(twist.linear.x, twist.linear.y):.2f} m/s')
        self.cells['nav'].set(*NAV_STATE.get(self.node.nav_status, ('unknown', GREY)))
        rate = self.node.camera_fps()
        self.cells['camera'].set(f'{rate:.0f} fps' if rate else '\u2014', WHITE if rate else GREY)


class Viewport(QWidget):
    """The camera picture with the heads-up display over it."""

    def __init__(self, window):
        super().__init__()
        self.window_ = window
        self.node = window.node
        self.frame_msg = None
        self.image = None
        self.picture = None     # where the camera picture ended up, so the overlay can sit on it
        self.setMinimumSize(640, 360)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def font(self, size, bold=True, mono=False):
        f = QFont('DejaVu Sans Mono' if mono else 'DejaVu Sans', size)
        f.setBold(bold)
        return f

    def text(self, p, x, y, s, size, color=WHITE, mono=False, bold=True):
        """Draw a line of text from its baseline, with a shadow so it reads over any picture."""
        p.setFont(self.font(size, bold, mono))
        p.setPen(QPen(QColor(0, 0, 0, 160)))
        p.drawText(x + 1, y + 1, s)
        p.setPen(QPen(color))
        p.drawText(x, y, s)

    def centred(self, p, rect, s, size, color=WHITE, mono=False, bold=True, align=Qt.AlignCenter):
        """Draw a line of text laid out inside a rectangle, centred unless told otherwise."""
        p.setFont(self.font(size, bold, mono))
        p.setPen(QPen(QColor(0, 0, 0, 160)))
        p.drawText(rect.translated(1.0, 1.0), align, s)
        p.setPen(QPen(color))
        p.drawText(rect, align, s)

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), DARK)

        msg = self.node.frame
        if msg is not None and msg is not self.frame_msg:
            self.frame_msg = msg
            image = QImage(bytes(msg.data), msg.width, msg.height, msg.step, QImage.Format_RGB888)
            self.image = image.rgbSwapped() if msg.encoding == 'bgr8' else image.copy()

        if self.image is None:
            self.picture = None
            p.setFont(self.font(12, bold=True))
            p.setPen(QPen(GREY))
            p.drawText(self.rect(), Qt.AlignCenter, f'waiting for {self.node.image_topic}')
        else:
            scaled = self.image.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            left = (self.width() - scaled.width()) // 2
            top = (self.height() - scaled.height()) // 2
            p.drawImage(left, top, scaled)
            self.picture = (left, top, scaled.width(), scaled.height())

        self.paint_overlay(p)

    def paint_overlay(self, p):
        """One panel, bottom left: the keys as they are held and the command they add up to.

        Laid out from the font metrics rather than from guessed offsets, so the labels, the bars and
        the numbers keep their distance whatever the font ends up being.
        """
        v, w = self.window_.command
        caption = QFontMetrics(self.font(7))
        reading = QFontMetrics(self.font(9, mono=True))

        size, gap, pad = 28, 4, 16
        keys_width = 3 * size + 2 * gap
        label_width = max(caption.horizontalAdvance('DRIVE'), caption.horizontalAdvance('TURN'))
        value_width = reading.horizontalAdvance('-0.00')
        bar_width = 150
        hint = 'SHIFT boost     SPACE brake'

        body = keys_width + 20 + label_width + 10 + bar_width + 10 + value_width
        width = pad + max(body, caption.horizontalAdvance(hint)) + pad
        height = 124
        left, top, picture_width, picture_height = self.picture or (0, 0, self.width(), self.height())
        x = left + 16                               # anchored to the picture, not to the letterbox beside it
        y = top + picture_height - height - 16

        p.setPen(Qt.NoPen)
        p.setBrush(PANEL)
        p.drawRoundedRect(QRectF(x, y, width, height), 8.0, 8.0)

        driving = self.window_.teleop_active()
        self.text(p, x + pad, y + 24, 'TELEOP' if driving else 'TELEOP IDLE', 8, TEAL if driving else GREY)
        for chip, shown, color in (('BOOST', self.window_.boost, AMBER), ('BRAKE', self.window_.braking, RED)):
            if shown:
                self.text(p, x + width - pad - caption.horizontalAdvance(chip) - 4, y + 24, chip, 8, color)

        keys_x, keys_y = x + pad, y + 34
        held = self.window_.held_letters()
        places = {'W': (keys_x + size + gap, keys_y),
                  'A': (keys_x, keys_y + size + gap),
                  'S': (keys_x + size + gap, keys_y + size + gap),
                  'D': (keys_x + 2 * (size + gap), keys_y + size + gap)}
        for key, (key_x, key_y) in places.items():
            down = key in held
            p.setPen(Qt.NoPen)
            p.setBrush(TEAL if down else QColor(255, 255, 255, 32))
            p.drawRoundedRect(QRectF(key_x, key_y, size, size), 4.0, 4.0)
            self.centred(p, QRectF(key_x, key_y, size, size), key, 10, DARK if down else WHITE)
        self.text(p, keys_x, y + 114, hint, 7, GREY)

        bar_left = keys_x + keys_width + 20 + label_width + 10
        bar_height = 12
        for row, (label, value, limit) in enumerate((('DRIVE', v, self.node.v_boost),
                                                     ('TURN', w, self.node.w_boost))):
            bar_top = y + 45 + row * 26             # the two bars sit centred against the block of keys
            middle = bar_left + bar_width // 2
            self.centred(p, QRectF(bar_left - 10 - label_width, bar_top, label_width, bar_height),
                         label, 7, GREY, align=Qt.AlignRight | Qt.AlignVCenter)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(255, 255, 255, 32))
            p.drawRoundedRect(QRectF(bar_left, bar_top, bar_width, bar_height), 3.0, 3.0)
            span = int(min(abs(value) / limit, 1.0) * (bar_width // 2))
            if span:
                p.setBrush(TEAL)
                p.drawRoundedRect(QRectF(middle if value > 0 else middle - span, bar_top, span, bar_height), 3.0, 3.0)
            p.setPen(QPen(QColor(255, 255, 255, 70)))
            p.drawLine(middle, bar_top, middle, bar_top + bar_height)
            self.centred(p, QRectF(bar_left + bar_width + 10, bar_top, value_width, bar_height),
                         f'{value:+.2f}', 9, WHITE, mono=True, align=Qt.AlignRight | Qt.AlignVCenter)


class ControlWindow(QWidget):
    """The window: the telemetry bar, the view, then the two buttons and the log."""

    logged = pyqtSignal(str)
    reset_finished = pyqtSignal()

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.keys = set()
        self.command = (0.0, 0.0)
        self.boost = False
        self.braking = False
        self.publish_until = 0.0
        self.resetting = False

        self.setWindowTitle('IMPRIMIS simulation control')
        self.setMinimumSize(1120, 700)      # narrower than this and the telemetry bar starts to crowd
        self.setObjectName('window')    # scoped, so the bar and the labels keep their own backgrounds
        self.setStyleSheet(f'#window {{ background: {DARK.name()}; }} '
                           'QLabel { color: #e6eaf0; background: transparent; }')
        self.setFocusPolicy(Qt.StrongFocus)

        self.telemetry = Telemetry(node)
        self.viewport = Viewport(self)
        self.waypoint_button = self.make_button('Send IGVC Waypoints', TEAL, self.on_waypoints, filled=True)
        self.reset_button = self.make_button('Reset', RED, self.on_reset, filled=False)
        self.first_turn_button = self.make_button('Reset to First Turn', AMBER, self.on_reset_first_turn,
                                                  filled=False)
        self.buttons = (self.waypoint_button, self.reset_button, self.first_turn_button)

        reference = QLabel(f'reference  {node.reference_latitude:.7f}, {node.reference_longitude:.7f}')
        reference.setFont(QFont('DejaVu Sans Mono', 9))
        reference.setStyleSheet(f'color: {GREY.name()};')

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setFocusPolicy(Qt.NoFocus)
        self.log.setMaximumBlockCount(400)
        self.log.setMinimumHeight(56)
        self.log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.log.setFrameShape(QFrame.NoFrame)
        self.log.setFont(QFont('DejaVu Sans Mono', 9))
        self.log.setStyleSheet(f'background: {SUNKEN}; color: #c0c7d2; border: 1px solid {EDGE}; '
                               'border-radius: 6px; padding: 6px;')

        buttons = QHBoxLayout()
        buttons.setSpacing(10)
        for button in self.buttons:
            buttons.addWidget(button)
        buttons.addStretch(1)
        buttons.addWidget(reference)

        # The buttons and the log are one block under the splitter handle, so dragging the handle
        # gives the console its height back out of the view.
        console = QWidget()
        console_layout = QVBoxLayout(console)
        console_layout.setContentsMargins(0, 0, 0, 0)
        console_layout.setSpacing(10)
        console_layout.addLayout(buttons)
        console_layout.addWidget(self.log)
        console.setMinimumHeight(110)

        self.split = QSplitter(Qt.Vertical)
        self.split.setHandleWidth(10)
        self.split.setChildrenCollapsible(False)
        self.split.setStyleSheet('QSplitter::handle:vertical { background: #1a1f27; margin: 3px 0; '
                                 f'border-radius: 2px; }} QSplitter::handle:vertical:hover {{ background: {EDGE}; }}')
        self.split.addWidget(self.viewport)
        self.split.addWidget(console)
        self.split.setStretchFactor(0, 1)       # the view takes the growth when the window is resized
        self.split.setStretchFactor(1, 0)
        self.split.setSizes([640, 150])

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)
        layout.addWidget(self.telemetry)
        layout.addWidget(self.split, 1)

        self.logged.connect(self.append_log)
        self.reset_finished.connect(self.on_reset_finished)

        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self.control_tick)
        self.control_timer.start(20)        # 50 Hz, the rate the velocity command goes out at
        self.paint_timer = QTimer(self)
        self.paint_timer.timeout.connect(self.viewport.update)
        self.paint_timer.start(33)          # 30 Hz, the rate the front camera runs at
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self.telemetry.refresh)
        self.telemetry_timer.start(200)     # 5 Hz: numbers that can be read without flickering

        self.append_log(f'camera {node.image_topic} | fix {node.gps_topic} | waypoints {node.waypoint_topic}')
        self.append_log(f'reset puts the robot at {node.reference_latitude:.7f}, {node.reference_longitude:.7f}; '
                        f'first turn at {node.first_turn_latitude:.7f}, {node.first_turn_longitude:.7f}')

    def make_button(self, label, accent, slot, filled):
        """Two kinds: the filled one for the ordinary action, the outlined one for the heavy-handed one."""
        button = QPushButton(label)
        button.setFocusPolicy(Qt.NoFocus)       # the keys belong to driving, never to a button
        button.setFixedHeight(40)
        button.setMinimumWidth(200)
        button.setCursor(Qt.PointingHandCursor)
        button.setFont(QFont('DejaVu Sans', 10, QFont.Bold))
        if filled:
            idle = f'background: {accent.name()}; color: #0b0f14; border: 1px solid {accent.name()};'
        else:
            idle = f'background: transparent; color: {accent.name()}; border: 1px solid {accent.name()};'
        button.setStyleSheet(
            f'QPushButton {{ {idle} border-radius: 6px; }}'
            f'QPushButton:hover {{ background: {accent.name()}; color: #ffffff; }}'
            f'QPushButton:disabled {{ background: transparent; color: #5c6472; border: 1px solid {EDGE}; }}')
        button.clicked.connect(slot)
        return button

    # ---------------------------------------------------------------- logging

    def append_log(self, line):
        self.log.appendPlainText(f'{time.strftime("%H:%M:%S")}  {line}')
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    # ---------------------------------------------------------------- buttons

    def on_waypoints(self):
        points, listeners = self.node.send_igvc_waypoints()
        legs = self.node.leg_lengths()
        self.append_log(f'sent {len(points)} GPS waypoints to {listeners} subscriber(s)')
        for i, ((latitude, longitude), leg) in enumerate(zip(points, legs), start=1):
            self.append_log(f'  {i}  {latitude:.7f}, {longitude:.7f}   {leg:6.1f} m from the '
                            f'{"start" if i == 1 else "one before"}')
        if listeners == 0:
            self.append_log(f'  nobody is subscribed to {self.node.waypoint_topic}; is map_goal_to_odom running?')

    def on_reset(self):
        self.start_reset(self.node.reference_latitude, self.node.reference_longitude, 'the start')

    def on_reset_first_turn(self):
        self.start_reset(self.node.first_turn_latitude, self.node.first_turn_longitude, 'the first turn')

    def start_reset(self, latitude, longitude, name):
        if self.resetting:
            return
        self.resetting = True
        for button in self.buttons:
            button.setEnabled(False)
        self.keys.clear()
        threading.Thread(target=self.reset_worker, args=(latitude, longitude, name), daemon=True).start()

    def reset_worker(self, latitude, longitude, name):
        try:
            self.node.reset(self.logged.emit, latitude, longitude, name)
        except Exception as e:                  # a reset must never take the window down with it
            self.logged.emit(f'reset: FAILED, {e}')
        finally:
            self.reset_finished.emit()

    def on_reset_finished(self):
        self.resetting = False
        for button in self.buttons:
            button.setEnabled(True)

    # ---------------------------------------------------------------- driving

    def held_letters(self):
        return {chr(k) for k in self.keys if Qt.Key_A <= k <= Qt.Key_Z}

    def teleop_active(self):
        return time.monotonic() < self.publish_until

    def keyPressEvent(self, e):
        if e.isAutoRepeat():
            return
        if e.key() == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
        elif e.key() == Qt.Key_Escape and self.isFullScreen():
            self.showNormal()
        else:
            self.keys.add(e.key())

    def keyReleaseEvent(self, e):
        if not e.isAutoRepeat():
            self.keys.discard(e.key())

    def focusOutEvent(self, e):
        self.keys.clear()       # a window that loses the keyboard must not keep driving

    def control_tick(self):
        letters = self.held_letters()
        self.boost = Qt.Key_Shift in self.keys
        self.braking = Qt.Key_Space in self.keys
        v = w = 0.0
        if not self.resetting and not self.braking:
            if 'W' in letters:
                v += self.node.v_boost if self.boost else self.node.v_normal
            if 'S' in letters:
                v -= self.node.v_reverse
            if 'A' in letters:
                w += self.node.w_boost if self.boost else self.node.w_normal
            if 'D' in letters:
                w -= self.node.w_boost if self.boost else self.node.w_normal
        self.command = (v, w)

        if self.resetting:
            return              # the reset thread owns the velocity command while it runs
        if v or w or self.braking:
            self.node.drive(v, w)
            # Keep publishing for a moment after the keys come up, so the simulator is never left
            # holding the last command, then fall silent and leave the topic to Nav2.
            self.publish_until = time.monotonic() + 0.5
        elif self.teleop_active():
            self.node.drive(0.0, 0.0)

    def closeEvent(self, e):
        self.control_timer.stop()
        self.paint_timer.stop()
        self.node.drive(0.0, 0.0)
        e.accept()


def main(args=None):
    rclpy.init(args=args)
    node = SimControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    app = QApplication(sys.argv)
    window = ControlWindow(node)
    window.resize(1280, 860)
    window.show()
    window.setFocus()

    # Ctrl-C. Qt's event loop is C++ and never lets Python run a signal handler on its own, and
    # rclpy's handler only shuts the context down, which this window would not notice. So: take the
    # signal back off rclpy, close the window with it, and keep a slow timer running purely to give
    # the interpreter a moment between Qt events in which the handler can fire.
    def interrupt(*_):
        signal.signal(signal.SIGINT, signal.SIG_DFL)    # a second Ctrl-C kills it outright
        print('')
        app.quit()

    signal.signal(signal.SIGINT, interrupt)
    wake = QTimer()
    wake.timeout.connect(lambda: None)
    wake.start(100)

    code = app.exec_()

    node.drive(0.0, 0.0)        # whatever was held when the window went away
    executor.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
