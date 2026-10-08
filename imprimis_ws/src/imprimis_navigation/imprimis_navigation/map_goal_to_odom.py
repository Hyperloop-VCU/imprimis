import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, PoseArray
from action_msgs.msg import GoalStatus, GoalStatusArray
from geographic_msgs.msg import GeoPoint, GeoPath
import tf2_ros
import tf2_geometry_msgs
from threading import Lock
from math import dist, atan2, sin, cos
from robot_localization.srv import FromLL


class MapToOdomGoalConverter(Node):
    """
    Converts goals from the global frame (map) to the local frame (odom) and feeds them to Nav2 one at a time.

    Inputs (each one replaces any sequence already in progress):
        - inputTopic          (PoseStamped): single goal
        - inputArrayTopic     (PoseArray):   waypoints, visited in order
        - inputGPSTopic       (GeoPoint):    single GPS goal, converted with navsat_transform_node's /fromLL
        - inputGPSArrayTopic  (GeoPath):     GPS waypoints, visited in order (only pose.position is used)

    A single goal is just a sequence of length one.

    Sequencing:
        - Intermediate waypoints are "captured" when the robot comes within waypointCaptureRadius (global frame).
          The next goal is then published immediately, which preempts the active Nav2 goal so the robot does not stop.
        - The final waypoint is left to Nav2's goal checker; the sequence ends when Nav2 reports SUCCEEDED.
        - If Nav2 reports SUCCEEDED for an intermediate waypoint (e.g. its goal checker tolerance is larger than the
          capture radius), the sequence also advances.
        - CANCELED ends the sequence. ABORTED ends it if stopOnFailure, otherwise that waypoint is skipped.

    Drift correction (unchanged): while navigating, the active global goal is periodically re-expressed in the local
    frame. If it has moved more than errorBeforeRepublish (m) from the published local goal, the local goal is republished.
    errorBeforeRepublish should be higher than the acceptable map jitter but low enough to account for odom drift.

    Headings: GPS goals have no heading, so each GPS waypoint faces the next one (the last faces along the final leg,
    and a lone GPS goal faces away from the robot's position). Pose goals keep their orientation unless
    overridePoseHeadings is true.

    If there is no navigation happening (according to navStatusTopic), no drift corrections or captures are published.
    Assumes this node and bt_navigator share a clock (same machine, same use_sim_time), since goal acceptance
    stamps are compared against this node's publish times.
    """

    def __init__(self):
        super().__init__("map_goal_to_odom")

        # Params
        self.globalFrame = self.declare_parameter("globalFrame", "map").value
        self.localFrame = self.declare_parameter("localFrame", "odom").value
        self.robotFrame = self.declare_parameter("robotFrame", "base_link").value
        self.inputTopic = self.declare_parameter("inputTopic", "goal_pose").value
        self.inputArrayTopic = self.declare_parameter("inputArrayTopic", "goal_pose_array").value
        self.inputGPSTopic = self.declare_parameter("inputGPSTopic", "gps_waypoint_goal").value
        self.inputGPSArrayTopic = self.declare_parameter("inputGPSArrayTopic", "gps_waypoint_goal_array").value
        self.outputTopic = self.declare_parameter("outputTopic", "odom_goal_pose").value
        self.outputRvizTopic = self.declare_parameter("outputRvizTopic", "odom_goal_pose_rviz").value
        self.errorBeforeRepublish = self.declare_parameter("errorBeforeRepublish", 3.0).value  # meters
        self.errorCheckPeriod = self.declare_parameter("errorCheckPeriod", 1.0).value  # seconds
        self.captureRadius = self.declare_parameter("waypointCaptureRadius", 1.0).value  # meters
        self.progressCheckPeriod = self.declare_parameter("progressCheckPeriod", 0.1).value  # seconds
        self.stopOnFailure = self.declare_parameter("stopOnFailure", True).value
        self.overridePoseHeadings = self.declare_parameter("overridePoseHeadings", False).value
        self.navStatusTopic = self.declare_parameter("navStatusTopic", "navigate_to_pose/_action/status").value
        self.tfTimeout = self.declare_parameter("tfTimeout", 0.02).value  # seconds
        self.debug = self.declare_parameter("debug", False).value

        # dynamic parameter
        self.useGps = self.declare_parameter("useGps", True).value

        # ROS objects
        self.outputPub = self.create_publisher(PoseStamped, self.outputTopic, 10)
        self.outputRvizPub = self.create_publisher(PoseStamped, self.outputRvizTopic, 10)
        self.inputSub = self.create_subscription(PoseStamped, self.inputTopic, self.new_global_goal_cb, 10)
        self.inputArraySub = self.create_subscription(PoseArray, self.inputArrayTopic, self.new_global_goal_array_cb, 10)

        if self.useGps:
            self.cb_group = ReentrantCallbackGroup()
            self.inputGPSSub = self.create_subscription(GeoPoint, self.inputGPSTopic, self.new_gps_goal_cb, 10, callback_group=self.cb_group)
            self.inputGPSArraySub = self.create_subscription(GeoPath, self.inputGPSArrayTopic, self.new_gps_goal_array_cb, 10, callback_group=self.cb_group)
            self.fromLLclient = self.create_client(FromLL, "/fromLL", callback_group=self.cb_group)

        self.tfBuffer = tf2_ros.buffer.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.errorChecker = self.create_timer(self.errorCheckPeriod, self.error_check_cb)
        self.progressChecker = self.create_timer(self.progressCheckPeriod, self.progress_check_cb)

        self.navStatusSub = self.create_subscription(GoalStatusArray, self.navStatusTopic, self.navStatus_cb, 10)

        # Internal variables (guarded by goalLock)
        self.goalLock = Lock()
        self.waypoints = []            # list[PoseStamped] in globalFrame
        self.waypointIndex = 0
        self.currGlobalGoal = None
        self.currLocalGoal = None
        self.lastPublishTime = None    # Nav2 goals accepted before this belong to waypoints we've moved past
        self.requestGeneration = 0     # bumped on every new request so slow GPS conversions can't override newer goals
        self.navigating = False

    # ------------------------------------------------------------------ input callbacks

    def new_global_goal_cb(self, receivedGoalMsg: PoseStamped):
        p = receivedGoalMsg.pose.position
        self.get_logger().info(f"\n\nReceived new navigation goal in the {receivedGoalMsg.header.frame_id} frame: {p.x}, {p.y}")
        generation = self.bump_generation()
        globalGoal = self.to_global_frame(receivedGoalMsg)
        if globalGoal is None:
            return
        self.start_sequence([globalGoal], generation, autoHeading=self.overridePoseHeadings)

    def new_global_goal_array_cb(self, msg: PoseArray):
        if not msg.poses:
            self.get_logger().warn("Received an empty pose array; ignoring")
            return
        self.get_logger().info(f"\n\nReceived {len(msg.poses)} waypoints in the {msg.header.frame_id} frame")
        generation = self.bump_generation()

        globalGoals = []
        for pose in msg.poses:
            poseStamped = PoseStamped()
            poseStamped.header = msg.header
            poseStamped.pose = pose
            globalGoal = self.to_global_frame(poseStamped)
            if globalGoal is None:
                return
            globalGoals.append(globalGoal)

        self.start_sequence(globalGoals, generation, autoHeading=self.overridePoseHeadings)

    async def new_gps_goal_cb(self, msg: GeoPoint):
        self.get_logger().info(f"\n\nReceived new GPS waypoint navigation goal: {msg.latitude}, {msg.longitude}")
        await self.start_gps_sequence([msg])

    async def new_gps_goal_array_cb(self, msg: GeoPath):
        geoPoints = [geoPoseStamped.pose.position for geoPoseStamped in msg.poses]
        self.get_logger().info(f"\n\nReceived {len(geoPoints)} GPS waypoints")
        await self.start_gps_sequence(geoPoints)

    # ------------------------------------------------------------------ sequence setup

    async def start_gps_sequence(self, geoPoints):
        if not geoPoints:
            self.get_logger().warn("Received an empty GPS waypoint list; ignoring")
            return
        generation = self.bump_generation()

        if not self.wait_for_fromLL():
            return

        # convert each lat/long to the map frame, in order
        mapGoals = []
        for i, geoPoint in enumerate(geoPoints):
            request = FromLL.Request()
            request.ll_point.latitude = geoPoint.latitude
            request.ll_point.longitude = geoPoint.longitude
            request.ll_point.altitude = 0.0

            response = await self.fromLLclient.call_async(request)
            if response is None or (response.map_point.x == 0.0 and response.map_point.y == 0.0):
                self.get_logger().error(f"Cannot follow GPS goal: zero or empty map-frame coordinate received from navsat_transform_node for waypoint {i + 1}")
                return

            mapGoal = PoseStamped()
            mapGoal.header.frame_id = self.globalFrame
            mapGoal.header.stamp = self.get_clock().now().to_msg()
            mapGoal.pose.position.x = response.map_point.x
            mapGoal.pose.position.y = response.map_point.y
            mapGoals.append(mapGoal)

        # GPS points carry no heading, so always point each goal along the route
        self.start_sequence(mapGoals, generation, autoHeading=True)

    def wait_for_fromLL(self):
        i = 0
        while not self.fromLLclient.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{i} GPS goal requested but navsat_transform_node's fromLL service is not available yet")
            i += 1
            if i > 3:
                self.get_logger().error("Cannot follow GPS goal: Waited too long for navsat_transform_node's fromLL service to become available")
                return False
        return True

    def start_sequence(self, globalGoals, generation, autoHeading):
        if autoHeading:
            self.fill_headings(globalGoals)

        with self.goalLock:
            if generation != self.requestGeneration:
                self.get_logger().info("Discarding goal sequence: a newer goal arrived while it was being prepared")
                return
            self.waypoints = globalGoals
            self.waypointIndex = 0
            self.get_logger().info(f"Starting sequence of {len(globalGoals)} waypoint(s)")
            self.publish_current_waypoint_locked()

    def bump_generation(self):
        with self.goalLock:
            self.requestGeneration += 1
            return self.requestGeneration

    def fill_headings(self, goals):
        """Point each goal at the next one; the last goal faces along the final leg (or away from the robot if alone)."""
        for i, goal in enumerate(goals):
            p = goal.pose.position
            if i + 1 < len(goals):
                nxt = goals[i + 1].pose.position
                yaw = atan2(nxt.y - p.y, nxt.x - p.x)
            else:
                if i > 0:
                    prevXY = (goals[i - 1].pose.position.x, goals[i - 1].pose.position.y)
                else:
                    prevXY = self.robot_xy_global()
                yaw = 0.0 if prevXY is None else atan2(p.y - prevXY[1], p.x - prevXY[0])

            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = sin(yaw / 2.0)
            goal.pose.orientation.w = cos(yaw / 2.0)

    # ------------------------------------------------------------------ sequence state (call with goalLock held)

    def publish_current_waypoint_locked(self):
        globalGoal = self.waypoints[self.waypointIndex]
        localGoal = self.global_to_local(globalGoal)
        if localGoal is None:
            self.get_logger().error("Could not express waypoint in the local frame; abandoning sequence")
            self.clear_sequence_locked()
            return

        self.currGlobalGoal = globalGoal
        self.currLocalGoal = localGoal
        self.lastPublishTime = self.get_clock().now()  # set before publishing so Nav2's accept stamp is after it
        self.outputPub.publish(localGoal)
        self.outputRvizPub.publish(localGoal)

        p = globalGoal.pose.position
        self.get_logger().info(f"Navigating to waypoint {self.waypointIndex + 1}/{len(self.waypoints)}: ({p.x:.2f}, {p.y:.2f}) in {self.globalFrame}")

    def advance_locked(self):
        if self.waypointIndex + 1 >= len(self.waypoints):
            self.get_logger().info("\n\nWaypoint sequence complete\n")
            self.clear_sequence_locked()
            return
        self.waypointIndex += 1
        self.publish_current_waypoint_locked()

    def clear_sequence_locked(self):
        self.waypoints = []
        self.waypointIndex = 0
        self.currGlobalGoal = None
        self.currLocalGoal = None
        self.lastPublishTime = None

    # ------------------------------------------------------------------ periodic checks

    def progress_check_cb(self):
        """Capture intermediate waypoints by distance so the robot rolls through them instead of stopping."""
        with self.goalLock:
            if not self.navigating or not self.waypoints:
                return
            if self.waypointIndex >= len(self.waypoints) - 1:
                return  # final waypoint: let Nav2's goal checker decide when we've arrived

            robotXY = self.robot_xy_global()
            if robotXY is None:
                return
            goal = self.waypoints[self.waypointIndex].pose.position
            distance = dist(robotXY, (goal.x, goal.y))
            if distance <= self.captureRadius:
                self.get_logger().info(f"Captured waypoint {self.waypointIndex + 1}/{len(self.waypoints)} at {distance:.2f}m")
                self.advance_locked()

    def error_check_cb(self):
        """
        Transforms the current global goal to the local frame and compares it to the current local goal.
        If the distance between the two is too high, republish the local goal so it equals the global goal.
        """
        with self.goalLock:
            if not self.navigating or self.currGlobalGoal is None or self.currLocalGoal is None:
                return

            newLocalGoal = self.global_to_local(self.currGlobalGoal, warnOnly=True)
            if newLocalGoal is None:
                return

            # See it in rviz
            self.outputRvizPub.publish(newLocalGoal)

            # Error grows over time as the odometry drifts
            error = dist(
                (newLocalGoal.pose.position.x, newLocalGoal.pose.position.y),
                (self.currLocalGoal.pose.position.x, self.currLocalGoal.pose.position.y)
            )

            if error > self.errorBeforeRepublish:
                self.currLocalGoal = newLocalGoal
                # The preempted Nav2 goal will report ABORTED; this keeps that from being read as a failure
                self.lastPublishTime = self.get_clock().now()
                self.outputPub.publish(newLocalGoal)
                self.get_logger().info(f"\n\nCorrecting odom goal; error was {error:.2f}m.\n")

            elif self.debug:
                self.get_logger().info(f"Global to local goal error: {error}")

    def navStatus_cb(self, msg: GoalStatusArray):
        """
        Tracks whether Nav2 is navigating (so we never start autonomous motion without a human setting a goal),
        and advances or ends the sequence when the goal for the current waypoint finishes.
        """
        if not msg.status_list:
            return

        newest = max(msg.status_list, key=lambda s: Time.from_msg(s.goal_info.stamp).nanoseconds)
        status = newest.status

        with self.goalLock:
            self.navigating = status == GoalStatus.STATUS_EXECUTING

            if not self.waypoints or self.lastPublishTime is None:
                return
            # Goals accepted before our last publish belong to waypoints (or drift corrections) we've already moved past
            if Time.from_msg(newest.goal_info.stamp).nanoseconds < self.lastPublishTime.nanoseconds:
                return

            label = f"{self.waypointIndex + 1}/{len(self.waypoints)}"
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"Reached waypoint {label}")
                self.advance_locked()
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(f"Navigation canceled at waypoint {label}; ending sequence")
                self.clear_sequence_locked()
            elif status == GoalStatus.STATUS_ABORTED:
                if self.stopOnFailure:
                    self.get_logger().error(f"Nav2 aborted waypoint {label}; ending sequence")
                    self.clear_sequence_locked()
                else:
                    self.get_logger().warn(f"Nav2 aborted waypoint {label}; skipping it")
                    self.advance_locked()

    # ------------------------------------------------------------------ tf helpers

    def to_global_frame(self, goal: PoseStamped):
        receivedFrame = goal.header.frame_id
        if receivedFrame in ("", self.globalFrame):
            goal.header.frame_id = self.globalFrame
            return goal
        try:
            transform = self.tfBuffer.lookup_transform(self.globalFrame, receivedFrame, Time(), Duration(seconds=self.tfTimeout))
            return tf2_geometry_msgs.do_transform_pose_stamped(goal, transform)
        except Exception as e:
            self.get_logger().error(f"Transform exception ({receivedFrame} -> {self.globalFrame}): {e}")
            return None

    def global_to_local(self, globalGoal: PoseStamped, warnOnly=False):
        try:
            transform = self.tfBuffer.lookup_transform(self.localFrame, self.globalFrame, Time(), Duration(seconds=self.tfTimeout))
            return tf2_geometry_msgs.do_transform_pose_stamped(globalGoal, transform)
        except Exception as e:
            log = self.get_logger().warn if warnOnly else self.get_logger().error
            log(f"Transform exception ({self.globalFrame} -> {self.localFrame}): {e}")
            return None

    def robot_xy_global(self):
        try:
            t = self.tfBuffer.lookup_transform(self.globalFrame, self.robotFrame, Time(), Duration(seconds=self.tfTimeout))
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as e:
            if self.debug:
                self.get_logger().warn(f"Transform exception ({self.globalFrame} -> {self.robotFrame}): {e}")
            return None

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = MapToOdomGoalConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()