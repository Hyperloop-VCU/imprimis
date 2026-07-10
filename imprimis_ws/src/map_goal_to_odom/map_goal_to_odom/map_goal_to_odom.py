import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from action_msgs.msg import GoalStatusArray
from geographic_msgs.msg import GeoPoint
import tf2_ros
import tf2_geometry_msgs
from threading import Lock
from math import dist
from robot_localization.srv import FromLL, FromLL_Request, FromLL_Response

class MapToOdomGoalConverter(Node):
    """
    Converts a pose message from the global frame (map) to the local frame (odom).
    Republishes a local goal to outputTopic if either of the following happen:
        - A new goal was published to inputTopic
        - The globalFrame to localFrame transform has deviated by more than errorBeforeRepublish (m) since the last time we updated it
    
    Additionally, GPS NavSatFix messages can be published to inputGPSTopic. On receiving one,
    this node will use navsat_transform_node's /FromLL service to convert it to a map goal, handles it the same as as any other map goal.
    
    If there is no navigation happening (according to navStatusTopic), this node will not publish a local goal.

    errorBeforeRepublish is a crucial parameter. It should be higher than the acceptable map jitter but low enough to account for odom drift.
    """

    def __init__(self):
        super().__init__("map_goal_to_odom")

        # Params
        self.globalFrame = self.declare_parameter("globalFrame", "map").value
        self.localFrame = self.declare_parameter("localFrame", "odom").value
        self.inputTopic = self.declare_parameter("inputTopic", "goal_pose").value
        self.inputGPSTopic = self.declare_parameter("inputGPSTopic", "gps_goal").value
        self.outputTopic = self.declare_parameter("outputTopic", "odom_goal_pose").value
        self.outputRvizTopic = self.declare_parameter("outputRvizTopic", "odom_goal_pose_rviz").value
        self.errorBeforeRepublish = self.declare_parameter("errorBeforeRepublish", 3.0).value  # meters
        self.errorCheckPeriod = self.declare_parameter("errorCheckPeriod", 1.0).value # seconds
        self.navStatusTopic = self.declare_parameter("navStatusTopic", "navigate_to_pose/_action/status").value
        self.tfTimeout = self.declare_parameter("tfTimeout", 0.02).value # seconds
        self.debug = self.declare_parameter("debug", False).value

        # ROS objects
        self.outputPub = self.create_publisher(PoseStamped, self.outputTopic, 10)
        self.outputRvizPub = self.create_publisher(PoseStamped, self.outputRvizTopic, 10)
        self.inputSub = self.create_subscription(PoseStamped, self.inputTopic, self.new_global_goal_cb, 10)
        self.inputGPSSub = self.create_subscription(NavSatFix, self.inputGPSTopic, self.new_gps_goal_cb, 10)
        self.tfBuffer = tf2_ros.buffer.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.errorChecker = self.create_timer(self.errorCheckPeriod, self.error_check_cb)
        self.navStatusSub = self.create_subscription(GoalStatusArray, self.navStatusTopic, self.navStatus_cb, 10)
        self.fromLLclient = self.create_client(FromLL, "/fromLL")

        # Internal variables
        self.currGlobalGoal = None
        self.currLocalGoal = None
        self.goalLock = Lock()
        self.navigating = False 

    def new_gps_goal_cb(self, receivedFixMsg: NavSatFix):
        
        # convert NavSatFix to GeoPoint
        self.get_logger().info(f"\n\nReceived new GPS waypoint navigation goal: {receivedFixMsg.latitude}, {receivedFixMsg.longitude}")
        geoPointMsg = FromLL_Request()
        geoPointMsg.ll_point.latitude = receivedFixMsg.latitude
        geoPointMsg.ll_point.longitude = receivedFixMsg.longitude
        geoPointMsg.ll_point.altitude = 0.0

        # call navsat_transform_node's /FromLL service to convert it to a map goal coordinate
        i = 0
        while not self.fromLLclient.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{i} GPS goal requested but navsat_transform_node\'s fromLL service is not available yet')
            i += 1
            if i > 3:
                self.get_logger().error('Cannot follow GPS goal: Waited too long for navsat_transform_node\'s fromLL service to become available')
                return
        self.get_logger().info(f"Called the service {geoPointMsg.ll_point.latitude} {geoPointMsg.ll_point.longitude}")
        mapGoalCoords: FromLL_Response = self.fromLLclient.call(geoPointMsg)
        self.get_logger().info(f"\n\nService completed")
        if mapGoalCoords is None or mapGoalCoords.map_point.x == 0.0 or mapGoalCoords.map_point.y == 0.0:
            self.get_logger().error('Cannot follow GPS goal: zero or empty map coordinate received from navsat_transform_node')


        # convert it to a PoseStamped, and handle it the exact same as a regular map goal
        mapGoal = PoseStamped()
        mapGoal.pose.position.x = mapGoalCoords.map_point.x
        mapGoal.pose.position.y = mapGoalCoords.map_point.y
        mapGoal.header.stamp = self.get_clock().now().to_msg()
        self.new_global_goal_cb(mapGoal)

    def new_global_goal_cb(self, receivedGoalMsg: PoseStamped):
        """
        Callback that runs when a new global goal is published.
        If it isn't in the global frame, transform it to that frame, then transform it to the local frame.
        Publish it, then save the global goal.
        """  
        localGoalMsg = PoseStamped()

        # If goal was given in the wrong frame, transform it to the global frame
        receivedFrame = receivedGoalMsg.header.frame_id
        self.get_logger().info(f"\n\nReceived new navigation goal in the {receivedFrame} frame: {receivedGoalMsg.pose.position.x}, {receivedGoalMsg.pose.position.y}")
        if receivedFrame != self.globalFrame:
            try:
                if self.tfBuffer.can_transform(receivedFrame, self.globalFrame, Time(), Duration(seconds=self.tfTimeout)):
                    receivedToGlobalTransform = self.tfBuffer.lookup_transform(self.globalFrame, receivedFrame, Time(), Duration(seconds=self.tfTimeout))
                globalGoalMsg = tf2_geometry_msgs.do_transform_pose_stamped(receivedGoalMsg, receivedToGlobalTransform)
            except Exception as e:
                self.get_logger().error(f"Transform exception ({receivedFrame} -> {self.globalFrame}): {e}")
                return
        else:
            globalGoalMsg = receivedGoalMsg

        # Transform goal from global to local frame
        try:
            if self.tfBuffer.can_transform(self.globalFrame, self.localFrame, Time(), Duration(seconds=self.tfTimeout)):
                globalToLocalTransform = self.tfBuffer.lookup_transform(self.localFrame, self.globalFrame, Time(), Duration(seconds=self.tfTimeout))
            localGoalMsg = tf2_geometry_msgs.do_transform_pose_stamped(globalGoalMsg, globalToLocalTransform)
        except Exception as e:
            self.get_logger().error(f"Transform exception ({self.globalFrame} -> {self.localFrame}): {e}")
            return

        # Publish and update state
        self.outputPub.publish(localGoalMsg)
        self.outputRvizPub.publish(localGoalMsg)
        with self.goalLock:
            self.currGlobalGoal = globalGoalMsg
            self.currLocalGoal = localGoalMsg


    def error_check_cb(self):
        """
        This method transforms the current global goal to the local frame and compares it to the current local goal.
        If the distance between the two is too high, republish the local goal so it equals the global goal.
        """
        with self.goalLock:
            if not self.navigating or self.currGlobalGoal is None or self.currLocalGoal is None: 
                return

            # Transform current global goal to local frame
            try:
                if self.tfBuffer.can_transform(self.globalFrame, self.localFrame, Time(), Duration(seconds=self.tfTimeout)):
                    transform = self.tfBuffer.lookup_transform(self.localFrame, self.globalFrame, Time(), Duration(seconds=self.tfTimeout))
                newLocalGoal = tf2_geometry_msgs.do_transform_pose_stamped(self.currGlobalGoal, transform)
            except Exception as e:
                self.get_logger().warn(f"Transform exception ({self.globalFrame} -> {self.localFrame}): {e}")
                return
            
            # See it in rviz
            self.outputRvizPub.publish(newLocalGoal)

            # Get error; the distance between the current local goal and the current global goal transformed into the local frame.
            # This error grows over time as the odometry drifts.
            error = dist(
                (newLocalGoal.pose.position.x, newLocalGoal.pose.position.y),
                (self.currLocalGoal.pose.position.x, self.currLocalGoal.pose.position.y)
            )
            
            # If the error is too high, republish the goal to make the error zero again.
            if error > self.errorBeforeRepublish:
                self.outputPub.publish(newLocalGoal)
                self.currLocalGoal = newLocalGoal
                self.get_logger().info(f"\n\nCorrecting odom goal; error was {error:.2f}m.\n")

            elif self.debug:
                self.get_logger().info(f"Global to local goal error: {error}")
    

    def navStatus_cb(self, msg: GoalStatusArray):
        """
        This callback ensures we don't republish a goal and trigger a navigation sequence without a human setting one.
        Without it, a new local goal will be published if the error gets too big, triggering unexpected autonomous activity.
        """
        status = msg.status_list[-1].status
        if status == 2:
            self.navigating = True
        else:
            self.navigating = False

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
