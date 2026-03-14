import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray
import tf2_ros
import tf2_geometry_msgs
from threading import Lock
from math import dist

class MapToOdomGoalConverter(Node):
    """
    Converts a pose message from the global frame (map) to the local frame (odom).
    Republishes a local goal to outputTopic if any of the following are true:
        - A new goal was published to inputTopic
        - The globalFrame to localFrame transform has deviated by more than errorBeforeRepublish (m) since the last time we published to outputTopic
    
    If there is no navigation happening (according to navStatusTopic), this node will not publish a local goal.

    errorBeforeRepublish is a crucial parameter. It should be higher than the acceptable map jitter but low enough to account for odom drift.
    """

    def __init__(self):
        super().__init__("map_goal_to_odom")

        # Params
        self.globalFrame = self.declare_parameter("globalFrame", "map").value
        self.localFrame = self.declare_parameter("localFrame", "odom").value
        self.inputTopic = self.declare_parameter("inputTopic", "goal_pose").value
        self.outputTopic = self.declare_parameter("outputTopic", "odom_goal_pose").value
        self.outputRvizTopic = self.declare_parameter("outputRvizTopic", "odom_goal_pose_rviz").value
        self.errorBeforeRepublish = self.declare_parameter("errorBeforeRepublish", 3.0).value  # meters
        self.errorCheckPeriod = self.declare_parameter("errorCheckPeriod", 1.0).value
        self.navStatusTopic = self.declare_parameter("navStatusTopic", "navigate_to_pose/_action/status").value
        self.tfTimeout = self.declare_parameter("tfTimeout", 0.02).value
        self.debug = self.declare_parameter("debug", False).value

        # ROS objects
        self.outputPub = self.create_publisher(PoseStamped, self.outputTopic, 10)
        self.outputRvizPub = self.create_publisher(PoseStamped, self.outputRvizTopic, 10)
        self.inputSub = self.create_subscription(PoseStamped, self.inputTopic, self.new_global_goal_cb, 10)
        self.tfBuffer = tf2_ros.buffer.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.errorChecker = self.create_timer(self.errorCheckPeriod, self.error_check_cb)
        self.navStatusSub = self.create_subscription(GoalStatusArray, self.navStatusTopic, self.navStatus_cb, 10)

        # Internal variables
        self.currGlobalGoal = None
        self.currLocalGoal = None
        self.goalLock = Lock()
        self.navigating = False 

    def new_global_goal_cb(self, receivedGoalMsg: PoseStamped):
        """
        Callback that runs when a new global goal is published.
        If it isn't in the global frame, transform it to that frame, then transform it to the local frame.
        Publish it, then save the global goal.
        """  
        localGoalMsg = PoseStamped()

        # If goal was given in the wrong frame, transform it to the global frame
        receivedFrame = receivedGoalMsg.header.frame_id
        if receivedFrame != self.globalFrame:
            self.get_logger().info(f"\n\nReceived goal in the {receivedFrame} frame, transforming it to the {self.globalFrame} frame.\n")
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
