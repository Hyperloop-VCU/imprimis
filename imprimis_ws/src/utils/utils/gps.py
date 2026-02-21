import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSFixRepublisher(Node):
    """
    Subscribes to a NavSatFix topic, sets its covariance to a fixed value, then republishes it on a different topic.
    This is needed when GPS data comes from gazebo; unfortunately, there's no way to tell gazebo to output the topic with nonzero covariance.
    """

    def __init__(self):
        super().__init__("gps_fix_republisher")

        # params
        self.fix_input_topic = self.declare_parameter("fix_input_topic", "/gps/fix_no_cov").value
        self.fix_output_topic = self.declare_parameter("fix_output_topic", "/gps/fix").value
        self.covariance = self.declare_parameter("covariance", 0.1).value

        # pub and sub
        self.fix_output_pub = self.create_publisher(NavSatFix, self.fix_output_topic, 10)
        self.fix_input_sub = self.create_subscription(NavSatFix, self.fix_input_topic, self.fix_input_cb, 10)

    def fix_input_cb(self, msg):
        outputMsg = msg
        outputMsg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        for i in range(9):
            msg.position_covariance[i] = self.covariance
        self.fix_output_pub.publish(outputMsg)
    

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = GPSFixRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
