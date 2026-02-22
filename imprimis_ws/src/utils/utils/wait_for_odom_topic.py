import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from time import sleep


class WaitForOdom(Node):
    """
    Waits for Odom messages on a topic.
    """

    def __init__(self):
        super().__init__("wait_for_odom")

        # params
        self.topic = self.declare_parameter("topic", "diffbot_base_controller/odom").value
        self.additional_delay = self.declare_parameter("additional_delay", 0.05).value

        self.sub = self.create_subscription(Odometry, self.topic, self.sub_cb, 10)

    def sub_cb(self, msg):
        self.get_logger().info(f"Odometry source is ready on {self.topic}. :)")
        rclpy.shutdown()

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = WaitForOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
