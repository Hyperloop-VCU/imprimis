import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class WorkshopDemo(Node):
    """
    Publishes a message to diffbot_base_controller/cmd_vel repeatedly
    """

    def __init__(self):
        super().__init__("workshop_demo_node")

        self.rotation_speed = 1.0
        self.linear_speed = 2.0

        self.get_logger().info("Starting workshop demo node. Publishing:")
        self.get_logger().info(f"Rotation speed = {self.rotation_speed}")
        self.get_logger().info(f"Linear speed = {self.linear_speed}")

        self.cmd_vel_pub = self.create_publisher(TwistStamped, "diffbot_base_controller/cmd_vel", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_msg_cb)

    def publish_msg_cb(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.linear_speed
        msg.twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(msg)

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = WorkshopDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
