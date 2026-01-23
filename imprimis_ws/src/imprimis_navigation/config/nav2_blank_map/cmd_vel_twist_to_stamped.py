# cmd_vel_twist_to_stamped.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped_adapter')
        self.in_topic = self.declare_parameter('in_topic', '/cmd_vel').value
        self.out_topic = self.declare_parameter('out_topic', '/diffbot_base_controller/cmd_vel').value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        self.pub = self.create_publisher(TwistStamped, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self.cb, 10)
        self.get_logger().info(f"Adapting {self.in_topic} (Twist) -> {self.out_topic} (TwistStamped)")

    def cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)

def main():
    rclpy.init()
    node = TwistToStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
