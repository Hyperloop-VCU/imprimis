import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSMonitor(Node):
    """
    Very simple node that screams out if the GPS has a fix or not.
    No other node does this! Not navsat transform, not the global EKF, nothing.
    """

    def __init__(self):
        super().__init__("gps_monitor")
        self.fix_input_topic = self.declare_parameter("fix_input_topic", "/gps/fix").value
        self.fix_input_sub = self.create_subscription(NavSatFix, self.fix_input_topic, self.fix_input_cb, 10)

    def fix_input_cb(self, msg):
        if msg.status.status == -1:
            self.get_logger().warn("WARNING: Using GPS-based global localization, but the GPS has no signal!")
    
    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = GPSMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
