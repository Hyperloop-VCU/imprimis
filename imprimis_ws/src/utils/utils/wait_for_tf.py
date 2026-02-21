import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration


class WaitForTf(Node):
    """
    Waits for a valid transform from source ->  target frame, then exits.
    """

    def __init__(self):
        super().__init__("wait_for_tf")

        # params
        self.source_frame = self.declare_parameter("source_frame", "odom").value
        self.target_frame = self.declare_parameter("target_frame", "base_link").value
        self.output_error = self.declare_parameter("output_error", False).value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_check_timer = self.create_timer(0.02, self.tf_check_cb)

    def tf_check_cb(self):
        try:
            now = self.get_clock().now()
            transform = self.tf_buffer.can_transform(self.target_frame, self.source_frame, now)
        except Exception as e:
            if self.output_error: self.get_logger().warn(f"{e}")
            return
        self.get_logger().info(f"Transform from {self.source_frame} to {self.target_frame} available, exiting...")
        rclpy.shutdown()

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = WaitForTf()
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
