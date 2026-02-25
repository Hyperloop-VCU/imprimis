import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import PointCloud2
from time import sleep


class WaitForTf(Node):
    """
    Waits for a valid transform from source ->  target frame, then exits.
    Can also block until messages on a PointCloud2 topic are available.
    """

    def __init__(self):
        super().__init__("wait_for_tf")

        # params
        self.source_frame = self.declare_parameter("source_frame", "odom").value
        self.target_frame = self.declare_parameter("target_frame", "base_link").value
        self.output_error = self.declare_parameter("output_error", False).value
        self.use_topic = self.declare_parameter("use_topic", False).value
        self.topic = self.declare_parameter("topic", "velodyne_points").value
        self.msgs_required = self.declare_parameter("msgs_required", 10).value
        self.additional_delay = self.declare_parameter("additional_delay", 0.05).value

        self.canExit = not self.use_topic
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_check_timer = self.create_timer(0.02, self.tf_check_cb)
        self.msgs_received = 0
        if self.use_topic:
            self.pointcloud_sub = self.create_subscription(PointCloud2, self.topic, self.pointcloud_cb, 10)

    def tf_check_cb(self):
        try:
            if not self.tf_buffer.can_transform(self.target_frame, self.source_frame, time=Time()):
                if self.output_error: 
                    self.get_logger().info(f"Waiting for transform from {self.source_frame} to {self.target_frame}")
                return
            
        except Exception as e:
            if self.output_error: 
                self.get_logger().warn(f"{e}")
            return
        
        self.get_logger().info(f"\n\nTf {self.source_frame} -> {self.target_frame} available! :)\n", throttle_duration_sec=5.0)
        if not self.canExit:
            return
        if self.use_topic:
            self.get_logger().info(f"\n\nLidar data available on {self.topic}! :)\n")
        sleep(self.additional_delay)
        rclpy.shutdown()

    def pointcloud_cb(self, msg):
        self.msgs_received += 1
        if self.msgs_received >= self.msgs_required:
            self.canExit = True

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
