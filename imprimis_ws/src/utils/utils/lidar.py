import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class VelodyneRepublisher(Node):
    """
    Subscribes to a pointcloud2 topic, and republishes it with its timestamp slightly in the past.
    This is needed because the velodyne pointcloud data arrives with a timestamp that's slightly off.
    This ~0.05 second error is enough to prevent nodes that use this topic from transforming from the lidar frame to odom or map,
    so this node just edits the timestamp slightly to hide that tiny delay.
    """

    def __init__(self):
        super().__init__("lidar_pointcloud2_republisher")

        # params
        self.lidar_input_topic = self.declare_parameter("lidar_input_topic", "/velodyne_points").value
        self.lidar_output_topic = self.declare_parameter("lidar_output_topic", "/velodyne_points_fixed").value
        self.time_offset = self.declare_parameter("time_offset", 0.2).value

        self.time_offset_ns = int(self.time_offset * (10 ** 9))

        # pub and sub
        self.lidar_output_pub = self.create_publisher(PointCloud2, self.lidar_output_topic, 10)
        self.lidar_input_sub = self.create_subscription(PointCloud2, self.lidar_input_topic, self.lidar_input_cb, 10)

    def lidar_input_cb(self, msg):
        outputMsg = msg
        if outputMsg.header.stamp.sec < 1:
            return
        if outputMsg.header.stamp.nanosec > self.time_offset_ns:
            outputMsg.header.stamp.nanosec -= self.time_offset_ns
        else:
            outputMsg.header.stamp.sec -= 1
            outputMsg.header.stamp.nanosec = (10 ** 9) - (self.time_offset_ns - outputMsg.header.stamp.nanosec)
        
        self.lidar_output_pub.publish(outputMsg)
    

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = VelodyneRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
