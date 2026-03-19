#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml, math, os
from nav2_msgs.srv import GetCostmap
from lifecycle_msgs.srv import GetState

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.declare_parameter('waypoints_file', '')
        path = self.get_parameter('waypoints_file').value
        with open(path) as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']
        self.index = 0
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        self._client.wait_for_server()
        import time
        time.sleep(3.0)
        self.send_next()

    def send_next(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info("All waypoints complete")
            return
        wp = self.waypoints[self.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'odom'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        yaw = float(wp.get('yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info(f"Sending waypoint {self.index + 1}/{len(self.waypoints)}: ({wp['x']}, {wp['y']})")
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected")
            return
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        self.index += 1
        self.send_next()

def main():
    rclpy.init()
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()