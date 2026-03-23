#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import yaml, math, os
from nav2_msgs.srv import GetCostmap
from lifecycle_msgs.srv import GetState
from action_msgs.msg import GoalStatusArray

class WaypointSender(Node):
    def __init__(self):
        #Declarations
        super().__init__('waypoint_sender')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.declare_parameter('waypoints_file', '')
        path = self.get_parameter('waypoints_file').value
        self.status_sub = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_cb, 10)
        self.retry_counter = 0
       # self.nav2_ready = False
       # self.first_send = True
        
        
        #check waypoints.yaml
        with open(path) as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']
        self.index = 0
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        
        #wait for everything to initialize before sending a goal
        import time
        time.sleep(5.0)
        self.send_next()

    #send goal function(sending to /goal_pose so map_goal_to_odom node can translate it to odom goal)
    def send_next(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info("All waypoints complete")
            return
        wp = self.waypoints[self.index]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(wp['x'])
        goal.pose.position.y = float(wp['y'])
        yaw = float(wp.get('yaw', 0.0)) #not really useful cuz we aren't looking for goal orientation right now
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info(f"Sending waypoint {self.index + 1}/{len(self.waypoints)}: ({wp['x']}, {wp['y']})")
        self.goal_pub.publish(goal)
    
    #check if goal succeeded or failed, and call send_next if there are more than one waypoint
    def status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
       # self.nav2_ready = True

      #  if self.index == 0 and self.nav2_ready == True and self.first_send == True:
         #   self.send_next()
          #  self.first_send = False
            
        if status == 4:
            self.index += 1
            self.send_next()
        elif status == 6:
            if self.retry_counter <=3:
                self.retry_counter += 1
                self.send_next()
                

            else:
                self.index +=1
                self.retry_counter = 0
                self.send_next()
        
    
    

def main():
    rclpy.init()
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()