#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy




class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')

        #parameters, threshold to swap to mppi, radius it checks for obstacles, and cost threshold to swap back to RPP
        #obstacle cost should be higher than swap back and not equal so that they do not keep swapping back and forth
        self.declare_parameter('obstacle_cost_threshold', 80)
        self.declare_parameter('check_radius', 1.5)
        self.declare_parameter('swap_back_threshold',50)


        self.obstacle_cost_threshold = self.get_parameter('obstacle_cost_threshold').value
        self.check_radius = self.get_parameter('check_radius').value
        self.swap_back_threshold = self.get_parameter('swap_back_threshold').value



        #declare the currentcontroller(by default RPP) and initialize costmaps
        self.current_controller = 'FollowPath'
        self.costmap = None
        self.costmap_info = None


        #publisher, sends strings to topic /controller_selector, durability must be transient_local
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.controller_pub = self.create_publisher(String, '/controller_selector', qos)

        #subscrube to local costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10
        )

        #timer to call function at 2 hz
        self.timer = self.create_timer(0.5, self.check_and_switch)

        self.get_logger().info('Controller switcher started')


    # get costmap data like height and width
    # the array is used to turn a list of data and turn it into a 2d grid that can be used to map each costvalue to the actual cell on the costmap
    def costmap_callback(self,msg):
        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.costmap_info = msg.info


    def get_max_cost_near_robot(self):
        #make sure costmap has recieved data before starting
        if self.costmap is None or self.costmap_info is None:
            return 0
            


        resolution = self.costmap_info.resolution
        width = self.costmap_info.width
        height = self.costmap_info.height


        #checking from middle of the robot
        center_x = width // 2
        center_y = height // 2

        #how many cells to check around robot rather than meters
        radius_cells = int(self.check_radius / resolution)

        #square around checking radius
        x_min = max(0, center_x - radius_cells)
        x_max = min(width, center_x + radius_cells)
        y_min = max(0, center_y - radius_cells)
        y_max = min(height, center_y + radius_cells)

        #only check cost values in the defined square
        region = self.costmap[y_min:y_max, x_min:x_max]



        known = region[region >=0 ]
        if len(known) == 0:
            return 0
        
        return int(np.max(known))
    

    def check_and_switch(self):
        max_cost = self.get_max_cost_near_robot()


        if self.current_controller == 'FollowPath' and max_cost >= self.obstacle_cost_threshold:
            self.get_logger().info(f'obstacles detected (cost = {max_cost}), swtiching to MPPI')
            self.switch_to('MPPI')

        elif self.current_controller == 'MPPI' and max_cost < self.swap_back_threshold:
            self.get_logger().info(f'Clear of obstacles (cost ={max_cost}), switching to RPP')
            self.switch_to('FollowPath')


    def switch_to(self, controller_name):
        msg = String()
        msg.data = controller_name
        self.controller_pub.publish(msg)
        self.current_controller = controller_name


def main(args=None):
    rclpy.init(args=args)
    node = ControllerSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

    
