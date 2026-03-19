import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomRemapper(Node):
    """
    Republishes odometry messages on inputTopic to outputTopic.
    Why don't I do this with a proper remapping?
    Because I need to remap the diff drive controller, which is spawned by a special spawner node. The node is not spawned directly in the launch file.
    This spawner DOES have an argument --controller-ros-args to give params and remappings to the controller, but it just does not work, no matter what I tried.
    The error is: --controller-ros-args: expected one argument
    """
    def __init__(self):
        super().__init__("odom_remapper")
        
        # declare params
        self.inputTopic = self.declare_parameter("inputTopic", "diffbot_base_controller/odom").value 
        self.outputTopic = self.declare_parameter("outputTopic", "odometry/filtered/local").value
        
        # create pub and sub
        self.pub = self.create_publisher(Odometry, self.outputTopic, 10)
        self.sub = self.create_subscription(Odometry, self.inputTopic, self.sub_cb, 10)
    
    def sub_cb(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()

    node = OdomRemapper()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
