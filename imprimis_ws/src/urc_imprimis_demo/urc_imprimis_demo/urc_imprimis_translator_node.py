import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Joy

class UrcImprimisTranslator(Node):

    def __init__(self):
        """
        Declare parameters,
        create publisher to robot topic, 
        and create subscriber to URC topic
        """
        super().__init__('imu_arm_control_node')

        self.declare_parameter("scaleAngularZ", 0.5)
        self.declare_parameter("scaleLinearX", 0.5)

        self.robot_topic_publisher = self.create_publisher(
            TwistStamped,
            'diffbot_base_controller/cmd_vel',
            qos_profile=10
        )

        self.urc_topic_subscription = self.create_subscription(
            Imu, 
            'bno055/imu', # urc topics
            self.urc_robot_mapping_function, 
            qos_profile=10
        )



    def urc_robot_mapping_function(self, urcInput : Imu):
        """
        This is the 'function' I was talking about, it defines the mapping of urc->robot.
        In advanced mode, the users will write this function themselves.
        In basic mode, the users will have some block coding interface / GUI for writing this function.
        """
        outputMsg = TwistStamped()
        outputMsg.header.stamp = self.get_clock().now().to_msg()

        scaleAngularZ = self.get_parameter("scaleAngularZ").get_parameter_value().double_value()
        scaleLinearX = self.get_parameter("scaleLinearX").get_parameter_value().double_value()
        imuRoll, imuPitch, imuYaw = euler_from_quaternion(
            urcInput.orientation.x, 
            urcInput.orientation.y, 
            urcInput.orientation.z, 
            urcInput.orientation.w
        )

        outputMsg.twist.angular.z = urcInput.angular_velocity.z * scaleAngularZ
        outputMsg.twist.linear.x = imuPitch * scaleLinearX

        self.robot_topic_publisher.publish(outputMsg)



def main(args=None):
    """
    Classic ROS boilerplate
    """
    rclpy.init(args=args)

    node = UrcImprimisTranslator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()