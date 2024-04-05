import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import serial
import struct

SERIAL_PORT = 'COM3'
SERIAL_BAUDRATE = 115200

class MotorController(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_info = self.create_publisher(JointState, 'info', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['leftWheel', 'rightWheel']
        self.joint_state_msg.position = [0.0, 0.0]
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.ser = None
        self.open_serial()

    def open_serial(self):
        # Open serial communication with motor controller
        self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0.5, bytesize=8)
        self.ser.set_buffer_size(rx_size=12800, tx_size=12800)

    def close_serial(self):
        # Close serial communication with motor controller
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def publish_joint_states(self):
        # Read motor controller data from serial
        try:
            self.ser.write(b'd')
            data_bytes = self.ser.read(16)
            leftPos, leftVel, rightPos, rightVel = struct.unpack('ffff', data_bytes)
        except Exception as e:
            self.get_logger().error(f"Error reading data from serial port: {e}")

        # publish
        self.joint_state_msg.position = [leftPos, rightPos]
        self.joint_state_msg.velocity = [leftVel, rightVel]
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_info.publish(self.joint_state_msg)

    def cmd_vel_callback(self, msg):
        # write new motor controller setpoint to serial
        try:
            self.ser.write(b't')
            data = struct.pack('ff', msg.linear.x, msg.angular.z)
            self.ser.write(data)
        except Exception as e:
            self.get_logger().error(f"Error writing data to serial port: {e}")

def main(args=None):
    rclpy.init(args=args)
    motorcontroller = MotorController()
    motorcontroller.open_serial()
    rclpy.spin(motorcontroller)
    motorcontroller.close_serial()
    motorcontroller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()