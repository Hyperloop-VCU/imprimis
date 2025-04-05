# my_python_package/cmd_vel_listener.py


# all of these must match the corresponding values in the MotionController config.h
SERIAL_BAUD_RATE = 115200
MOTDATA_PUBLISH_RATE = 0.05 # must match DT
COUNTS_PER_REV = 3750
HALF_WHEEL_TRACK_LENGTH = 0.4432
WHEEL_RADIUS = 0.1651

from math import pi
CPL_2_ANGVEL = (2*pi) / (COUNTS_PER_REV * MOTDATA_PUBLISH_RATE)
CPL_2_LINVEL = CPL_2_ANGVEL * WHEEL_RADIUS
WHEEL_TRACK_LENGTH = HALF_WHEEL_TRACK_LENGTH * 2
# V_lwheel = V_robot - HALF_WHEEL_TRACK_LENGTH*w_robot
# V_rwheel = V_robot - HALF_WHEEL_TRACK_LENGTH*w_robot


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import serial
import struct

class MotInterface(Node):
    def __init__(self):
        super().__init__('motInterface')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Twist, 'motInterface/velocity', 10)
        self.timer = self.create_timer(MOTDATA_PUBLISH_RATE, self.publishMotData_callback)
        self.reset_srv = self.create_service(Empty, 'motInterface/reset_encoders', self.resetEnc_callback)
        self.ser = serial.Serial("/dev/ttyUSB0", baudrate=SERIAL_BAUD_RATE, timeout=1.0)
        self.get_logger().info("Motor interface node started")

    def getCPL(self):
        data = self.ser.read(8) # two longs
        left = int.from_bytes(data[0:4], byteorder='little', signed=True)
        right = int.from_bytes(data[4:8], byteorder='little', signed=True)
        return left, right
    
    def publishMotData_callback(self):
        msg = Twist()
        lCPL, rCPL = self.getCPL()
        V_lwheel, V_rwheel = lCPL * CPL_2_LINVEL, rCPL * CPL_2_LINVEL
        angvel_robot = (V_rwheel - V_lwheel) / WHEEL_TRACK_LENGTH
        linvel_robot = V_lwheel + HALF_WHEEL_TRACK_LENGTH * angvel_robot
        # linvel_robot = V_rwheel - HALF_WHEEL_TRACK_LENGTH * angvel_robot
        msg.angular.z = angvel_robot
        msg.linear.x = linvel_robot
        self.publisher.publish(msg)

    def resetEnc_callback(self, request, response):
        self.ser.write(b'r')
        self.get_logger().info("Sent reset encoder command to arduino")
        self.leftPrevEncCount = 0
        self.rightPrevEncCount = 0
        return Empty.Response()

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info(f"Received cmd_vel - Linear X: {msg.linear.x}, Angular Z: {msg.angular.z}")
        self.ser.write(b's')
        data = struct.pack('ff', msg.linear.x, msg.angular.z)
        self.ser.write(data)
    
def main():
    rclpy.init()
    node = MotInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()