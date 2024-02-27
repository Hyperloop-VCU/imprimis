# Input 1: /cmd_vel twist message in m/sec for linear and rad/sec for angular - Only linear.x and angular.z components are needed
# Input 2: Six encoder inputs from pins
# Output 1: PWM duty cycle for speed to pin (one for each motor)
# Output 2: Single bit for direction to pin (one for each motor)
# Motor Driver DIP switch operating mode: 1011000 (PWM, both motors, linear)

"""
# PINS FOR INPUT
    + 5, 6, 16 (Left Encoder)
        + 5 -> A
        + 6 -> B
        + 16 -> index
    + 24, 25, 26 (Right Encoder)
        + 24 -> A
        + 25 -> B
        + 26 -> index

----------------------------------------------------------------

+ index -> origin point to check when system is restarted
    + dont start reading input until you get high from the index
+ A & B -> pulses in which are compared to figure out direction

+ These inputs are present for both encoders seperately

+ B leads A for clockwise shaft rotation, and A leads B for counterclockwise rotation viewed from the cover side of the encoder.

+ Our encoder is 500 pulses per revolution
"""

from pid_object import PIDController
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

# TODO figure out what all these are
WHEEL_RADIUS = 0.1651  # meters
WHEEL_TRACK_LENGTH = 0.8865  # meters
MAX_RPM = 160
ENCODER_PULSES_PER_REV = 500  # I want to measure this manually, as they did in the articulated robotics video

LEFT_ENCODER_INPUT_PINA = 5
LEFT_ENCODER_INPUT_PINB = 6
LEFT_ENCODER_INDEX_PIN = 16
RIGHT_ENCODER_INPUT_PINA = 24
RIGHT_ENCODER_INPUT_PINB = 25
RIGHT_ENCODER_INDEX_PIN = 16
LEFT_SPEED_OUTPUT_PIN = None  # PWM
LEFT_DIRECTION_OUTPUT_PIN = None  # digital
RIGHT_SPEED_OUTPUT_PIN = None  # PWM
RIGHT_DIRECTION_OUTPUT_PIN = None  # digital

PID_LOOPS_PER_SEC = 30

# commonly used values
wheel_circumference = math.tau * WHEEL_RADIUS
inplace_turn_radius = WHEEL_TRACK_LENGTH / 2

def twist_to_PID_setpoint(twist_msg):
    """Converts the linear.x (m/s) and angular.z (rad/s) components of the twist msg into the desired
    pulses per PID loop (PPL) and direction of each motor.

    :returns: A tuple of two tuples of (desired encoder pulses per PID loop, desired direction), one for each motor."""
    base_rps = twist_msg.linear.x / wheel_circumference

    left_rps = (-twist_msg.angular.z * inplace_turn_radius) / wheel_circumference
    right_rps = (twist_msg.angular.z * inplace_turn_radius) / wheel_circumference

    left_rps += base_rps
    right_rps += base_rps

    left_ppl = (left_rps / PID_LOOPS_PER_SEC) * ENCODER_PULSES_PER_REV
    right_ppl = (right_rps / PID_LOOPS_PER_SEC) * ENCODER_PULSES_PER_REV
    left_dir = True if left_ppl >= 0 else False
    right_dir = True if right_ppl >= 0 else False
    return (abs(left_ppl), left_dir), (abs(right_ppl), right_dir)



# TODO set up subscriber node, read from encoders, write to pins
