#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import signal
import sys


MOVING_THRESHOLD = 0.05



class VelocityTracker(Node):
    def __init__(self):
        super().__init__('velocity_tracker')
        self.sub = self.create_subscription(TwistStamped, '/cmd_vel_raw', self.cb, 10)
        self.samples = []
        self.get_logger().info("Tracking /cmd_vel_raw - Ctrl+C to print summary")
        

    def cb(self,msg):
        vx = abs(msg.twist.linear.x)
        if vx > MOVING_THRESHOLD:
            self.samples.append(vx)
            print(f"\r  vx: {vx:.3f} m/s  |  avg: {sum(self.samples)/len(self.samples):.3f}  |  max: {max(self.samples):.3f}  |  n={len(self.samples)}", end="", flush=True)

    def summary(self):
        if not self.samples:
            return
        avg = sum(self.samples) / len(self.samples)
        with open('/tmp/velocity_summary.txt', 'w') as f:
            f.write(f"Samples : {len(self.samples)}\n")
            f.write(f"Avg vx  : {avg:.4f} m/s\n")
            f.write(f"Max vx  : {max(self.samples):.4f} m/s\n")
            f.write(f"Min vx  : {min(self.samples):.4f} m/s\n")
        self.get_logger().info(f"Summary saved to /tmp/velocity_summary.txt")

            
def main():
    rclpy.init()
    node = VelocityTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


