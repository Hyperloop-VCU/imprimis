#!/usr/bin/env python3
"""Read the D435i's own accelerometer and print how far the camera looks down (tilt) and how far it leans (roll).

Use it on the tripod to set and record the camera angle instead of a phone level:
    1. Start the camera as usual (ros2 launch imprimis_perception tripod.launch.py ...).
    2. python3 imu_tilt.py
       It turns the camera's accelerometer on (the enable_accel parameter of the camera node; the launch files
       leave it off), then prints the tilt and roll once a second, averaged over the last 2 s.
       Adjust the tripod until the tilt reads what you want, hold still, then press Ctrl-C.
    3. On Ctrl-C it prints the final numbers and saves them to harness/out/imu_tilt.yaml.
       Use the tilt as tripod.launch.py's tilt:= argument.

The accelerometer measures gravity, so the angles are relative to level, not to the ground. On flat ground those
are the same; on a slope they differ by the slope.

The accelerometer stays on after this exits, so the bag can record /cameras/<camera>/accel/sample as well.

Options:
    --camera NAME     camera name (default d435i); the node is /cameras/NAME, topics /cameras/NAME/...
    --window S        seconds to average over (default 2.0)
    --no-enable       do not touch the camera's parameters (the accelerometer is already on)
"""
import argparse
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out')


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def angles(up):
    """Tilt and roll in degrees from the 'up' direction expressed in an optical frame (x right, y down, z forward).

    tilt > 0: the camera looks down. roll > 0: the right side of the image is lower than the left.
    """
    u = up / np.linalg.norm(up)
    tilt = math.degrees(math.asin(max(-1.0, min(1.0, -u[2]))))
    roll = math.degrees(math.atan2(-u[0], -u[1]))
    return tilt, roll


class ImuTilt(Node):
    def __init__(self, args):
        super().__init__('imu_tilt')
        self.args = args
        self.cam = args.camera
        self.samples = deque()          # (time, vector in the accel frame)
        self.frame = None
        self.rot = None                 # accel frame -> color optical frame, from TF
        self.rot_note = 'no TF yet; accelerometer axes used as they are'
        self.flipped = False
        self.count = 0
        self.create_subscription(Imu, f'/cameras/{self.cam}/accel/sample', self.on_imu, qos_profile_sensor_data)
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.report)
        self.start = time.time()
        self.warned = False

    def enable_accel(self):
        client = self.create_client(SetParameters, f'/cameras/{self.cam}/set_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            print(f'Camera node /cameras/{self.cam} not found. Is the camera running?')
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(name='enable_accel',
                                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))]
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None or not res.results or not res.results[0].successful:
            reason = res.results[0].reason if res and res.results else 'no answer'
            print(f'The camera refused to turn the accelerometer on ({reason}).')
            return False
        print('Accelerometer turned on.')
        return True

    def on_imu(self, msg):
        a = msg.linear_acceleration
        self.frame = msg.header.frame_id
        now = time.time()
        self.samples.append((now, np.array([a.x, a.y, a.z])))
        self.count += 1
        while self.samples and self.samples[0][0] < now - self.args.window:
            self.samples.popleft()

    def lookup_rotation(self):
        if self.rot is not None or not self.frame:
            return
        target = f'{self.cam}_color_optical_frame'
        try:
            t = self.tf_buffer.lookup_transform(target, self.frame, rclpy.time.Time())
            q = t.transform.rotation
            self.rot = quat_to_matrix(q.x, q.y, q.z, q.w)
            self.rot_note = f'{self.frame} -> {target} from TF'
        except Exception:
            if time.time() - self.start > 5.0:
                self.rot = np.eye(3)    # the accel optical frame and the color optical frame are aligned to < 1 deg
                self.rot_note = f'no TF {self.frame} -> {target}; accelerometer axes used as they are'

    def current(self):
        if len(self.samples) < 10:
            return None
        v = np.mean([s[1] for s in self.samples], axis=0)
        spread = np.std([s[1] for s in self.samples], axis=0)
        self.lookup_rotation()
        v_cam = (self.rot if self.rot is not None else np.eye(3)) @ v
        # the accelerometer reads +g pointing up; with the camera upright, up is -y in the optical frame.
        # some driver versions report the opposite sign, which would put up at +y: flip it if so.
        self.flipped = v_cam[1] > 0
        up = -v_cam if self.flipped else v_cam
        tilt, roll = angles(up)
        return tilt, roll, v, float(np.linalg.norm(v)), float(np.linalg.norm(spread))

    def report(self):
        r = self.current()
        if r is None:
            if time.time() - self.start > 8.0 and not self.warned:
                self.warned = True
                print(f'No accelerometer data on /cameras/{self.cam}/accel/sample yet. If nothing arrives, stop the '
                      f'camera and start it on its own with the accelerometer on (see the end of this message), then '
                      f'run this again with --no-enable:\n  ros2 launch realsense2_camera rs_launch.py '
                      f'camera_name:={self.cam} camera_namespace:=cameras enable_accel:=true')
            return
        tilt, roll, _, g, shake = r
        side = 'right' if roll > 0 else 'left'
        still = 'still' if shake < 0.05 else 'MOVING - hold still'
        print(f'tilt {tilt:5.1f} deg down   lean {abs(roll):4.1f} deg ({side} side low)   [{still}]', flush=True)

    def finish(self):
        r = self.current()
        if r is None:
            print('No accelerometer data was received, nothing saved.')
            return
        tilt, roll, v, g, shake = r
        print('\nFinal (average of the last {:.1f} s, {} samples total):'.format(self.args.window, self.count))
        print(f'  tilt  {tilt:.2f} deg down   -> tripod.launch.py tilt:={tilt:.1f}')
        print(f'  roll  {roll:.2f} deg ({"right" if roll > 0 else "left"} side low)')
        print(f'  |g| {g:.2f} m/s^2 (should be close to 9.8), shake {shake:.3f}')
        print(f'  frame: {self.rot_note}{"; sign flipped" if self.flipped else ""}')
        os.makedirs(OUT, exist_ok=True)
        path = os.path.join(OUT, 'imu_tilt.yaml')
        with open(path, 'w') as f:
            f.write(f'time: "{time.strftime("%Y-%m-%d %H:%M:%S")}"\ncamera: {self.cam}\n'
                    f'tilt_deg: {tilt:.3f}\nroll_deg: {roll:.3f}\n'
                    f'accel_mean: [{v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f}]\naccel_frame: {self.frame}\n'
                    f'g_norm: {g:.4f}\nshake: {shake:.4f}\nwindow_s: {self.args.window}\n'
                    f'frame_note: "{self.rot_note}"\nsign_flipped: {str(self.flipped).lower()}\n')
        print(f'  saved to {path}')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--camera', default='d435i')
    p.add_argument('--window', type=float, default=2.0)
    p.add_argument('--no-enable', action='store_true')
    args = p.parse_args()
    rclpy.init()
    node = ImuTilt(args)
    if not args.no_enable:
        node.enable_accel()
    print('Reading the accelerometer. Press Ctrl-C when the camera is set and still.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.finish()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
