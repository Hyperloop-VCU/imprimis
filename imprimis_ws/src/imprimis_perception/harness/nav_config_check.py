#!/usr/bin/env python3
"""Load a Nav2 params file into the real Nav2 servers and check that a lane-bounded path can be planned.

1. Resolves $(find-pkg-share ...) the way nav2_bringup does, into harness/out/nav_check/params.yaml.
2. Starts planner_server, controller_server, behavior_server, velocity_smoother, smoother_server and
   bt_navigator from /opt/ros/kilted with that file, and brings them up with a lifecycle manager.
   Every server must reach "active"; every ERROR/WARN line from them is printed.
3. Publishes static TF for a robot standing at the odom origin (base_link, the D435i optical frame, the
   Velodyne, map = odom) and, at 10 Hz, the two lines of a 10 ft lane that curves left with a 1.524 m
   inner radius, as lane_projection would publish them (base_link, ground height).
4. Asks the planner (ComputePathToPose) for a path to a goal around the curve and reports how close the
   path and the robot's footprint along it come to each lane line.

Usage (ROS sourced, workspace built so imprimis_navigation's share directory exists):
    python3 nav_config_check.py ../../imprimis_navigation/config/nav2/Lane2027.yaml
"""
import math
import os
import re
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUT = os.path.join(HERE, 'out', 'nav_check')

from clearing_study import (FRONT_PITCH_DEG, FRONT_XYZ, GROUND_Z, INNER_RADIUS, LANE_HALF, VELODYNE_XYZ,  # noqa: E402
                            build_path, camera_mount, lane_lines)

SERVERS = ['planner_server', 'controller_server', 'behavior_server', 'velocity_smoother', 'smoother_server',
           'bt_navigator']
EXE = {'planner_server': 'nav2_planner', 'controller_server': 'nav2_controller', 'behavior_server': 'nav2_behaviors',
       'velocity_smoother': 'nav2_velocity_smoother', 'smoother_server': 'nav2_smoother',
       'bt_navigator': 'nav2_bt_navigator'}
FOOTPRINT_HALF_WIDTH = 0.485


def resolve(params_path):
    from ament_index_python.packages import get_package_share_directory
    text = open(params_path).read()
    text = re.sub(r'\$\(find-pkg-share ([\w_]+)\)', lambda m: get_package_share_directory(m.group(1)), text)
    os.makedirs(OUT, exist_ok=True)
    out = os.path.join(OUT, 'params.yaml')
    open(out, 'w').write(text)
    for path in re.findall(r'(?:lattice_filepath|default_nav_to_pose_bt_xml):\s*"?([^"\s#]+)', text):
        print(f'  file referenced: {path} -> {"exists" if os.path.exists(path) else "MISSING"}')
    return out


def main():
    params = resolve(os.path.abspath(sys.argv[1]))
    lm_params = os.path.join(OUT, 'lifecycle.yaml')
    open(lm_params, 'w').write('lifecycle_manager_check:\n  ros__parameters:\n    autostart: true\n'
                               '    bond_timeout: 0.0\n    node_names: [' + ', '.join(f'"{s}"' for s in SERVERS) + ']\n')
    procs = []
    logs = {}
    for s in SERVERS:
        logs[s] = os.path.join(OUT, f'{s}.log')
        procs.append(subprocess.Popen([f'/opt/ros/kilted/lib/{EXE[s]}/{s}', '--ros-args', '--params-file', params],
                                      stdout=open(logs[s], 'w'), stderr=subprocess.STDOUT))

    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init()
    node = Node('nav_config_check')
    static = StaticTransformBroadcaster(node)

    def tf(parent, child, xyz, q=(0.0, 0.0, 0.0, 1.0)):
        m = TransformStamped()
        m.header.frame_id, m.child_frame_id = parent, child
        m.header.stamp = node.get_clock().now().to_msg()
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, xyz)
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
        return m

    r_cam, t_cam = camera_mount()
    w = math.sqrt(1 + r_cam[0, 0] + r_cam[1, 1] + r_cam[2, 2]) / 2
    q_cam = ((r_cam[2, 1] - r_cam[1, 2]) / (4 * w), (r_cam[0, 2] - r_cam[2, 0]) / (4 * w),
             (r_cam[1, 0] - r_cam[0, 1]) / (4 * w), w)
    static.sendTransform([tf('map', 'odom', (0, 0, 0)), tf('odom', 'base_link', (0, 0, 0)),
                          tf('base_link', 'base_footprint', (-0.35, 0, GROUND_Z)),
                          tf('base_link', 'd435i_color_optical_frame', t_cam, q_cam),
                          tf('base_link', 'velodyne', VELODYNE_XYZ)])

    lm = subprocess.Popen(['/opt/ros/kilted/lib/nav2_lifecycle_manager/lifecycle_manager', '--ros-args',
                           '-r', '__node:=lifecycle_manager_check', '--params-file', lm_params],
                          stdout=open(os.path.join(OUT, 'lifecycle_manager.log'), 'w'), stderr=subprocess.STDOUT)
    procs.append(lm)

    # the lane: 2 m straight, then a left curve with the tightest legal inner radius
    center = build_path([('line', 2.0), ('arc', INNER_RADIUS + LANE_HALF, 120.0), ('line', 2.0)])
    center[:, 0] -= 0.5                                  # the robot stands 0.5 m into the lane
    left, right = lane_lines(center)
    lines = np.vstack([left, right])[::5]
    pts = np.zeros((len(lines), 3), np.float32)
    pts[:, :2] = lines
    pts[:, 2] = GROUND_Z
    lane_pub = node.create_publisher(PointCloud2, '/perception/lane_points', 5)
    node.create_timer(0.1, lambda: lane_pub.publish(point_cloud2.create_cloud_xyz32(
        Header(frame_id='base_link', stamp=node.get_clock().now().to_msg()), pts)))

    # wait for the lifecycle manager
    deadline = time.time() + 60
    active = False
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        text = open(os.path.join(OUT, 'lifecycle_manager.log')).read()
        if 'Managed nodes are active' in text:
            active = True
            break
        if 'Failed to bring up all requested nodes' in text or 'Failed to change state' in text:
            break
    print(f'lifecycle: {"ALL ACTIVE" if active else "NOT ACTIVE"}')
    for _ in range(30):                                   # let the cost maps take in a few lane clouds
        rclpy.spin_once(node, timeout_sec=0.1)

    result_line = 'planner: not asked'
    if active:
        client = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        client.wait_for_server(timeout_sec=10)
        goal = ComputePathToPose.Goal()
        end = center[-150]
        goal.goal = PoseStamped(header=Header(frame_id='odom'))
        goal.goal.pose.position.x, goal.goal.pose.position.y = float(end[0]), float(end[1])
        goal.goal.pose.orientation.z, goal.goal.pose.orientation.w = math.sin(end[2] / 2), math.cos(end[2] / 2)
        goal.planner_id = 'GridBased'
        goal.use_start = False
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
        res_fut = fut.result().get_result_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=15)
        res = res_fut.result()
        path = np.array([[p.pose.position.x, p.pose.position.y] for p in res.result.path.poses]) if res else np.zeros((0, 2))
        if len(path):
            from scipy.spatial import cKDTree
            d_left = cKDTree(left).query(path)[0]
            d_right = cKDTree(right).query(path)[0]
            result_line = (f'planner: path of {len(path)} poses, {np.sum(np.hypot(*np.diff(path, axis=0).T)):.2f} m long; '
                           f'closest approach of the path (base_link centre) to the left line {d_left.min():.2f} m, '
                           f'right line {d_right.min():.2f} m; footprint half width {FOOTPRINT_HALF_WIDTH} m, so the '
                           f'outline clears the lines by at least {min(d_left.min(), d_right.min()) - FOOTPRINT_HALF_WIDTH:.2f} m '
                           f'(tail swing not included)')
            np.save(os.path.join(OUT, 'path.npy'), path)
        else:
            result_line = f'planner: NO PATH (error code {getattr(res.result, "error_code", "?") if res else "timeout"})'
    print(result_line)

    for p in procs:
        p.terminate()
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()
    for s, path in logs.items():
        bad = [ln.strip() for ln in open(path) if re.search(r'\[(ERROR|WARN)\]', ln) and 'signal_handler' not in ln]
        print(f'{s}: {len(bad)} ERROR/WARN lines')
        for ln in bad[:8]:
            print('    ' + ln[:220])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
