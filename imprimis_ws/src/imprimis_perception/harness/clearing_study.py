#!/usr/bin/env python3
"""P0 clearing study: how well does a cost map layer REMEMBER lane lines once the camera stops seeing them?

A real nav2_costmap_2d node is started with one lane-layer configuration (harness/configs/p0_*.yaml). This script
then drives a virtual robot down a lane at a fixed speed. At 10 Hz it publishes the robot's odom pose on TF and the
lane cloud that lane_projection would publish at that pose. The cloud is made by the real projection code:
the tape is drawn into a camera mask through build_ground_lookup (so pixel coarseness at range is the real one),
read back with mask_to_ground_xy, and thinned with surface_tools.thin_structures exactly as LaneProjection.mask_cb
does. Segmentation is assumed perfect except where a scenario adds noise on purpose. The LiDAR veto is skipped
(no obstacles). Every published cost map is recorded and scored against the true lines.

Usage (ROS sourced):
    python3 clearing_study.py all                    # every case in CASES, in parallel, then analyze
    python3 clearing_study.py run <case>             # one case (uses ROS_DOMAIN_ID from the environment)
    python3 clearing_study.py analyze                # rescore saved runs

Output goes to harness/out/p0/ (gitignored). Figures and the summary CSV are what get copied into a report.
"""
import math
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)
OUT = os.path.join(HERE, 'out', 'p0')

# ---------------------------------------------------------------------------------- robot and camera
LINK_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
GROUND_Z = -0.1227                     # base_footprint in base_link (URDF: -wheel_radius + wheel_zoff)
FRONT_XYZ = (0.198, 0.0175, 0.5635)    # front D435i link in base_link (test/conftest.py, 2026-11-04 measurement)
FRONT_PITCH_DEG = 12.0
VELODYNE_XYZ = (-0.369, 0.0, 1.103)    # from /tf_static in the nav iteration-2 bags
# lane_test_103026 CameraInfo (1280x720, fx 909.3 fy 907.0 cx 636.1 cy 354.3) at processing_width 640
INTRINSICS = (909.3 / 2, 907.0 / 2, 636.1 / 2, 354.3 / 2)
WIDTH, HEIGHT = 640, 360
LANE_HALF = 1.524                      # 10 ft lane
TAPE = 0.1016                          # 4 in
INNER_RADIUS = 1.524                   # 5 ft, the tightest reading of the IGVC 2027 turning-radius rule

# ---------------------------------------------------------------------------------- cases
# config, path, speed m/s, output_height (m above the ground, as lane_projection's parameter), noise
PATHS = {
    'straight': [('line', 12.0)],
    'left_arc': [('line', 4.0), ('arc', +(INNER_RADIUS + LANE_HALF), 120.0), ('line', 5.0)],
    'right_arc': [('line', 4.0), ('arc', -(INNER_RADIUS + LANE_HALF), 120.0), ('line', 5.0)],
    's_curve': [('line', 3.0), ('arc', +(INNER_RADIUS + LANE_HALF), 90.0), ('arc', -(INNER_RADIUS + LANE_HALF), 90.0),
                ('line', 5.0)],
}
CASES = {}
for cfg, height in (('obstacle_layer', 0.0), ('stvl', 0.62)):
    for path in PATHS:
        CASES[f'{cfg}__{path}'] = dict(config=cfg, path=path, speed=1.0, output_height=height)
    CASES[f'{cfg}__left_arc_noisy'] = dict(config=cfg, path='left_arc', speed=1.0, output_height=height,
                                         dropout=0.2, jitter=0.03)
    CASES[f'{cfg}__false_positive'] = dict(config=cfg, path='straight', speed=1.0, output_height=height,
                                         false_positive_at=3.0)
# main's lane_mark keeps points from z = 0.0 in odom; on the robot the ground is at -0.12 m in odom
CASES['stvl__straight_height0'] = dict(config='stvl', path='straight', speed=1.0, output_height=0.0)
# ablation: is the Velodyne frustum clearing what erases remembered lane voxels?
for path in ('straight', 'left_arc'):
    CASES[f'stvl_no_lidar_clear__{path}'] = dict(config='stvl_no_lidar_clear', path=path, speed=1.0,
                                                 output_height=0.62)
CASES['stvl_no_lidar_clear__false_positive'] = dict(config='stvl_no_lidar_clear', path='straight', speed=1.0,
                                                    output_height=0.62, false_positive_at=3.0)
# lane iteration 1: the local cost map exactly as the new Lane2027.yaml has it (STVL Velodyne layer + lane_layer
# + pothole_layer, 0.025 m), lane points at lane_projection's default output_height 0.0, at 0.5 and 1.0 m/s
for speed in (0.5, 1.0):
    tag = f'lane2027__v{speed:.1f}'
    for path in PATHS:
        CASES[f'{tag}__{path}'] = dict(config='lane2027', config_file='p1_lane2027_local.yaml', path=path,
                                       speed=speed, output_height=0.0)
    CASES[f'{tag}__left_arc_noisy'] = dict(config='lane2027', config_file='p1_lane2027_local.yaml',
                                           path='left_arc', speed=speed, output_height=0.0, dropout=0.2, jitter=0.03)
    CASES[f'{tag}__false_positive'] = dict(config='lane2027', config_file='p1_lane2027_local.yaml',
                                           path='straight', speed=speed, output_height=0.0, false_positive_at=3.0)


def build_path(segments, step=0.01):
    """Centerline samples (x, y, heading) in odom, starting at the origin heading +x."""
    x, y, h = 0.0, 0.0, 0.0
    pts = [(x, y, h)]
    for seg in segments:
        if seg[0] == 'line':
            for _ in range(int(round(seg[1] / step))):
                x += step * math.cos(h)
                y += step * math.sin(h)
                pts.append((x, y, h))
        else:
            radius, degrees = seg[1], seg[2]
            n = int(round(abs(radius) * math.radians(degrees) / step))
            dh = math.radians(degrees) / n * (1 if radius > 0 else -1)
            for _ in range(n):
                h += dh / 2
                x += step * math.cos(h)
                y += step * math.sin(h)
                h += dh / 2
                pts.append((x, y, h))
    return np.array(pts)


def lane_lines(center):
    """Left and right line samples: the centerline offset by half the lane width along its normal."""
    n = np.stack([-np.sin(center[:, 2]), np.cos(center[:, 2])], axis=-1)
    return center[:, :2] + LANE_HALF * n, center[:, :2] - LANE_HALF * n


def pose_at(center, s):
    i = int(min(max(s / 0.01, 0), len(center) - 1))
    return center[i]


def to_base(xy, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    d = xy - pose[:2]
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=-1)


def to_odom(xy, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.stack([pose[0] + c * xy[:, 0] - s * xy[:, 1], pose[1] + s * xy[:, 0] + c * xy[:, 1]], axis=-1)


def camera_mount():
    p = math.radians(FRONT_PITCH_DEG)
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    return ry @ LINK_FROM_OPTICAL, np.array(FRONT_XYZ, float)


# ---------------------------------------------------------------------------------- run one case
def run(name):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from scipy.spatial import cKDTree
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    from imprimis_perception import surface_tools as st
    from imprimis_perception.lane_projection import build_ground_lookup, grid_bounds, mask_to_ground_xy

    case = CASES[name]
    rng = np.random.default_rng(0)
    os.makedirs(OUT, exist_ok=True)
    params = os.path.join(HERE, 'configs', case.get('config_file', f"p0_{case['config']}.yaml"))
    log = open(os.path.join(OUT, f'{name}.costmap.log'), 'w')
    costmap = subprocess.Popen(['/opt/ros/kilted/lib/nav2_costmap_2d/nav2_costmap_2d', '--ros-args',
                                '--params-file', params], stdout=log, stderr=subprocess.STDOUT)

    center = build_path(PATHS[case['path']])
    left, right = lane_lines(center)
    truth = np.vstack([left, right])
    truth_tree = cKDTree(truth)
    s_end = (len(center) - 1) * 0.01 - 4.5          # stop while there is still lane 4 m ahead

    r_cam, t_cam = camera_mount()
    u, v, look_xy = build_ground_lookup(INTRINSICS, WIDTH, HEIGHT, r_cam, t_cam, GROUND_Z, 0.5, 4.0)
    x_range, half_width = grid_bounds(look_xy)
    fp_done = []

    def lane_cloud(pose, t):
        """What lane_projection would publish at this pose, in base_link."""
        if case.get('dropout') and rng.random() < case['dropout']:
            return np.zeros((0, 2))
        odom = to_odom(look_xy, pose)
        d, _ = truth_tree.query(odom, distance_upper_bound=0.2)
        mask = np.zeros((HEIGHT, WIDTH), np.uint8)
        on = d <= TAPE / 2
        mask[v[on], u[on]] = 255
        white = mask_to_ground_xy(mask, u, v, look_xy)
        if case.get('false_positive_at') is not None and t >= case['false_positive_at'] and not fp_done:
            fp_done.append(t)               # exactly one frame, the first at or after false_positive_at
            # inside the lane but off the robot's path (half width 0.485 m): footprint clearing must not reach it
            gx, gy = np.meshgrid(np.arange(2.4, 2.6, 0.01), np.arange(0.7, 0.9, 0.01))
            white = np.vstack([white, np.stack([gx.ravel(), gy.ravel()], -1)])
        if len(white) == 0:
            return np.zeros((0, 2))
        cells = st.thin_structures(white, cell=0.05, x_range=x_range, half_width=half_width, max_width=0.45)
        if case.get('jitter') and len(cells):
            cells = cells + np.array([0.0, rng.normal(0, case['jitter'])])
        return cells

    rclpy.init()
    node = Node('p0_driver')
    static = StaticTransformBroadcaster(node)
    tf = TransformBroadcaster(node)

    def transform(parent, child, xyz, rot=None, stamp=None):
        m = TransformStamped()
        m.header.frame_id, m.child_frame_id = parent, child
        m.header.stamp = stamp or node.get_clock().now().to_msg()
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, xyz)
        q = (0.0, 0.0, 0.0, 1.0) if rot is None else rot
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
        return m

    def matrix_to_quat(r):
        w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
        x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
        y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
        z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
        return (x, y, z, w)

    static.sendTransform([
        transform('base_link', 'd435i_color_optical_frame', t_cam, matrix_to_quat(r_cam)),
        transform('base_link', 'velodyne', VELODYNE_XYZ),
        transform('base_link', 'base_footprint', (-0.35, 0.0, GROUND_Z)),
    ])
    lane_pub = node.create_publisher(PointCloud2, '/perception/lane_points', 5)
    lidar_pub = node.create_publisher(PointCloud2, '/velodyne_points_nav', 5)
    records = []

    state = {'start': None, 'done': False}

    def robot_s(now):
        if state['start'] is None:
            return 0.0
        return min(max(0.0, (now - state['start']) * case['speed']), s_end)

    def grid_cb(msg):
        if state['start'] is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        g = np.array(msg.data, np.int16).reshape(msg.info.height, msg.info.width)
        rows, cols = np.nonzero(g >= 99)
        res = msg.info.resolution
        xy = np.stack([msg.info.origin.position.x + (cols + 0.5) * res,
                       msg.info.origin.position.y + (rows + 0.5) * res], -1)
        records.append((t - state['start'], robot_s(t), res, xy.astype(np.float32)))

    node.create_subscription(OccupancyGrid, '/costmap', grid_cb, 10)

    client = node.create_client(ChangeState, '/costmap/change_state')
    client.wait_for_service(timeout_sec=20.0)
    for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        # odom -> base_link must exist before activation
        tf.sendTransform(transform('odom', 'base_link', (0, 0, 0)))
        req = ChangeState.Request()
        req.transition.id = transition
        fut = client.call_async(req)
        end = time.time() + 20.0
        while not fut.done() and time.time() < end:
            tf.sendTransform(transform('odom', 'base_link', (0, 0, 0)))
            rclpy.spin_once(node, timeout_sec=0.05)
        if not fut.done() or not fut.result().success:
            raise RuntimeError(f'{name}: costmap lifecycle transition {transition} failed')

    ticks = {'n': 0}
    published = []

    def tick():
        now_msg = node.get_clock().now()
        now = now_msg.nanoseconds * 1e-9
        if state['start'] is None:
            state['start'] = now + 1.0
        s = robot_s(now)
        pose = pose_at(center, s)
        stamp = now_msg.to_msg()
        tf.sendTransform(transform('odom', 'base_link', (pose[0], pose[1], 0.0),
                                   (0.0, 0.0, math.sin(pose[2] / 2), math.cos(pose[2] / 2)), stamp))
        ticks['n'] += 1
        if ticks['n'] % 5:
            return                                      # 50 Hz TF, 10 Hz camera and LiDAR
        cells = lane_cloud(pose, now - state['start'])
        pts = np.zeros((len(cells), 3), np.float32)
        pts[:, :2] = cells
        pts[:, 2] = GROUND_Z + case['output_height']
        header = Header(frame_id='base_link', stamp=stamp)
        lane_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))
        published.append((now - state['start'], s, len(cells)))
        ring = np.array([[8.0 * math.cos(a), 8.0 * math.sin(a), 0.0] for a in np.linspace(0, 2 * math.pi, 64)],
                        np.float32)
        lidar_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id='velodyne', stamp=stamp), ring))
        if s >= s_end and now - state['start'] > s_end / case['speed'] + 3.0:
            state['done'] = True

    node.create_timer(0.02, tick)
    while rclpy.ok() and not state['done']:
        rclpy.spin_once(node, timeout_sec=0.05)

    costmap.terminate()
    costmap.wait(timeout=10)
    np.savez_compressed(os.path.join(OUT, f'{name}.npz'),
                        times=np.array([r[0] for r in records]), s=np.array([r[1] for r in records]),
                        res=np.array([r[2] for r in records]),
                        cells=np.array([r[3] for r in records], dtype=object),
                        published=np.array(published), case=np.array([repr(case)]))
    node.destroy_node()
    rclpy.shutdown()
    print(f'{name}: {len(records)} cost maps, {len(published)} clouds')


# ---------------------------------------------------------------------------------- scoring
def score(name):
    from scipy.spatial import cKDTree

    from imprimis_perception.lane_projection import build_ground_lookup

    data = np.load(os.path.join(OUT, f'{name}.npz'), allow_pickle=True)
    case = CASES[name]
    center = build_path(PATHS[case['path']])
    left, right = lane_lines(center)
    truth = np.vstack([left, right])[::5]          # 5 cm samples
    r_cam, t_cam = camera_mount()
    _, _, look_xy = build_ground_lookup(INTRINSICS, WIDTH, HEIGHT, r_cam, t_cam, GROUND_Z, 0.5, 4.0)
    look_tree = cKDTree(look_xy)

    seen = np.zeros(len(truth), bool)
    seen_s = np.full(len(truth), np.nan)
    rows = []
    prev_marked = None
    for t, s, res, cells in zip(data['times'], data['s'], data['res'], data['cells']):
        if t < 0:
            continue
        pose = pose_at(center, s)
        base = to_base(truth, pose)
        visible = look_tree.query(base, distance_upper_bound=0.05)[0] < 0.05
        newly = visible & ~seen
        seen_s[newly] = s
        seen |= visible
        marked = np.zeros(len(truth), bool)
        false_cells = 0
        band = np.nan
        if len(cells):
            tree = cKDTree(cells)
            tol = max(res, 0.05) + 0.03
            marked = tree.query(truth, distance_upper_bound=tol)[0] < tol
            d_line = cKDTree(truth).query(cells)[0]
            false_cells = int((d_line > 0.15).sum())
            near = d_line <= 0.3
            # the same window as the line length below, or remembered cells behind the robot inflate the width
            cells_base = to_base(cells, pose)
            in_window = (np.abs(cells_base[:, 1]) < 2.5) & (cells_base[:, 0] > -1) & (cells_base[:, 0] < 4.2)
            line_len = max(1e-6, 0.05 * (marked & (base[:, 0] > -1) & (base[:, 0] < 4.2)).sum())
            band = float((near & in_window).sum() * res * res / line_len) if line_len > 1e-3 else np.nan
        # beside the robot: from the tail to where the camera starts seeing the ground
        beside = (base[:, 0] > -0.9) & (base[:, 0] < 1.3) & (np.abs(base[:, 1]) < 2.5)
        ahead = (base[:, 0] >= 1.3) & (base[:, 0] < 4.2) & (np.abs(base[:, 1]) < 2.5)
        memory = beside & seen & ~visible
        erased = 0
        if prev_marked is not None:
            erased = int((prev_marked & ~marked & seen & ~visible & (base[:, 0] > -0.9) & (base[:, 0] < 4.2)).sum())
        prev_marked = marked
        rows.append(dict(
            t=float(t), s=float(s),
            beside_memory_recall=float(marked[memory].mean()) if memory.any() else np.nan,
            beside_never_seen=float((beside & ~seen).sum() * 0.05),
            ahead_recall=float(marked[ahead & visible].mean()) if (ahead & visible).any() else np.nan,
            erased_cells=erased, false_cells=false_cells, band_width_m=band))
    return rows, data


def analyze():
    import csv

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    summary = []
    for name in CASES:
        if not os.path.exists(os.path.join(OUT, f'{name}.npz')):
            continue
        rows, data = score(name)
        moving = [r for r in rows if r['s'] > 1.5]
        if not moving:
            continue

        def mean(key):
            vals = np.array([r[key] for r in moving], float)
            vals = vals[np.isfinite(vals)]
            return float(vals.mean()) if len(vals) else float('nan')

        def low(key, q=5):
            vals = np.array([r[key] for r in moving], float)
            vals = vals[np.isfinite(vals)]
            return float(np.percentile(vals, q)) if len(vals) else float('nan')

        fp_life = float('nan')
        if CASES[name].get('false_positive_at') is not None:
            # from the first cost map that shows the false mark to the first one that no longer does;
            # NaN if it was never marked at all (then the case tells us nothing about forgetting)
            after = [r for r in rows if r['t'] >= CASES[name]['false_positive_at']]
            shown = [i for i, r in enumerate(after) if r['false_cells'] > 0]
            if shown:
                gone = [r['t'] for r in after[shown[0]:] if r['false_cells'] == 0]
                fp_life = (gone[0] - after[shown[0]]['t']) if gone else float('inf')
        summary.append(dict(
            case=name, costmaps=len(rows),
            beside_memory_recall_mean=mean('beside_memory_recall'),
            beside_memory_recall_p5=low('beside_memory_recall'),
            ahead_recall_mean=mean('ahead_recall'),
            beside_never_seen_m_max=max(r['beside_never_seen'] for r in moving),
            erased_cells_total=int(sum(r['erased_cells'] for r in moving)),
            false_cells_mean=mean('false_cells'), band_width_m_mean=mean('band_width_m'),
            false_positive_lifetime_s=fp_life))
        with open(os.path.join(OUT, f'{name}.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
        ts = [r['t'] for r in rows]
        axes[0].plot(ts, [r['beside_memory_recall'] for r in rows], label='line beside the robot, remembered')
        axes[0].plot(ts, [r['ahead_recall'] for r in rows], label='line ahead, in view')
        axes[0].set_ylim(-0.05, 1.05)
        axes[0].set_xlabel('time (s)')
        axes[0].set_ylabel('fraction of true line marked')
        axes[0].legend(loc='lower left')
        axes[0].set_title(name)
        center = build_path(PATHS[CASES[name]['path']])
        left, right = lane_lines(center)
        last = data['cells'][-1]
        axes[1].plot(left[:, 0], left[:, 1], color='0.75', lw=1, zorder=1)
        axes[1].plot(right[:, 0], right[:, 1], color='0.75', lw=1, zorder=1)
        if len(last):
            axes[1].scatter(last[:, 0], last[:, 1], s=3, color='C3', zorder=3, label='lethal cells, last cost map')
        pose = pose_at(center, data['s'][-1])
        axes[1].plot(pose[0], pose[1], 'k^')
        axes[1].set_aspect('equal')
        axes[1].set_title('true lines (grey) and the cost map at the end')
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f'{name}.png'), dpi=110)
        plt.close(fig)
    if summary:
        with open(os.path.join(OUT, 'summary.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(summary[0]))
            w.writeheader()
            w.writerows(summary)
        for row in summary:
            print(', '.join(f'{k}={v:.3f}' if isinstance(v, float) else f'{k}={v}' for k, v in row.items()))


def run_all():
    os.makedirs(OUT, exist_ok=True)
    procs = []
    for i, name in enumerate(CASES):
        env = dict(os.environ, ROS_DOMAIN_ID=str(120 + i), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        procs.append((name, subprocess.Popen([sys.executable, __file__, 'run', name], env=env)))
    for name, p in procs:
        if p.wait() != 0:
            print(f'{name}: FAILED (see {name}.costmap.log)')
    analyze()


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if cmd == 'run':
        run(sys.argv[2])
    elif cmd == 'analyze':
        analyze()
    else:
        run_all()
