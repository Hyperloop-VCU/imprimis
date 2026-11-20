#!/usr/bin/env python3
"""Work out the camera's real tilt and roll on the tripod from two tape lines of known length (P1).

The camera sees each line end to end. On flat ground, the length a line gets when it is projected onto the ground
depends on the camera's tilt and height (both ends move), and the difference between the two lines depends on its
roll. Holding the height fixed (the tripod's TF, or --height), the tilt and roll that make both lines come out at their
known length are the camera's real angles. Yaw does not change a length, so it is not fitted.

What it does for each bag:
  1. Reads the camera_info, the tripod TF (base_link -> front_link: height and the tilt given to tripod.launch), a
     median color image (every 30th frame, so people walking past drop out) and every /perception/lane_points cloud.
  2. Finds the two tape lines in the image: white, unsaturated blobs; the two that span the most image rows. Check the
     overlay image to see that it picked the right two. Run it on bags where only the two lines are on the ground:
     a cross strip touching a line merges with it.
  3. Fits tilt and roll so both lines come out at --line-length. Also gives the tilt for +-2 in of height error.
  4. Reports, with the launch geometry and with the fitted one: each line's length, where it starts and ends, the gap
     between the lines at both ends and the angle between them. The difference between the two is how far off the
     cost map was.
  5. Checks lane_projection: projects the tape centrelines with the launch geometry (the one lane_projection used) and
     measures how far each published lane point is from the nearest centreline. Points within 0.15 m are "on tape";
     the rest are detections of something else.

Assumes a flat ground plane, a pinhole camera (the D435i colour stream reports zero distortion), and the camera's
optical centre at front_link (the 1.5 cm offset of the colour sensor inside the D435i is ignored).

Usage (ROS sourced):
    python3 calibrate_mount.py <bag> [<bag> ...] [--line-length 2.438] [--height M] [--out DIR]
Output: <out>/<bag>_lines.jpg (which blobs were used), <out>/<bag>_top.png (top view), <out>/calibration.csv and
<out>/calibration.yaml. Default out: harness/out/calibrate_mount/.
"""
import argparse
import csv
import math
import os

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
FRAME_STEP = 30
WHITE_V, WHITE_S = 170, 45            # HSV: tape is bright and unsaturated
ON_TAPE = 0.15                        # m: lane points closer than this to a tape centreline count as on the tape
MAX_GROUND = 8.0                      # m: tape farther than this is not looked for


# ---------------------------------------------------------------------------------------------------------- bag

def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_msgs.msg import TFMessage

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/cameras/d435i/color/image_raw', '/cameras/d435i/color/camera_info',
                                                  '/tf_static', '/perception/lane_points']))
    frames, n, info, mount, clouds, cloud_frame = [], 0, None, None, [], None
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/cameras/d435i/color/image_raw':
            n += 1
            if n % FRAME_STEP == 0:
                m = deserialize_message(data, Image)
                img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                frames.append(img[..., ::-1] if m.encoding == 'rgb8' else img)
        elif topic == '/cameras/d435i/color/camera_info' and info is None:
            info = deserialize_message(data, CameraInfo)
        elif topic == '/tf_static':
            for t in deserialize_message(data, TFMessage).transforms:
                if t.header.frame_id == 'base_link' and t.child_frame_id == 'front_link':
                    q = t.transform.rotation
                    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
                    mount = (t.transform.translation.z, pitch)
        elif topic == '/perception/lane_points':
            m = deserialize_message(data, PointCloud2)
            cloud_frame = m.header.frame_id
            p = point_cloud2.read_points_numpy(m, ('x', 'y'), skip_nans=True)
            if len(p):
                clouds.append(np.asarray(p, float).reshape(-1, 2))
    if not frames or info is None or mount is None:
        raise SystemExit(f'{path}: needs color images, camera_info and the tripod TF base_link -> front_link')
    if any(abs(d) > 1e-9 for d in info.d):
        print(f'  WARNING: {path} camera_info has distortion {list(info.d)}; it is ignored here')
    K = (info.k[0], info.k[4], info.k[2], info.k[5])
    return np.median(np.stack(frames), axis=0).astype(np.uint8), K, mount, clouds, cloud_frame


# ---------------------------------------------------------------------------------------------------------- geometry

def camera_axes(tilt, roll):
    """Camera optical axes (x right, y down, z forward) as columns, in a ground frame with X forward, Y left, Z up."""
    c, s = math.cos(tilt), math.sin(tilt)
    z = np.array([c, 0.0, -s])
    x = np.array([0.0, -1.0, 0.0])
    y = np.array([-s, 0.0, -c])
    cr, sr = math.cos(roll), math.sin(roll)
    return np.c_[cr * x + sr * y, -sr * x + cr * y, z]   # roll > 0: the right side of the image is lower


def to_ground(uv, K, tilt, roll, height):
    fx, fy, cx, cy = K
    rays = np.c_[(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy, np.ones(len(uv))] @ camera_axes(tilt, roll).T
    s = -height / rays[:, 2]
    s[rays[:, 2] >= 0] = np.nan                          # at or above the horizon
    return rays[:, :2] * s[:, None]


def find_lines(img, K, launch_tilt, height):
    """The two white blobs on the ground within MAX_GROUND m that span the most image rows.

    Returns per-row (v, u_left, u_right) for each, left line first."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 2] > WHITE_V) & (hsv[..., 1] < WHITE_S)).astype(np.uint8)
    fx, fy, cx, cy = K
    # the image row where flat ground is MAX_GROUND m away, allowing the launch tilt to be 3 deg too small;
    # everything above it (sky, trees, cars, far curbs) is left out
    cut = int(cy + fy * math.tan(math.atan(height / MAX_GROUND) - launch_tilt - math.radians(3)))
    mask[:max(cut, 0)] = 0
    n, lab, st, _ = cv2.connectedComponentsWithStats(mask, 8)
    cand = [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] > 500]
    cand.sort(key=lambda i: -st[i, cv2.CC_STAT_HEIGHT])
    if len(cand) < 2:
        raise SystemExit('fewer than two white blobs found below the horizon')
    lines = []
    for i in cand[:2]:
        rows = []
        y0, h = st[i, cv2.CC_STAT_TOP], st[i, cv2.CC_STAT_HEIGHT]
        for v in range(y0, y0 + h):
            xs = np.nonzero(lab[v] == i)[0]
            if len(xs):
                rows.append((v, xs.min() - 0.5, xs.max() + 0.5))
        lines.append(np.array(rows, float))
    lines.sort(key=lambda r: np.mean(r[:, 1]))           # left line first
    return lines, lab, cand[:2]


def line_on_ground(rows, K, tilt, roll, height):
    """Centreline points, direction, length and near/far ends of one line on the ground."""
    c = to_ground(np.c_[(rows[:, 1] + rows[:, 2]) / 2, rows[:, 0]], K, tilt, roll, height)
    c = c[np.isfinite(c).all(1)]
    mean = c.mean(0)
    d = np.linalg.svd(c - mean)[2][0]
    if d[0] < 0:
        d = -d                                           # point away from the camera
    t = (c - mean) @ d
    near, far = mean + d * t.min(), mean + d * t.max()
    return {'pts': c, 'dir': d, 'length': float(t.max() - t.min()), 'near': near, 'far': far}


def fit(lines, K, height, length, start):
    def resid(p):
        return [line_on_ground(r, K, p[0], p[1], height)['length'] - length for r in lines]
    sol = least_squares(resid, start)
    return sol.x, np.abs(sol.fun).max()


def seg_distance(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0, 1)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


# ---------------------------------------------------------------------------------------------------------- main

def calibrate(bag, args):
    name = os.path.basename(os.path.normpath(bag))
    img, K, (tf_height, launch_tilt), clouds, cloud_frame = read_bag(bag)
    height = args.height if args.height else tf_height
    lines, lab, ids = find_lines(img, K, launch_tilt, height)
    (tilt, roll), worst = fit(lines, K, height, args.line_length, [launch_tilt, 0.0])
    sens = {}
    for dh in (-0.0508, 0.0508):
        sens[dh] = fit(lines, K, height + dh, args.line_length, [tilt, roll])[0][0]

    geo = {'launch': (launch_tilt, 0.0), 'fitted': (tilt, roll)}
    g = {k: [line_on_ground(r, K, t, rl, height) for r in lines] for k, (t, rl) in geo.items()}

    row = {'bag': name, 'height_m': round(height, 4), 'launch_tilt_deg': round(math.degrees(launch_tilt), 2),
           'fitted_tilt_deg': round(math.degrees(tilt), 2), 'fitted_roll_deg': round(math.degrees(roll), 2),
           'tilt_if_2in_lower_deg': round(math.degrees(sens[-0.0508]), 2),
           'tilt_if_2in_higher_deg': round(math.degrees(sens[0.0508]), 2), 'fit_residual_m': round(float(worst), 4)}
    for side, i in (('left', 0), ('right', 1)):
        a, b = g['launch'][i], g['fitted'][i]
        row[f'{side}_length_at_launch_m'] = round(a['length'], 3)
        row[f'{side}_near_x_launch_m'] = round(float(a['near'][0]), 3)
        row[f'{side}_far_x_launch_m'] = round(float(a['far'][0]), 3)
        row[f'{side}_near_x_fitted_m'] = round(float(b['near'][0]), 3)
        row[f'{side}_far_x_fitted_m'] = round(float(b['far'][0]), 3)
        row[f'{side}_near_error_m'] = round(float(np.linalg.norm(a['near'] - b['near'])), 3)
        row[f'{side}_far_error_m'] = round(float(np.linalg.norm(a['far'] - b['far'])), 3)
        row[f'{side}_foot_distance_fitted_m'] = round(float(abs(np.cross(b['dir'], -b['pts'].mean(0)))), 3)
    for k in ('launch', 'fitted'):
        L, R = g[k]
        row[f'gap_near_{k}_m'] = round(float(np.linalg.norm(L['near'] - R['near'])), 3)
        row[f'gap_far_{k}_m'] = round(float(np.linalg.norm(L['far'] - R['far'])), 3)
        row[f'angle_between_lines_{k}_deg'] = round(math.degrees(math.acos(min(1.0, abs(L['dir'] @ R['dir'])))), 2)

    # lane_projection check: published points against the tape projected with the launch geometry
    pts = np.vstack(clouds) if clouds else np.zeros((0, 2))
    if len(pts):
        d = np.minimum(*[seg_distance(pts, ln['near'], ln['far']) for ln in g['launch']])
        on = d < ON_TAPE
        row['lane_points'] = int(len(pts))
        row['lane_points_on_tape_frac'] = round(float(on.mean()), 3)
        row['lane_points_on_tape_median_offset_m'] = round(float(np.median(d[on])), 3) if on.any() else None
        row['lane_points_frame'] = cloud_frame

    os.makedirs(args.out, exist_ok=True)
    vis = img.copy()
    for i, colour in zip(ids, ((0, 0, 255), (255, 0, 0))):
        vis[lab == i] = colour
    cv2.putText(vis, f'{name}: red = left line, blue = right line', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (255, 255, 255), 2)
    cv2.imwrite(os.path.join(args.out, f'{name}_lines.jpg'), cv2.resize(vis, (vis.shape[1] // 2, vis.shape[0] // 2)))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(5, 6))
    if len(pts):
        sub = pts[:: max(1, len(pts) // 20000)]
        ax.scatter(-sub[:, 1], sub[:, 0], s=1, c='#999999', label='published lane points')
    for k, colour, lbl in (('launch', '#e67e22', f'tape, launch tilt {math.degrees(launch_tilt):.1f}'),
                           ('fitted', '#1e8449', f'tape, fitted tilt {math.degrees(tilt):.1f} roll {math.degrees(roll):+.1f}')):
        for j, ln in enumerate(g[k]):
            ax.plot([-ln['near'][1], -ln['far'][1]], [ln['near'][0], ln['far'][0]], c=colour, lw=2,
                    label=lbl if j == 0 else None)
    ax.set_xlabel('right of the camera (m)')
    ax.set_ylabel('ahead of the camera (m)')
    ax.set_title(name)
    ax.set_aspect('equal')
    ax.grid(alpha=0.4)
    ax.legend(fontsize=7, loc='upper left')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, f'{name}_top.png'), dpi=90)
    plt.close(fig)
    return row


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('bags', nargs='+')
    p.add_argument('--line-length', type=float, default=2 * 48 * 0.0254, help='m; default two 48 in strips')
    p.add_argument('--height', type=float, default=None, help='m; default the tripod TF height in the bag')
    p.add_argument('--out', default=os.path.join(HERE, 'out', 'calibrate_mount'))
    args = p.parse_args()
    rows = []
    for bag in args.bags:
        r = calibrate(bag, args)
        rows.append(r)
        if abs(r['fitted_roll_deg']) > 5:
            print(f"  WARNING: {r['bag']} roll {r['fitted_roll_deg']} deg is implausible; a line probably merged with "
                  f"another white blob (a cross strip, or the background in dim light). Check {r['bag']}_lines.jpg.")
        print(f"{r['bag']}: tilt {r['fitted_tilt_deg']:.2f} deg (launch {r['launch_tilt_deg']:.2f}), roll "
              f"{r['fitted_roll_deg']:+.2f} deg; far ends off by {r['left_far_error_m']:.2f} / {r['right_far_error_m']:.2f} m "
              f"at the launch tilt; lines {r['gap_near_fitted_m']:.2f} m apart near, {r['gap_far_fitted_m']:.2f} m far; "
              f"lane points on tape {r.get('lane_points_on_tape_frac')}")
    keys = list(rows[0])
    for r in rows[1:]:
        keys += [k for k in r if k not in keys]
    with open(os.path.join(args.out, 'calibration.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(args.out, 'calibration.yaml'), 'w') as f:
        for r in rows:
            f.write(f"{r['bag']}:\n" + ''.join(f'  {k}: {v}\n' for k, v in r.items() if k != 'bag'))
    print(f'saved to {args.out}')


if __name__ == '__main__':
    main()
