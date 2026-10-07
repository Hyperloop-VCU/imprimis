"""Geometry shared by the ramp detector and the lane mapper.

Nothing in this file imports ROS, so every function can be tested offline on saved sensor frames.

Frames used here:
  camera link   x forward, y left, z up (the frame Gazebo and the RealSense driver call <name>_link)
  base          base_link, on the drive axle, x forward, y left, z up
  level         base_link turned by the IMU roll and pitch so that z is true vertical. On flat ground
                it equals base. On the ramp it is what the navigation stack assumes base_link to be,
                because the odometry filter never estimates roll or pitch.
Heights "above ground" are measured from the plane the wheels stand on.
"""
import math

import numpy as np

try:
    import cv2
except ImportError:  # only the lane functions need OpenCV
    cv2 = None


# ---------------------------------------------------------------------------------- attitude
def quat_to_matrix(x, y, z, w):
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]])


def roll_pitch_of_base(imu_quat, imu_yaw_in_base=0.0):
    """Roll and pitch of base_link in radians from the IMU orientation (x, y, z, w).

    ROS convention: nose up is NEGATIVE pitch. imu_yaw_in_base is the yaw of the IMU's axes
    inside base_link; the simulated IMU is mounted turned half a circle (3.14159).
    """
    r_ws = quat_to_matrix(*imu_quat)
    c, s = math.cos(imu_yaw_in_base), math.sin(imu_yaw_in_base)
    r_bs = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    r_wb = r_ws @ r_bs.T
    pitch = -math.asin(max(-1.0, min(1.0, r_wb[2, 0])))
    roll = math.atan2(r_wb[2, 1], r_wb[2, 2])
    return roll, pitch


def level_matrix(roll, pitch):
    """Rotation that takes base coordinates to level coordinates."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return ry @ rx


# ---------------------------------------------------------------------------------- camera
def intrinsics_from_fov(width, height, horizontal_fov):
    """fx, fy, cx, cy of an ideal pinhole camera with square pixels."""
    f = (width / 2.0) / math.tan(horizontal_fov / 2.0)
    return f, f, width / 2.0, height / 2.0


def depth_to_link_points(depth, intr, rows, cols):
    """3-D points in the camera link frame for pixel arrays rows, cols of a depth image in meters."""
    fx, fy, cx, cy = intr
    d = depth[rows, cols].astype(np.float64)
    pts = np.stack([d, -(cols - cx) / fx * d, -(rows - cy) / fy * d], axis=-1)
    return pts, np.isfinite(d) & (d > 0.05)


# ---------------------------------------------------------------------------------- ramp, by depth camera
def ground_profile(points_level, ground_z, half_width=0.5, x_min=0.8, x_max=6.0, step=0.25, min_points=20):
    """Median height above the wheel plane in bins along x, for points near the robot's centerline.

    Returns (bin centers, heights). A height is NaN where the bin holds too few points.
    """
    x, y, z = points_level[:, 0], points_level[:, 1], points_level[:, 2]
    keep = (np.abs(y) <= half_width) & (x >= x_min) & (x < x_max)
    x, h = x[keep], z[keep] - ground_z
    edges = np.arange(x_min, x_max + 1e-6, step)
    centers = (edges[:-1] + edges[1:]) / 2.0
    heights = np.full(len(centers), np.nan)
    idx = np.floor((x - x_min) / step).astype(int)
    for i in range(len(centers)):
        hi = h[idx == i]
        if hi.size >= min_points:
            heights[i] = float(np.median(hi))
    return centers, heights


def find_ramp(centers, heights, min_grade=0.07, max_grade=0.30, min_length=0.75, min_rise=0.10,
              max_start_height=0.15, max_step=0.12):
    """Look for a stretch that climbs steadily, which is what a ramp looks like and a barrel does not.

    Returns None, or a dict with distance (to where the climb starts), grade, length and rise.
    """
    best = None
    start = None
    for i in range(len(centers) - 1):
        h0, h1 = heights[i], heights[i + 1]
        ok = False
        if np.isfinite(h0) and np.isfinite(h1):
            dh = h1 - h0
            grade = dh / (centers[i + 1] - centers[i])
            ok = (min_grade <= grade <= max_grade) and abs(dh) <= max_step
        if ok and start is None:
            start = i
        if start is not None and (not ok or i == len(centers) - 2):
            end = i + 1 if ok else i
            length = centers[end] - centers[start]
            rise = heights[end] - heights[start]
            if (length >= min_length and rise >= min_rise and heights[start] <= max_start_height
                    and (best is None or length > best['length'])):
                # where the fitted line meets the wheel plane
                grade = rise / length
                foot = centers[start] - heights[start] / grade
                best = {'distance': float(max(0.0, foot)), 'grade': float(grade),
                        'length': float(length), 'rise': float(rise)}
            start = None
    return best


def ramp_half_width(points_level, ground_z, x_from, x_to, min_height=0.08, max_height=0.7):
    """Half width of the raised surface between two distances, from all depth points (not only the centerline)."""
    x, y, z = points_level[:, 0], points_level[:, 1], points_level[:, 2]
    h = z - ground_z
    keep = (x >= x_from) & (x <= x_to) & (h >= min_height) & (h <= max_height)
    if keep.sum() < 30:
        return None
    return float(np.percentile(np.abs(y[keep]), 98))


# ---------------------------------------------------------------------------------- LiDAR
def classify_lidar(points_level, ground_z, min_height=0.15, max_height=2.0, surface_max_height=0.60,
                   max_surface_grade=0.35, grade_margin=0.03, min_range=0.3, max_range=30.0,
                   body_box=(-0.95, 0.30, 0.55)):
    """Split an organized LiDAR cloud into obstacles and drivable surface.

    points_level has shape (rings, columns, 3), rings sorted from the lowest beam to the highest,
    already rotated into the level frame. A return is "surface" when it is low and the returns of
    the neighboring rings in the same column lie on a gentle slope from it, as on flat ground or a
    ramp of up to about 30 percent. A barrel fails this test because neighboring rings hit it at
    the same distance and different heights.

    Returns (obstacle mask, surface mask), both of shape (rings, columns).
    """
    x, y, z = points_level[..., 0], points_level[..., 1], points_level[..., 2]
    rho = np.hypot(x, y)
    h = z - ground_z
    valid = np.isfinite(rho) & np.isfinite(z) & (rho > min_range) & (rho < max_range)
    rings, cols = rho.shape

    def steep_against_neighbor(order):
        steep = np.zeros((rings, cols), bool)
        last_rho = np.zeros(cols)
        last_h = np.zeros(cols)
        have = np.zeros(cols, bool)
        for r in order:
            v = valid[r]
            dh = np.abs(h[r] - last_h)
            dr = np.abs(rho[r] - last_rho)
            steep[r] = v & have & (dh > max_surface_grade * dr + grade_margin)
            last_rho = np.where(v, rho[r], last_rho)
            last_h = np.where(v, h[r], last_h)
            have = have | v
        return steep

    steep = steep_against_neighbor(range(rings)) | steep_against_neighbor(range(rings - 1, -1, -1))
    x0, x1, half_w = body_box
    body = (x > x0) & (x < x1) & (np.abs(y) < half_w)
    surface = valid & (h < surface_max_height) & ~steep
    obstacle = valid & (h >= min_height) & (h <= max_height) & ~surface & ~body
    return obstacle, surface


def organize_by_ring(points, ring, n_rings=16, columns=1800):
    """Arrange an unorganized cloud that has a ring number per point (the real VLP-16 driver) into (rings, columns, 3)."""
    out = np.full((n_rings, columns, 3), np.nan, np.float32)
    az = np.arctan2(points[:, 1], points[:, 0])
    col = np.clip(((az + math.pi) / (2 * math.pi) * columns).astype(int), 0, columns - 1)
    ok = (ring >= 0) & (ring < n_rings) & np.isfinite(points).all(axis=1)
    out[ring[ok], col[ok]] = points[ok]
    return out


# ---------------------------------------------------------------------------------- lane lines
def white_mask(rgb, max_saturation=60, min_value=200, blur=3):
    """Pixels that are bright and colorless: white tape. rgb is an (h, w, 3) uint8 image in RGB order."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    if blur and blur > 1:
        hsv = cv2.GaussianBlur(hsv, (blur, blur), 0)
    return cv2.inRange(hsv, (0, 0, int(min_value)), (179, int(max_saturation), 255))


def mask_to_ground_points(mask, rgb_intr, depth, depth_intr, r_base_cam, t_base_cam, r_level, ground_z,
                          height_tolerance=0.07, max_range=5.0, min_range=0.4, half_width=4.0, pixel_step=2):
    """Positions on the ground (x, y in the level frame) of the mask pixels.

    Each pixel is placed where its ray meets the wheel plane. That uses the color image alone, so
    the position does not suffer when the depth image is a frame older than the color image, which
    matters in a turn. The depth image, when there is one, is used as a check: the pixel is kept
    only if the measured depth agrees with the distance to the wheel plane. White things that
    stand up (barrel bands, a pale ramp) fail the check, because they are nearer than the ground
    behind them. height_tolerance is the height above the plane that still passes.
    """
    rows, cols = np.nonzero(mask[::pixel_step, ::pixel_step])
    if rows.size == 0:
        return np.zeros((0, 2))
    rows = rows * pixel_step
    cols = cols * pixel_step
    fx, fy, cx, cy = rgb_intr
    xn = (cols - cx) / fx  # normalized image coordinates, right and down positive
    yn = (rows - cy) / fy
    rot = r_level @ r_base_cam
    origin = r_level @ t_base_cam
    rays = np.stack([np.ones_like(xn), -xn, -yn], axis=-1) @ rot.T
    down = rays[:, 2] < -1e-3
    rays, xn, yn = rays[down], xn[down], yn[down]
    scale = (ground_z - origin[2]) / rays[:, 2]      # also the depth the camera should measure there
    pts = origin + rays * scale[:, None]
    keep = (pts[:, 0] >= min_range) & (pts[:, 0] <= max_range) & (np.abs(pts[:, 1]) <= half_width)
    if depth is not None:
        dfx, dfy, dcx, dcy = depth_intr
        dc = np.round(dcx + dfx * xn).astype(int)
        dr = np.round(dcy + dfy * yn).astype(int)
        inside = (dc >= 0) & (dc < depth.shape[1]) & (dr >= 0) & (dr < depth.shape[0])
        d = np.full(len(pts), np.nan)
        d[inside] = depth[dr[inside], dc[inside]]
        # a surface height_tolerance above the plane shortens the ray by that height over the ray's fall
        allowed = height_tolerance * scale / np.maximum(1e-3, origin[2] - ground_z) + 0.05
        with np.errstate(invalid='ignore'):
            keep &= np.isfinite(d) & (np.abs(d - scale) <= allowed)
    return pts[keep, :2]


def morphological_skeleton(image):
    """Reduce every mark in a binary image to a line about one cell wide (its middle)."""
    cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    skeleton = np.zeros_like(image)
    work = image.copy()
    for _ in range(64):
        eroded = cv2.erode(work, cross)
        opened = cv2.dilate(eroded, cross)
        skeleton = cv2.bitwise_or(skeleton, cv2.subtract(work, opened))
        work = eroded
        if cv2.countNonZero(work) == 0:
            break
    return skeleton


def thin_structures(xy, cell=0.05, x_range=(0.0, 6.0), half_width=4.0, max_width=0.45, min_cells=5, skeleton=True):
    """Keep only narrow marks, such as 4 inch tape, and drop wide white areas.

    The points are drawn on a top-view grid. Anything that survives erosion by max_width is a wide
    area; it is removed together with a border around it. Specks smaller than min_cells are removed
    too. With skeleton set, each remaining mark is reduced to its middle line, so that tape seen in
    many frames does not build up into a wide band in the cost map. Returns the centers of the
    remaining cells.
    """
    if len(xy) == 0:
        return np.zeros((0, 2))
    nx = int(round((x_range[1] - x_range[0]) / cell))
    ny = int(round(2 * half_width / cell))
    ix = ((xy[:, 0] - x_range[0]) / cell).astype(int)
    iy = ((xy[:, 1] + half_width) / cell).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    grid = np.zeros((nx, ny), np.uint8)
    grid[ix[ok], iy[ok]] = 255
    closed = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    k = max(3, int(round(max_width / cell)))
    wide = cv2.morphologyEx(closed, cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
    wide = cv2.dilate(wide, np.ones((k, k), np.uint8))
    thin = cv2.bitwise_and(closed, cv2.bitwise_not(wide))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(thin, connectivity=8)
    small = np.nonzero(stats[:, cv2.CC_STAT_AREA] < min_cells)[0]
    if small.size:
        thin[np.isin(labels, small)] = 0
    if skeleton:
        thin = morphological_skeleton(thin)
    gx, gy = np.nonzero(thin)
    return np.stack([x_range[0] + (gx + 0.5) * cell, -half_width + (gy + 0.5) * cell], axis=-1)


# ---------------------------------------------------------------------------------- potholes
def _top_view(xy, cell, x_range, half_width):
    nx = int(round((x_range[1] - x_range[0]) / cell))
    ny = int(round(2 * half_width / cell))
    ix = ((xy[:, 0] - x_range[0]) / cell).astype(int)
    iy = ((xy[:, 1] + half_width) / cell).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    grid = np.zeros((nx, ny), np.uint8)
    grid[ix[ok], iy[ok]] = 255
    return grid


def _round_blobs(grid, cell, x_range, half_width, min_diameter, max_diameter, min_fill, min_aspect):
    """Blobs in a top-view grid that are about as long as they are wide and mostly filled: (x, y, diameter, cells)."""
    found = []
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(grid, connectivity=8)
    for i in range(1, count):
        w = stats[i, cv2.CC_STAT_HEIGHT] * cell       # extent across the robot (the grid's second axis)
        h = stats[i, cv2.CC_STAT_WIDTH] * cell        # extent along the robot... the names are OpenCV's, for an image
        long_side, short_side = max(w, h), min(w, h)
        if not (min_diameter <= long_side <= max_diameter) or short_side < min_aspect * long_side:
            continue
        area = stats[i, cv2.CC_STAT_AREA] * cell * cell
        if area < min_fill * math.pi / 4.0 * w * h:
            continue
        gx, gy = np.nonzero(labels == i)
        cells = np.stack([x_range[0] + (gx + 0.5) * cell, -half_width + (gy + 0.5) * cell], axis=-1)
        found.append((float(cells[:, 0].mean()), float(cells[:, 1].mean()), float(long_side), cells))
    return found


def find_white_discs(xy, cell=0.05, x_range=(0.0, 3.5), half_width=4.0, min_diameter=0.40, max_diameter=0.85,
                     tape_width=0.30, min_fill=0.70, min_aspect=0.72, alone_margin=0.25):
    """Painted potholes: round white areas on the ground. In the IGVC a pothole is a solid white circle 2 ft across.

    xy are white pixels placed on the ground (from mask_to_ground_points, so things that stand up are
    already gone). Lane tape is removed by an opening wider than the tape; what is left and is about
    as long as it is wide, and filled, is a pothole. Returns a list of (x, y, diameter, cells), where
    cells are the centers of the grid cells the pothole covers.

    Three things keep lane tape from being taken for a pothole, which happened in a first version where
    the tape curves or the robot stands close and askew to it. The shape must be nearly as wide as it is
    long. Its size must be near that of the painted circle (0.61 m). And it must stand alone: the white
    area it belongs to, before the tape was removed, may not reach more than alone_margin beyond it. A
    piece of lane line always has more line attached. The price is that a pothole touching a lane line,
    or one only half in view, is not reported until it is seen whole.
    """
    if cv2 is None or len(xy) == 0:
        return []
    grid = _top_view(xy, cell, x_range, half_width)
    # far away the rows of pixels land more than a cell apart: close the gaps before judging the shape
    closed = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    k = max(3, int(round(tape_width / cell)))
    wide = cv2.morphologyEx(closed, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    found = []
    count, labels, stats, _ = cv2.connectedComponentsWithStats(closed, connectivity=8)
    for hx, hy, diameter, cells in _round_blobs(wide, cell, x_range, half_width, min_diameter, max_diameter, min_fill, min_aspect):
        # the white area this blob was part of before the opening
        ix = int((hx - x_range[0]) / cell)
        iy = int((hy + half_width) / cell)
        label = labels[min(max(ix, 0), labels.shape[0] - 1), min(max(iy, 0), labels.shape[1] - 1)]
        if label == 0:
            continue
        whole = max(stats[label, cv2.CC_STAT_WIDTH], stats[label, cv2.CC_STAT_HEIGHT]) * cell
        if whole > diameter + alone_margin:
            continue                    # more white is attached: a lane line, not a pothole
        # not cut off by the edge of what the camera can place on the ground
        if hx + diameter / 2 > x_range[1] - cell or hx - diameter / 2 < x_range[0] + cell:
            continue
        found.append((hx, hy, diameter, cells))
    return found


def ground_drop_points(depth, depth_intr, r_base_cam, t_base_cam, r_level, ground_z, x_range=(0.7, 3.5), half_width=2.0,
                       min_drop=0.12, pixel_step=4):
    """Negative obstacles: places where the ground is lower than the plane the wheels stand on, or is missing.

    Every sampled pixel of the depth image is a ray that points forward and down at a known angle. On flat
    ground that ray would be a known length. Where the camera measures clearly more than that, or gets no
    return at all, the ground has dropped away there. min_drop is the depth of hole, in meters, that counts.
    Returns the positions (x, y in the level frame) where the ground should have been and was not.
    Something that stands up shortens the ray instead and is never reported here.
    """
    fx, fy, cx, cy = depth_intr
    rows, cols = np.mgrid[0:depth.shape[0]:pixel_step, 0:depth.shape[1]:pixel_step]
    rows, cols = rows.ravel(), cols.ravel()
    xn = (cols - cx) / fx
    yn = (rows - cy) / fy
    rot = r_level @ r_base_cam
    origin = r_level @ t_base_cam
    rays = np.stack([np.ones_like(xn), -xn, -yn], axis=-1) @ rot.T
    down = rays[:, 2] < -1e-3
    rays, rows, cols = rays[down], rows[down], cols[down]
    expected = (ground_z - origin[2]) / rays[:, 2]          # the depth flat ground would give
    pts = origin + rays * expected[:, None]
    keep = (pts[:, 0] >= x_range[0]) & (pts[:, 0] <= x_range[1]) & (np.abs(pts[:, 1]) <= half_width)
    d = depth[rows, cols].astype(np.float64)
    # a hole of depth g makes the ray longer by g times (ray length over camera height)
    extra = min_drop * expected / max(1e-3, origin[2] - ground_z) + 0.05
    with np.errstate(invalid='ignore'):
        dropped = np.isposinf(d) | (d > expected + extra)
    return pts[keep & dropped, :2]


def cluster_drops(xy, cell=0.1, x_range=(0.0, 3.5), half_width=2.0, min_size=0.25, max_diameter=3.0):
    """Group drop points into holes. A hole must be at least min_size across both ways, so that a stray
    pixel or the thin shadow behind a barrel is not taken for one. Returns (x, y, diameter, cells)."""
    if cv2 is None or len(xy) == 0:
        return []
    grid = _top_view(xy, cell, x_range, half_width)
    closed = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    found = []
    count, labels, stats, _ = cv2.connectedComponentsWithStats(closed, connectivity=8)
    for i in range(1, count):
        w = stats[i, cv2.CC_STAT_HEIGHT] * cell
        h = stats[i, cv2.CC_STAT_WIDTH] * cell
        if min(w, h) < min_size or max(w, h) > max_diameter:
            continue
        gx, gy = np.nonzero(labels == i)
        cells = np.stack([x_range[0] + (gx + 0.5) * cell, -half_width + (gy + 0.5) * cell], axis=-1)
        found.append((float(cells[:, 0].mean()), float(cells[:, 1].mean()), float(max(w, h)), cells))
    return found


def points_outside_polygon(xy, polygon):
    """Boolean mask of the points that are NOT inside a convex or concave polygon given as an (n, 2) array."""
    if polygon is None or len(polygon) < 3 or len(xy) == 0:
        return np.ones(len(xy), bool)
    x, y = xy[:, 0], xy[:, 1]
    inside = np.zeros(len(xy), bool)
    n = len(polygon)
    for i in range(n):
        x0, y0 = polygon[i]
        x1, y1 = polygon[(i + 1) % n]
        crosses = (y0 > y) != (y1 > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            xi = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
        inside ^= crosses & (x < xi)
    return ~inside
