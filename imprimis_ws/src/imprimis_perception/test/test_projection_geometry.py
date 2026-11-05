"""Ground projection: does a white pixel land where the tape really is?

Synthetic masks are drawn from tape at known ground positions, through a camera at a known mount, and
run back through lane_projection's geometry. Everything is exact here, so any error is the code's.
"""
import numpy as np
import pytest

from conftest import (GROUND_Z, HEIGHT, INTRINSICS, LANE_HALF_WIDTH, TAPE_WIDTH, WIDTH,
                      ground_rect, optical_mount, render_strips)
from imprimis_perception import surface_tools as st
from imprimis_perception.lane_projection import (build_ground_lookup, camera_height_and_pitch, grid_bounds,
                                                 mask_to_ground_xy, scaled_intrinsics)


def lane_cells(mask, mount, intr=INTRINSICS, max_range=4.0):
    u, v, xy = build_ground_lookup(intr, mask.shape[1], mask.shape[0], mount[0], mount[1], GROUND_Z,
                                   max_range=max_range)
    white = mask_to_ground_xy(mask, u, v, xy)
    x_range, half_width = grid_bounds(xy)
    return st.thin_structures(white, cell=0.05, x_range=x_range, half_width=half_width, max_width=0.45)


def test_mount_reads_back(front_mount):
    height, pitch = camera_height_and_pitch(front_mount[0], front_mount[1], GROUND_Z)
    assert height == pytest.approx(0.686, abs=1e-3)
    assert pitch == pytest.approx(12.0, abs=1e-6)


def test_lookup_agrees_with_raycast(front_mount):
    """Independent check: for ground points the lookup assigned to a pixel, cast that pixel's ray down to the
    ground analytically; it lands within the pixel's footprint of the point. (Near the robot one pixel covers
    less than the 1 cm grid, so not every pixel has a grid point; that is fine for 4 in tape.)"""
    r, t = front_mount
    fx, fy, cx, cy = INTRINSICS
    u, v, xy = build_ground_lookup(INTRINSICS, WIDTH, HEIGHT, r, t, GROUND_Z)
    assert len(xy) > 10000
    rng = np.random.default_rng(1)
    for i in rng.choice(len(xy), 200, replace=False):
        ray = r @ np.array([(u[i] - cx) / fx, (v[i] - cy) / fy, 1.0])
        hit = t + ray * (GROUND_Z - t[2]) / ray[2]
        reach = np.hypot(*(hit[:2] - t[:2]))
        height = t[2] - GROUND_Z
        # half a pixel's footprint along the ground: ~ reach^2 / (f * height), plus rounding slack
        footprint = 0.5 * reach ** 2 / (fy * height) + 0.5 * reach / fx
        assert np.linalg.norm(xy[i] - hit[:2]) <= footprint + 0.005, f'pixel {(u[i], v[i])} at {reach:.2f} m'


def test_nearest_visible_ground(front_mount):
    """Where the front camera starts seeing the ground. Inside this, tape is invisible to it."""
    u, v, xy = build_ground_lookup(INTRINSICS, WIDTH, HEIGHT, *front_mount, GROUND_Z, min_range=0.0)
    ahead = xy[np.abs(xy[:, 1]) < 0.05]
    nearest = ahead[:, 0].min()
    print(f'front camera sees the ground from x = {nearest:.2f} m (base_link) straight ahead')
    lane = xy[np.abs(xy[:, 1] - LANE_HALF_WIDTH) < 0.02]
    print(f'a lane line {LANE_HALF_WIDTH} m to the side is first visible at x = {lane[:, 0].min():.2f} m')
    assert 0.8 < nearest < 2.0


def test_lane_lines_land_on_the_tape(front_mount):
    strips = [ground_rect(0.8, 4.2, side * LANE_HALF_WIDTH, TAPE_WIDTH) for side in (1, -1)]
    cells = lane_cells(render_strips(strips, front_mount), front_mount)
    for side in (1, -1):
        line = cells[(np.sign(cells[:, 1]) == side) & (cells[:, 0] > 1.0) & (cells[:, 0] < 4.0)]
        assert len(line) > 20, 'too little of the line survived'
        err = np.abs(line[:, 1] - side * LANE_HALF_WIDTH)
        assert np.median(err) <= 0.03 and err.max() <= 0.06, f'side {side}: median {np.median(err):.3f} max {err.max():.3f}'
    assert np.all(np.abs(np.abs(cells[:, 1]) - LANE_HALF_WIDTH) < 0.1), 'cells off the tape'


def test_cross_line_distance(front_mount):
    """A line across the lane at a measured distance: the field calibration layout in miniature."""
    for x in (2.0, 3.0, 3.8):
        strips = [ground_rect(-0.8, 0.8, x, TAPE_WIDTH, along_x=False)]
        cells = lane_cells(render_strips(strips, front_mount), front_mount)
        assert len(cells) > 10
        assert np.median(np.abs(cells[:, 0] - x)) <= 0.03, f'cross line at {x} m'


@pytest.mark.parametrize('name, mount, line_y', [
    ('left-facing', optical_mount((0.0, 0.3, 0.6), pitch_deg=30, yaw_deg=90), 1.8),
    ('rear-facing', optical_mount((-0.8, 0.0, 0.6), pitch_deg=20, yaw_deg=180), 1.2),
    ('rolled 5 deg', optical_mount((0.2, 0.0, 0.686 + GROUND_Z), pitch_deg=12, roll_deg=5), 1.2),
])
def test_any_mount(name, mount, line_y):
    """The same code places tape correctly for cameras pointing anywhere: what more cameras need."""
    x0, x1 = (-2.5, 2.5) if 'left' in name else ((-4.2, -1.2) if 'rear' in name else (0.8, 3.5))
    strips = [ground_rect(x0, x1, line_y, TAPE_WIDTH)]
    cells = lane_cells(render_strips(strips, mount), mount)
    assert len(cells) > 10, name
    assert np.median(np.abs(cells[:, 1] - line_y)) <= 0.03, name


def test_resized_mask(front_mount):
    """Segmentation may shrink the image; the projector rescales the intrinsics to the mask."""
    k = np.array([[INTRINSICS[0], 0, INTRINSICS[2]], [0, INTRINSICS[1], INTRINSICS[3]], [0, 0, 1]])
    intr = scaled_intrinsics(k, WIDTH, HEIGHT, WIDTH // 2, HEIGHT // 2)
    strips = [ground_rect(0.8, 4.2, LANE_HALF_WIDTH, TAPE_WIDTH)]
    mask = render_strips(strips, front_mount, intr, size=(WIDTH // 2, HEIGHT // 2))
    cells = lane_cells(mask, front_mount, intr)
    assert len(cells) > 10
    # within one 5 cm output cell: the skeleton of 4 in tape can fall in either of the two cells it covers
    assert np.median(np.abs(cells[:, 1] - LANE_HALF_WIDTH)) <= 0.05


def test_wide_white_area_is_not_a_lane(front_mount):
    strips = [ground_rect(1.5, 3.0, 0.0, 1.5)]
    assert len(lane_cells(render_strips(strips, front_mount), front_mount)) == 0
