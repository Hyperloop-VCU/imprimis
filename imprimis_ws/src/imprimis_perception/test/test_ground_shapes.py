"""Top-view shape filters in surface_tools: tape survives, wide white goes, a 2 ft disc is a pothole and tape is not."""
import math

import numpy as np

from imprimis_perception import surface_tools as st


def fill(x0, x1, y0, y1, step=0.01):
    gx, gy = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.stack([gx.ravel(), gy.ravel()], axis=-1)


def disc(cx, cy, diameter, step=0.01):
    pts = fill(cx - diameter / 2, cx + diameter / 2, cy - diameter / 2, cy + diameter / 2, step)
    return pts[np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) <= diameter / 2]


def test_tape_survives_as_one_cell_line():
    cells = st.thin_structures(fill(1.0, 4.0, 1.47, 1.57), x_range=(0.0, 4.5), max_width=0.45)
    assert len(cells) > 40
    assert np.median(np.abs(cells[:, 1] - 1.52)) <= 0.03
    # skeletonized: about one 5 cm cell across, not the full tape width
    assert len(cells) < 1.5 * (3.0 / 0.05)


def test_wide_white_area_is_removed():
    assert len(st.thin_structures(fill(1.0, 2.5, -0.75, 0.75), x_range=(0.0, 4.5))) == 0


def test_two_foot_disc_is_a_pothole():
    found = st.find_white_discs(disc(2.0, 0.3, 0.61), x_range=(0.0, 3.5))
    assert len(found) == 1
    hx, hy, diameter, _ = found[0]
    assert math.hypot(hx - 2.0, hy - 0.3) < 0.05
    assert abs(diameter - 0.61) < 0.08


def test_lane_tape_is_not_a_pothole():
    assert st.find_white_discs(fill(0.5, 3.4, 1.47, 1.57), x_range=(0.0, 3.5)) == []
