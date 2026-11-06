"""White-tape segmentation on synthetic asphalt (lane_segmentation.segment_lane_pixels, default thresholds).

Synthetic images only show whether the pipeline CAN pass; the real numbers come from labeled bags
(harness/, Phase 3). Known weaknesses are marked xfail with the reason, so they stay visible until fixed.
"""
import cv2
import numpy as np
import pytest

from conftest import INTRINSICS, LANE_HALF_WIDTH, TAPE_WIDTH, HEIGHT, WIDTH, ground_rect, render_strips
from imprimis_perception.lane_segmentation import segment_lane_pixels, shrink_to_width

MAX_SATURATION, MIN_CONTRAST = 80, 40.0
KERNEL = np.ones((3, 3), np.uint8)
# native 1280x720 pixels, as the node declares them; the synthetic images are 640 wide, scale 0.5
TOPHAT_SIZE, MIN_AREA, MAX_AREA = 102, 120.0, 60000.0


def asphalt(seed=0, gray=85, noise=8.0, size=(WIDTH, HEIGHT)):
    """Dark gray, nearly colorless, grainy."""
    rng = np.random.default_rng(seed)
    base = np.array([gray + 3, gray, gray - 3], float)            # BGR, a slight blue cast
    img = base + rng.normal(0, noise, (size[1], size[0], 1))
    return np.clip(img, 0, 255).astype(np.uint8)


def with_tape(img, strips, mount, value=235, intr=INTRINSICS):
    tape = render_strips(strips, mount, intr, size=(img.shape[1], img.shape[0]))
    out = img.copy()
    out[tape > 0] = value
    return out, tape > 0


def run(img, scale):
    mask, _ = segment_lane_pixels(img, MAX_SATURATION, MIN_CONTRAST, int(round(TOPHAT_SIZE * scale)), KERNEL,
                                  MIN_AREA * scale ** 2, MAX_AREA * scale ** 2)
    return mask


def segment(img):
    return run(img, 0.5) > 0


def test_lane_tape_on_asphalt(front_mount):
    strips = [ground_rect(0.8, 4.2, side * LANE_HALF_WIDTH, TAPE_WIDTH) for side in (1, -1)]
    img, truth = with_tape(asphalt(), strips, front_mount)
    found = segment(img)
    recall = (found & truth).sum() / truth.sum()
    # the 5x5 blur leaves a fringe up to two pixels wide along each edge; it moves the line's middle, which is all the
    # projector keeps, by nothing. Anything further out is a false detection.
    near = cv2.dilate(truth.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    precision = (found & near).sum() / max(1, found.sum())
    assert recall >= 0.85, f'recall {recall:.3f}'
    assert precision >= 0.99, f'precision {precision:.3f}'


def test_dim_tape_is_found(front_mount):
    bright = [ground_rect(0.8, 4.2, LANE_HALF_WIDTH, TAPE_WIDTH)]
    dim = [ground_rect(0.8, 4.2, -LANE_HALF_WIDTH, TAPE_WIDTH)]
    img, truth_bright = with_tape(asphalt(), bright, front_mount)
    img, truth_dim = with_tape(img, dim, front_mount, value=150)
    found = segment(img)
    assert found[truth_bright].mean() >= 0.85
    assert found[truth_dim].mean() >= 0.85


def test_wall_sized_white_area_is_not_tape():
    img = asphalt()
    img[60:300, 420:600] = (225, 226, 228)
    assert segment(img).sum() == 0


def test_bare_asphalt_is_empty():
    for seed in range(3):
        assert segment(asphalt(seed)).sum() == 0


def test_dashed_piece_far_away(front_mount):
    """A 0.6 m dash 3.4-4.0 m out. The contour-area filter must not drop it."""
    strips = [ground_rect(3.4, 4.0, LANE_HALF_WIDTH, TAPE_WIDTH)]
    img, truth = with_tape(asphalt(), strips, front_mount)
    print(f'dash area in image: {truth.sum()} px (min_area {MIN_AREA * 0.25:.0f} at this size)')
    assert segment(img)[truth].mean() >= 0.8


def test_glare_patch_is_not_tape():
    img = asphalt()
    img[300:380, 200:400] = (212, 214, 216)       # glare: bright and colorless, wider than tape
    assert segment(img).sum() == 0


def test_shrink_from_native_keeps_the_tape(front_mount):
    """The robot's path: the D435i's 1280x720 shrunk to processing_width 640, min_area scaled to match.
    The tape found must cover the same ground as segmenting at full size."""
    fx, fy, cx, cy = INTRINSICS
    native = (2 * fx, 2 * fy, 2 * cx, 2 * cy)
    strips = [ground_rect(0.8, 4.2, side * LANE_HALF_WIDTH, TAPE_WIDTH) for side in (1, -1)]
    img, _ = with_tape(asphalt(size=(2 * WIDTH, 2 * HEIGHT)), strips, front_mount, intr=native)
    small, scale = shrink_to_width(img, WIDTH)
    assert scale == pytest.approx(0.5) and small.shape[:2] == (HEIGHT, WIDTH)
    mask = run(small, scale)
    full = run(img, 1.0)
    assert (mask > 0).sum() == pytest.approx((full > 0).sum() * scale ** 2, rel=0.15)


def test_shrink_to_320_keeps_the_tape(front_mount):
    strips = [ground_rect(0.8, 4.2, LANE_HALF_WIDTH, TAPE_WIDTH)]
    img, _ = with_tape(asphalt(), strips, front_mount)
    small, scale = shrink_to_width(img, WIDTH // 2)
    mask = run(small, 0.5 * scale)
    assert (mask > 0).sum() == pytest.approx(segment(img).sum() * scale ** 2, rel=0.15)
