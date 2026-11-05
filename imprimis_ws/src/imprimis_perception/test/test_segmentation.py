"""White-tape segmentation on synthetic asphalt (lane_segmentation.segment_lane_pixels, default thresholds).

Synthetic images only show whether the pipeline CAN pass; the real numbers come from labeled bags
(harness/, Phase 3). Known weaknesses are marked xfail with the reason, so they stay visible until fixed.
"""
import numpy as np
import pytest

from conftest import INTRINSICS, LANE_HALF_WIDTH, TAPE_WIDTH, HEIGHT, WIDTH, ground_rect, render_strips
from imprimis_perception.lane_segmentation import segment_lane_pixels, shrink_to_width

LOWER, UPPER = (0, 0, 200), (179, 60, 255)
KERNEL = np.ones((3, 3), np.uint8)
MIN_AREA, MAX_AREA = 400.0, 60000.0


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


def segment(img):
    mask, _ = segment_lane_pixels(img, LOWER, UPPER, KERNEL, MIN_AREA, MAX_AREA)
    return mask > 0


def test_lane_tape_on_asphalt(front_mount):
    strips = [ground_rect(0.8, 4.2, side * LANE_HALF_WIDTH, TAPE_WIDTH) for side in (1, -1)]
    img, truth = with_tape(asphalt(), strips, front_mount)
    found = segment(img)
    recall = (found & truth).sum() / truth.sum()
    precision = (found & truth).sum() / max(1, found.sum())
    # the 5x5 blur and 3x3 opening shave about a pixel off each tape edge (~13 % of the pixels here);
    # that moves the line's middle, which is all the projector keeps, by nothing
    assert recall >= 0.85, f'recall {recall:.3f}'
    assert precision >= 0.90, f'precision {precision:.3f}'


def test_bare_asphalt_is_empty():
    for seed in range(3):
        assert segment(asphalt(seed)).sum() == 0


@pytest.mark.xfail(reason='known: min_area=400 px (tuned on indoor ceiling-light speckle) drops a dashed segment '
                          '3.4-4 m out (~240 px). Proposed fix pending: lower it or make it depend on range; '
                          'thin_structures already removes specks on the ground (min_cells).', strict=True)
def test_dashed_piece_far_away(front_mount):
    """A 0.6 m dash 3.4-4.0 m out. The contour-area filter (min_area 400 px) must not drop it."""
    strips = [ground_rect(3.4, 4.0, LANE_HALF_WIDTH, TAPE_WIDTH)]
    img, truth = with_tape(asphalt(), strips, front_mount)
    print(f'dash area in image: {truth.sum()} px (min_area {MIN_AREA:.0f})')
    assert segment(img)[truth].mean() >= 0.8


@pytest.mark.xfail(reason='known: bright, colorless asphalt (sun glare, pale patches) passes a pure HSV '
                          'threshold. To be measured on real bags, Phase 3.', strict=True)
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
    mask, _ = segment_lane_pixels(small, LOWER, UPPER, KERNEL, MIN_AREA * scale ** 2, MAX_AREA * scale ** 2)
    full, _ = segment_lane_pixels(img, LOWER, UPPER, KERNEL, MIN_AREA, MAX_AREA)
    assert (mask > 0).sum() == pytest.approx((full > 0).sum() * scale ** 2, rel=0.15)


@pytest.mark.xfail(reason='known: the 5x5 blur and 3x3 opening are fixed in pixels, so shrinking below ~640 wide '
                          'erases 4 in tape past a few meters. Keep processing_width >= 640.', strict=True)
def test_shrink_to_320_keeps_the_tape(front_mount):
    strips = [ground_rect(0.8, 4.2, LANE_HALF_WIDTH, TAPE_WIDTH)]
    img, _ = with_tape(asphalt(), strips, front_mount)
    small, scale = shrink_to_width(img, WIDTH // 2)
    mask, _ = segment_lane_pixels(small, LOWER, UPPER, KERNEL, MIN_AREA * scale ** 2, MAX_AREA * scale ** 2)
    assert (mask > 0).sum() == pytest.approx(segment(img).sum() * scale ** 2, rel=0.15)
