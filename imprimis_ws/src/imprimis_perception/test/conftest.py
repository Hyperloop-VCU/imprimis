"""Shared setup for the lane harness tests.

Run on the robot laptop, with ROS sourced (lane_projection imports rclpy):
    source /opt/ros/kilted/setup.bash
    python3 -m pytest imprimis_ws/src/imprimis_perception/test -v

The tests import the package from src, not install, so they check the code as it is now without a rebuild.
"""
import math
import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# optical frame (x right, y down, z forward) in a camera link frame (x forward, y left, z up)
LINK_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])

# The front D435i as the URDF has it after the 2026-11-04 measurement: front_link in base_link, 12 deg down.
# base_footprint (the ground) is at z = -0.1227 in base_link (wheel_radius 0.175, wheel_zoff 0.0523).
GROUND_Z = -0.1227
FRONT_XYZ = (0.198, 0.0175, 0.5635)
FRONT_PITCH_DEG = 12.0
# about the D435i color stream at its default 1280x720 (69 x 42 deg), shrunk by segmentation to 640 wide.
# Not yet checked against the robot's CameraInfo.
INTRINSICS = (455.0, 455.0, 320.0, 180.0)
WIDTH, HEIGHT = 640, 360
LANE_HALF_WIDTH = 1.524     # IGVC lane about 10 ft wide
TAPE_WIDTH = 0.1016         # 4 in


def optical_mount(xyz, pitch_deg=0.0, yaw_deg=0.0, roll_deg=0.0):
    """(r, t) of a camera's optical frame in base_link, for a link at xyz turned by yaw, then pitch (down), then roll."""
    r, p, y = (math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg))
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return rz @ ry @ rx @ LINK_FROM_OPTICAL, np.array(xyz, float)


def project(points_base, mount, intr=INTRINSICS):
    """Pixel (u, v) of base-frame points, and whether each is in front of the camera."""
    r, t = mount
    fx, fy, cx, cy = intr
    p = (np.asarray(points_base, float) - t) @ r
    with np.errstate(divide='ignore', invalid='ignore'):
        uv = np.stack([fx * p[:, 0] / p[:, 2] + cx, fy * p[:, 1] / p[:, 2] + cy], axis=-1)
    return uv, p[:, 2] > 0.05


def ground_rect(x0, x1, y_center, width, ground_z=GROUND_Z, along_x=True):
    """Corners of a strip of tape on the ground, base frame."""
    h = width / 2.0
    if along_x:
        corners = [(x0, y_center - h), (x1, y_center - h), (x1, y_center + h), (x0, y_center + h)]
    else:   # a strip across the robot at x = y_center, from y = x0 to x1
        corners = [(y_center - h, x0), (y_center + h, x0), (y_center + h, x1), (y_center - h, x1)]
    return np.array([[x, y, ground_z] for x, y in corners])


def render_strips(strips, mount, intr=INTRINSICS, size=(WIDTH, HEIGHT), value=255, image=None):
    """Draw ground strips into a mono8 mask (or onto image). A flat strip stays a straight-edged polygon in the image."""
    out = np.zeros((size[1], size[0]), np.uint8) if image is None else image
    for corners in strips:
        uv, ahead = project(corners, mount, intr)
        assert ahead.all(), 'strip corners must be in front of the camera'
        cv2.fillPoly(out, [np.round(uv).astype(np.int32)], value)
    return out


@pytest.fixture
def front_mount():
    return optical_mount(FRONT_XYZ, FRONT_PITCH_DEG)
