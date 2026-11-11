"""Tests for the odometry pose buffer that replaces the TF listener."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from labrobot_follow.odom_pose import OdomPoseBuffer, compose_camera, quat_to_rot, yaw_of  # noqa: E402


def test_interpolation_and_bounds():
    b = OdomPoseBuffer(max_extrapolate_s=0.1)
    for i in range(51):                        # 1 s at 50 Hz, driving +x at 0.2 m/s, turning
        t = i * 0.02
        b.add(t, 0.2 * t, 0.0, 0.5 * t)
    x, y, a = b.at(0.511)
    assert x == pytest.approx(0.2 * 0.511) and a == pytest.approx(0.5 * 0.511)
    assert b.at(-0.01) is None                 # before the buffer
    assert b.at(1.05) is not None              # 50 ms past the newest: hold it
    assert b.at(1.2) is None                   # too far past
    b.add(0.5, 9, 9, 9)                        # out of order: ignored
    assert b.latest()[0] == pytest.approx(1.0)


def test_yaw_interpolation_across_pi():
    b = OdomPoseBuffer()
    b.add(0.0, 0, 0, math.pi - 0.1)
    b.add(1.0, 0, 0, -math.pi + 0.1)           # +0.2 rad through +-pi
    _, _, a = b.at(0.5)
    assert math.cos(a) == pytest.approx(-1.0, abs=1e-6)


def test_compose_camera_matches_full_transform():
    # mount: camera 0.844 up, 0.02 forward, optical frame pitched 10.78 deg down
    p = math.radians(10.78)
    # base -> camera_link pitch-down, then camera_link -> optical (x right, y down, z fwd)
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    R_link_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    R_bc = Ry @ R_link_opt
    t_bc = np.array([0.02, 0.0, 0.844])
    R, t = compose_camera((1.0, 2.0, math.pi / 2), R_bc, t_bc)
    # the optical axis (z) points along the robot heading (+y in odom), tilted down
    z_axis = R @ np.array([0, 0, 1.0])
    assert z_axis[1] == pytest.approx(math.cos(p)) and z_axis[2] == pytest.approx(-math.sin(p))
    np.testing.assert_allclose(t, [1.0, 2.02, 0.844], atol=1e-12)


def test_quat_helpers():
    q = (0, 0, math.sin(0.3), math.cos(0.3))   # yaw 0.6
    assert yaw_of(*q) == pytest.approx(0.6)
    np.testing.assert_allclose(quat_to_rot(*q)[:2, :2],
                               [[math.cos(0.6), -math.sin(0.6)], [math.sin(0.6), math.cos(0.6)]])
