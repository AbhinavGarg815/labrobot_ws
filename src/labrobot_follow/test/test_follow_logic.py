"""Offline tests for the follower's geometry (no ROS)."""

import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from labrobot_follow.follow_logic import Costmap, adaptive_speed, standoff_goal, wrap  # noqa: E402


def test_standoff_goal_on_the_line_facing_the_person():
    g, h = standoff_goal([0, 0], [3, 0], 1.2)
    np.testing.assert_allclose(g, [1.8, 0])
    assert h == pytest.approx(0.0)
    g, h = standoff_goal([1, 1], [1, 4], 1.2)
    np.testing.assert_allclose(g, [1, 2.8])
    assert h == pytest.approx(math.pi / 2)


def test_standoff_goal_never_behind_the_robot():
    g, h = standoff_goal([0, 0], [-0.5, 0.9], 1.2)     # inside the stand-off
    np.testing.assert_allclose(g, [0, 0])             # stay, just turn
    assert h == pytest.approx(math.atan2(0.9, -0.5))


def test_adaptive_speed():
    # standing person at the stand-off: floor speed
    assert adaptive_speed(0.0, 1.2, 1.2, 0.3, 0.06, 0.26) == pytest.approx(0.06)
    # walking 0.15 m/s, 0.5 m too far: 0.15 + 0.15
    assert adaptive_speed(0.15, 1.7, 1.2, 0.3, 0.06, 0.26) == pytest.approx(0.26)
    assert adaptive_speed(0.10, 1.4, 1.2, 0.3, 0.06, 0.26) == pytest.approx(0.16)
    # the test cap wins
    assert adaptive_speed(1.0, 3.0, 1.2, 0.3, 0.06, 0.06) == pytest.approx(0.06)


def test_wrap():
    assert wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)


def _grid(w=60, h=60, res=0.05, ox=-1.5, oy=-1.5):
    info = SimpleNamespace(resolution=res, width=w, height=h,
                           origin=SimpleNamespace(position=SimpleNamespace(x=ox, y=oy)))
    data = np.zeros(w * h, dtype=np.int8)
    return SimpleNamespace(header=SimpleNamespace(frame_id='odom'), info=info, data=data)


def test_costmap_lookup():
    msg = _grid()
    d = msg.data.reshape(60, 60)
    d[30, 40] = 100                       # cell at x = -1.5 + 40*0.05 = 0.5, y = 0
    cm = Costmap(msg)
    assert cm.cost([0.51, 0.01]) == 100
    assert cm.cost([0.0, 0.0]) == 0
    assert cm.cost([0.43, 0.0], radius=0.1) == 100     # 2 cells away
    assert cm.cost([0.40, 0.0], radius=0.05) == 0      # exactly cell 38, 1 cell radius
    assert cm.cost([-1.5, -1.5]) == 0 and cm.cost([-1.51, 0]) is None
    assert cm.cost([5.0, 0.0]) is None
