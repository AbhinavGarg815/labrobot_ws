"""Offline tests for the follower's geometry (no ROS)."""

import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from labrobot_follow.follow_logic import (Costmap, adaptive_speed, free_goal,  # noqa: E402
                                          standoff_goal, wrap)


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
    """A nav2_msgs/Costmap-shaped object (odom), all free."""
    md = SimpleNamespace(resolution=res, size_x=w, size_y=h,
                         origin=SimpleNamespace(position=SimpleNamespace(x=ox, y=oy)))
    return SimpleNamespace(header=SimpleNamespace(frame_id='odom'), metadata=md,
                           data=np.zeros(w * h, dtype=np.uint8))


def _cell(x, y, ox=-1.5, oy=-1.5, res=0.05):
    return int(math.floor((y - oy) / res + 1e-9)), int(math.floor((x - ox) / res + 1e-9))


def test_costmap_lookup():
    msg = _grid()
    d = msg.data.reshape(60, 60)
    d[30, 40] = 254                       # cell at x = -1.5 + 40*0.05 = 0.5, y = 0
    cm = Costmap(msg)
    assert cm.cost([0.51, 0.01]) == 254
    assert cm.cost([0.0, 0.0]) == 0
    assert cm.cost([0.43, 0.0], radius=0.1) == 254     # 2 cells away
    assert cm.cost([0.40, 0.0], radius=0.05) == 0      # cell 38, 1 cell radius
    assert cm.cost([-1.5, -1.5]) == 0 and cm.cost([-1.51, 0]) is None


def test_free_goal_keeps_a_free_goal():
    cm = Costmap(_grid())
    g, h, note = free_goal(cm, [0, 0], [1.0, 0], [-0.2, 0], 0.0, 1.2, 128)
    np.testing.assert_allclose(g, [-0.2, 0])
    assert note == ''


def test_free_goal_moves_round_the_person():
    msg = _grid()
    d = msg.data.reshape(60, 60)
    # robot at (-1, 0), person at (0.9, 0): stand-off goal (-0.3, 0) is blocked
    for x in np.arange(-0.45, -0.15, 0.05):
        for y in np.arange(-0.15, 0.2, 0.05):
            d[_cell(x, y)] = 254
    cm = Costmap(msg)
    g, h, note = free_goal(cm, [-1.0, 0], [0.9, 0], [-0.3, 0], 0.0, 1.2, 128)
    assert 'moved' in note
    assert np.linalg.norm(g - [0.9, 0]) == pytest.approx(1.2)
    assert cm.cost(g, 0.05) < 128
    # the new heading faces the person
    assert h == pytest.approx(math.atan2(0 - g[1], 0.9 - g[0]))


def test_free_goal_all_blocked_holds():
    msg = _grid()
    msg.data[:] = 254
    cm = Costmap(msg)
    g, h, note = free_goal(cm, [-1.0, 0.2], [0.5, 0.2], [-0.7, 0.2], 0.0, 1.2, 128)
    np.testing.assert_allclose(g, [-1.0, 0.2])
    assert 'holding' in note
    assert h == pytest.approx(0.0)


def test_free_goal_turn_only_goal_is_never_moved():
    msg = _grid()
    msg.data[:] = 254                     # robot standing in inflation
    cm = Costmap(msg)
    g, h, note = free_goal(cm, [0.1, 0.1], [0.8, 0.1], [0.1, 0.1], 0.3, 1.2, 128)
    np.testing.assert_allclose(g, [0.1, 0.1])
    assert h == 0.3 and note == ''


def test_free_goal_outside_window_kept():
    cm = Costmap(_grid())
    g, _, note = free_goal(cm, [0, 0], [4.0, 0], [2.8, 0], 0.0, 1.2, 128)
    np.testing.assert_allclose(g, [2.8, 0])
    assert note == ''
