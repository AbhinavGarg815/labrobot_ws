"""Pure follow-geometry helpers (no ROS), used by person_follower.py."""

import math

import numpy as np


def yaw_quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def standoff_goal(robot_xy, person_xy, standoff):
    """(goal_xy, heading) for a target at person_xy."""
    r = np.asarray(robot_xy, float)
    p = np.asarray(person_xy, float)
    d = p - r
    dist = float(np.linalg.norm(d))
    heading = math.atan2(d[1], d[0]) if dist > 1e-6 else 0.0
    if dist <= standoff:
        return r.copy(), heading
    return p - d / dist * standoff, heading


def adaptive_speed(person_speed, dist, standoff, gain, vmin, vmax):
    return float(np.clip(person_speed + gain * (dist - standoff), vmin, vmax))


class Costmap:

    def __init__(self, msg):
        md = msg.metadata
        self.frame = msg.header.frame_id
        self.res = md.resolution
        self.ox = md.origin.position.x
        self.oy = md.origin.position.y
        self.w = md.size_x
        self.h = md.size_y
        self.data = np.asarray(msg.data, dtype=np.uint8).reshape(self.h, self.w)

    def cost(self, xy, radius=0.0):
        """Max cost within radius of xy; None outside the window."""
        # floor with a tiny epsilon: (0.40 + 1.5) / 0.05 is 37.999...
        cx = math.floor((xy[0] - self.ox) / self.res + 1e-9)
        cy = math.floor((xy[1] - self.oy) / self.res + 1e-9)
        r = int(math.ceil(radius / self.res - 1e-9))
        if not (0 <= cx < self.w and 0 <= cy < self.h):
            return None
        patch = self.data[max(cy - r, 0):cy + r + 1, max(cx - r, 0):cx + r + 1]
        return int(patch.max())


def free_goal(costmap, robot_xy, person_xy, goal_xy, heading, standoff, max_cost,
              angles_deg=(15, -15, 30, -30, 45, -45, 60, -60)):
    """Move a stand-off goal that lies in an obstacle round the person."""
    robot_xy = np.asarray(robot_xy, float)
    person_xy = np.asarray(person_xy, float)
    goal_xy = np.asarray(goal_xy, float)
    if costmap is None or np.linalg.norm(goal_xy - robot_xy) < 1e-6:
        return goal_xy, heading, ''
    c = costmap.cost(goal_xy, 0.05)
    if c is None or c < max_cost:
        return goal_xy, heading, ''
    base = math.atan2(robot_xy[1] - person_xy[1], robot_xy[0] - person_xy[0])
    for dk in angles_deg:
        a = base + math.radians(dk)
        cand = person_xy + standoff * np.array([math.cos(a), math.sin(a)])
        cc = costmap.cost(cand, 0.05)
        if cc is not None and cc < max_cost:
            return cand, math.atan2(person_xy[1] - cand[1], person_xy[0] - cand[0]), \
                f'stand-off blocked, moved {dk:+d} deg round the person'
    face = math.atan2(person_xy[1] - robot_xy[1], person_xy[0] - robot_xy[0])
    return robot_xy.copy(), face, 'stand-off blocked all round - holding, facing the person'
