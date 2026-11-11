"""Pure follow-geometry helpers (no ROS), used by person_follower.py."""

import math

import numpy as np


def yaw_quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def standoff_goal(robot_xy, person_xy, standoff):
    """(goal_xy, heading) for a target at person_xy. Never behind the robot:
    inside the stand-off the goal is the robot's own position."""
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
    """Lookup into the local costmap (OccupancyGrid in odom, values 0-100)."""

    def __init__(self, msg):
        self.frame = msg.header.frame_id
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.w = msg.info.width
        self.h = msg.info.height
        self.data = np.asarray(msg.data, dtype=np.int16).reshape(self.h, self.w)

    def cost(self, xy, radius=0.0):
        """Max cost within radius of xy; None outside the window."""
        # floor with a tiny epsilon: (0.40 + 1.5) / 0.05 is 37.999... in floats
        cx = math.floor((xy[0] - self.ox) / self.res + 1e-9)
        cy = math.floor((xy[1] - self.oy) / self.res + 1e-9)
        r = int(math.ceil(radius / self.res))
        if not (0 <= cx < self.w and 0 <= cy < self.h):
            return None
        patch = self.data[max(cy - r, 0):cy + r + 1, max(cx - r, 0):cx + r + 1]
        return int(patch.max())
