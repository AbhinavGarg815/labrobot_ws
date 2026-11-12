"""Robot pose in odom from /odometry/filtered, instead of a TF listener.

Why: a Python TF listener deserialises every /tf message (~87/s on this
robot, several transforms each) and wakes the rclpy executor for each - 19%
of a core measured on its own (2026-11-11), the largest single cost of the
perception node after the networks. The EKF publishes /odometry/filtered
(50 Hz, one small message) AND the odom -> base_footprint TF from the same
estimate (ekf.yaml publish_tf: true), so the pose is identical. The camera's
mount (base_footprint -> camera optical frame) is static: looked up once.
"""

import math
from collections import deque

import numpy as np


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class OdomPoseBuffer:
    """Planar robot poses (t, x, y, yaw) with interpolation at a stamp."""

    # 0.25 s, not 0.1: under load the newest odometry can lag an image by more
    # than 0.1 s; with 0.1 the tracker skipped 31 frames in 10 s and the target
    # flickered LOST (first driving test, 2026-11-12). Holding the last pose
    # for 0.25 s at 0.06-0.26 m/s is 1.5-6.5 cm.
    def __init__(self, maxlen=500, max_extrapolate_s=0.25):
        self.buf = deque(maxlen=maxlen)       # 500 at 50 Hz = 10 s
        self.max_extrapolate_s = max_extrapolate_s

    def add(self, t, x, y, yaw):
        if self.buf and t <= self.buf[-1][0]:
            return                            # out of order / duplicate
        self.buf.append((t, x, y, yaw))

    def latest(self):
        return self.buf[-1] if self.buf else None

    def at(self, t):
        """(x, y, yaw) at time t, or None if t is outside the buffer
        (beyond max_extrapolate_s past the newest pose is refused)."""
        if not self.buf:
            return None
        b = self.buf
        if t >= b[-1][0]:
            if t - b[-1][0] > self.max_extrapolate_s:
                return None
            return b[-1][1:]
        if t < b[0][0]:
            return None
        lo, hi = 0, len(b) - 1                # binary search: b[lo].t <= t < b[hi].t
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if b[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0, a0 = b[lo]
        t1, x1, y1, a1 = b[hi]
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0), a0 + f * da


def compose_camera(base_pose, R_base_cam, t_base_cam):
    """odom <- camera: (R, t) from the planar base pose and the static mount."""
    x, y, yaw = base_pose
    c, s = math.cos(yaw), math.sin(yaw)
    R_ob = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return R_ob @ R_base_cam, R_ob @ t_base_cam + np.array([x, y, 0.0])
