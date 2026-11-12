#!/usr/bin/env python3
"""Print the local costmap around the robot as text, in the robot's frame.

    python3 tools/show_costmap_ahead.py [--ahead 1.5] [--side 1.0]

  #  lethal / inscribed (>= 253)   +  high cost (>= 128)
  .  some cost (> 0)               _  free        R  the robot centre
Columns run from the robot's left (+side) to its right (-side), 10 cm each;
rows are metres ahead. Reads /local_costmap/costmap_raw (the full grid each
cycle) and /odometry/filtered. The robot does not move.
"""
import argparse
import math
import time

import numpy as np
import rclpy
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ahead', type=float, default=1.5)
    ap.add_argument('--side', type=float, default=1.0)
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node('show_costmap_ahead')
    got = {}
    n.create_subscription(Costmap, '/local_costmap/costmap_raw', lambda m: got.__setitem__('c', m), 1)
    n.create_subscription(Odometry, '/odometry/filtered', lambda m: got.__setitem__('o', m), 10)
    t = time.time()
    while len(got) < 2 and time.time() - t < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    if len(got) < 2:
        print('no costmap / odometry received')
        return 1
    m = got['c']
    md = m.metadata
    d = np.array(m.data, np.uint8).reshape(md.size_y, md.size_x)
    o = got['o'].pose.pose
    r = np.array([o.position.x, o.position.y])
    q = o.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    f = np.array([math.cos(yaw), math.sin(yaw)])
    left = np.array([-f[1], f[0]])

    def cost(p):
        i = math.floor((p[0] - md.origin.position.x) / md.resolution + 1e-9)
        j = math.floor((p[1] - md.origin.position.y) / md.resolution + 1e-9)
        return int(d[j, i]) if 0 <= i < md.size_x and 0 <= j < md.size_y else None

    print(f'# lethal  + >=128  . >0  _ free  R robot; columns +{a.side:.1f} (left) .. '
          f'-{a.side:.1f} m (right), 10 cm each')
    for ahead in np.arange(a.ahead - 0.05, -0.3, -0.1):
        row = ''
        for lat in np.arange(a.side, -a.side - 0.05, -0.1):
            if abs(ahead) < 0.06 and abs(lat) < 0.06:
                row += 'R'
                continue
            v = cost(r + ahead * f + lat * left)
            row += ' ' if v is None else ('#' if v >= 253 else ('+' if v >= 128 else ('.' if v > 0 else '_')))
        print(f'{ahead:5.2f} {row}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
