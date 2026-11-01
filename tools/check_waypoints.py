#!/usr/bin/env python3
"""Check named waypoints against the LIVE global costmap, without driving.

    python3 tools/check_waypoints.py t1 t2 t3

Uses the same rule as `waypoints.py go` (feasible_target): a waypoint is
clear if it is free and in the robot's connected free region; otherwise the
mission would approach the nearest reachable spot within 0.5 m, or skip it.
Run with the navigation stack up (needs /global_costmap/costmap_raw and TF).
"""
import os
import sys
import time

import rclpy
from nav2_msgs.msg import Costmap
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'labrobot_bringup', 'scripts'))
import waypoints as W  # noqa: E402


def main():
    names = sys.argv[1:]
    if not names:
        print(__doc__); return 1
    rclpy.init(); tool = W.Tool(); cm = {}
    tool.create_subscription(Costmap, '/global_costmap/costmap_raw', lambda m: cm.__setitem__('c', m),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    t0 = time.time()
    while 'c' not in cm and time.time() - t0 < 10:
        rclpy.spin_once(tool, timeout_sec=0.1)
    here = tool.robot_pose()
    if 'c' not in cm or here is None:
        print('  no global costmap or no map pose - is navigation running?'); return 1
    wps = W.load()
    print(f'  robot at ({here[0]:+.2f},{here[1]:+.2f})')
    for n in names:
        if n not in wps:
            print(f'  {n}: unknown waypoint'); continue
        w = wps[n]
        r = W.feasible_target(cm['c'], here[:2], w['x'], w['y'])
        verdict = ('BLOCKED - nothing reachable within 0.5 m' if r is None else
                   'clear, reachable' if r[2] == 0 else
                   f'blocked - would approach {r[2]*100:.0f} cm away at ({r[0]:+.2f},{r[1]:+.2f})')
        print(f'  {n}: ({w["x"]:+.2f},{w["y"]:+.2f}, {w["yaw_deg"]:+.0f} deg) -> {verdict}')
    tool.destroy_node(); rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
