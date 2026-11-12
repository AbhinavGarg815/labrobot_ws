#!/usr/bin/env python3
"""Load the person-following behaviour tree in the live bt_navigator without
driving anywhere: a NavigateToPose goal with behavior_tree=follow_person.xml
at the robot's CURRENT pose (odom), held for --secs, a few /goal_update
messages at the same pose, then cancelled. Prints the robot's displacement.

    python3 tools/check_follow_bt.py [--secs 15]

The robot should not move (goal = where it is, facing where it faces).
"""
import argparse
import math
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=15.0)
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node('check_follow_bt')
    odom = {}
    n.create_subscription(Odometry, '/odometry/filtered', lambda m: odom.__setitem__('m', m), 10)
    upd = n.create_publisher(PoseStamped, '/goal_update', 10)
    nav = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    t = time.time()
    while 'm' not in odom and time.time() - t < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not nav.wait_for_server(timeout_sec=10) or 'm' not in odom:
        print('no navigate_to_pose server or no odometry')
        return 1
    p0 = odom['m'].pose.pose

    def here():
        g = PoseStamped()
        g.header.frame_id = 'odom'
        g.header.stamp = n.get_clock().now().to_msg()
        g.pose = odom['m'].pose.pose
        return g

    bt = os.path.join(get_package_share_directory('labrobot_follow'), 'behavior_trees',
                      'follow_person.xml')
    fut = nav.send_goal_async(NavigateToPose.Goal(pose=here(), behavior_tree=bt))
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    h = fut.result()
    print(f'goal accepted: {bool(h and h.accepted)}  (tree {bt})')
    if not h or not h.accepted:
        return 1
    res = h.get_result_async()
    t = time.time()
    k = 0
    while time.time() - t < a.secs and not res.done():
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - t > 2 + 2 * k:
            upd.publish(here())
            k += 1
    still_running = not res.done()
    if still_running:
        cf = h.cancel_goal_async()
        rclpy.spin_until_future_complete(n, cf, timeout_sec=5)
        rclpy.spin_until_future_complete(n, res, timeout_sec=10)
    status = res.result().status if res.done() else None
    p1 = odom['m'].pose.pose
    d = math.hypot(p1.position.x - p0.position.x, p1.position.y - p0.position.y)
    yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))  # noqa: E731
    dyaw = math.degrees(math.atan2(math.sin(yaw(p1.orientation) - yaw(p0.orientation)),
                                   math.cos(yaw(p1.orientation) - yaw(p0.orientation))))
    print(f'running for {a.secs:.0f} s until cancelled: {still_running}; final status {status} '
          f'(5 = canceled); {k} goal updates sent')
    print(f'robot moved {d * 1000:.0f} mm, turned {dyaw:+.1f} deg')
    return 0 if still_running and d < 0.05 and abs(dyaw) < 5 else 1


if __name__ == '__main__':
    raise SystemExit(main())
