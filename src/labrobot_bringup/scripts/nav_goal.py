#!/usr/bin/env python3
"""Send a Nav2 goal RELATIVE to where the robot is now, and report honestly.

    ros2 run labrobot_bringup nav_goal.py --forward 1.0
    ros2 run labrobot_bringup nav_goal.py --forward 1.0 --left 0.5 --turn 90

Why relative rather than absolute map coordinates
-------------------------------------------------
The map frame's origin is wherever RTAB-Map happened to start, which is not a
place a human can point at. "1 m forward from here" is checkable against the
room; "go to (2.13, -0.87)" is not, and getting it wrong on a robot with a 0.79 m
blind zone means driving somewhere nobody verified was clear.

The goal is still SENT in the map frame - Nav2 plans there - but it is computed
from the robot's current map pose, so what you type is what you can measure.

Safety
------
Prints the resolved goal before sending, streams distance-remaining while
driving, and always cancels the goal on Ctrl-C rather than leaving Nav2 to keep
executing after the operator has walked away.
"""

import argparse
import math
import sys
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavGoal(Node):
    def __init__(self):
        super().__init__('nav_goal')
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def current_pose(self, timeout=10.0):
        """(x, y, yaw) of base_footprint in the map frame."""
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.buf.lookup_transform('map', 'base_footprint',
                                               rclpy.time.Time())
            except Exception:  # noqa: BLE001
                continue
            t = tf.transform.translation
            return t.x, t.y, yaw_of(tf.transform.rotation)
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--forward', type=float, default=0.0, help='metres ahead')
    ap.add_argument('--left', type=float, default=0.0, help='metres to the left')
    ap.add_argument('--turn', type=float, default=0.0,
                    help='degrees of heading change at the goal (+ = left)')
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--skip-localisation-check', action='store_true',
                    help='send the goal even if the pose does not match the map '
                         '(only for testing the check itself)')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = NavGoal()

    here = node.current_pose()
    if here is None:
        print('  ERROR: no map -> base_footprint transform.')
        print('  Is mapping.launch.py running with localization:=true?')
        node.destroy_node(); rclpy.shutdown(); return 1
    x0, y0, yaw0 = here

    # Refuse to navigate on a wrong pose. On 2026-10-07 a hand move left the
    # pose 1.8 m out; RTAB-Map corrected it 13 s into the run, after this goal
    # had been computed from the wrong pose, and the robot overshot by 2.5 m.
    # The goal below is RELATIVE to the current pose, so a wrong pose means a
    # wrong goal - this must be checked first, not discovered mid-run.
    if not args.skip_localisation_check:
        import nav_gate   # sibling script, installed alongside this one
        ok, why = nav_gate.ready_to_navigate(node, node.buf)
        print(f'  localisation: {why}')
        if not ok:
            print('  NOT sending the goal.')
            node.destroy_node(); rclpy.shutdown(); return 1
        here = node.current_pose()   # re-read: the check took a few seconds
        x0, y0, yaw0 = here

    # Rotate the relative offset into the map frame using the CURRENT heading.
    c, s = math.cos(yaw0), math.sin(yaw0)
    gx = x0 + args.forward * c - args.left * s
    gy = y0 + args.forward * s + args.left * c
    gyaw = yaw0 + math.radians(args.turn)

    print(f'  now  : x={x0:+.3f} y={y0:+.3f} yaw={math.degrees(yaw0):+.1f} deg')
    print(f'  goal : x={gx:+.3f} y={gy:+.3f} yaw={math.degrees(gyaw):+.1f} deg')
    print(f'  (forward {args.forward:+.2f} m, left {args.left:+.2f} m, '
          f'turn {args.turn:+.0f} deg)')

    if not node.client.wait_for_server(timeout_sec=10.0):
        print('  ERROR: navigate_to_pose action server not available.')
        print('  Is navigation.launch.py running and activated?')
        node.destroy_node(); rclpy.shutdown(); return 1

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = gx
    goal.pose.pose.position.y = gy
    qx, qy, qz, qw = yaw_to_quat(gyaw)
    goal.pose.pose.orientation.x = qx
    goal.pose.pose.orientation.y = qy
    goal.pose.pose.orientation.z = qz
    goal.pose.pose.orientation.w = qw

    state = {'remaining': None, 'feedback': 0}

    def on_feedback(fb):
        state['remaining'] = fb.feedback.distance_remaining
        state['feedback'] += 1

    print('\n  sending goal - ROBOT WILL MOVE')
    send = node.client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print('  goal REJECTED by Nav2')
        node.destroy_node(); rclpy.shutdown(); return 1

    result_future = handle.get_result_async()
    t0 = time.time()
    last_print = 0.0
    rc = 1
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if result_future.done():
                break
            now = time.time()
            if now - last_print >= 2.0:
                last_print = now
                r = state['remaining']
                print(f'    [{now-t0:5.1f}s] distance remaining '
                      + (f'{r:.2f} m' if r is not None else '(no feedback yet)'))
            if now - t0 > args.timeout:
                print('  TIMEOUT - cancelling')
                handle.cancel_goal_async()
                break
    except KeyboardInterrupt:
        # Never leave Nav2 driving after the operator has stopped watching.
        print('\n  interrupted - cancelling goal')
        handle.cancel_goal_async()
        rclpy.spin_once(node, timeout_sec=2.0)

    if result_future.done():
        status = result_future.result().status
        names = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                 GoalStatus.STATUS_ABORTED: 'ABORTED',
                 GoalStatus.STATUS_CANCELED: 'CANCELED'}
        print(f'\n  result: {names.get(status, status)}  '
              f'after {time.time()-t0:.1f} s')
        rc = 0 if status == GoalStatus.STATUS_SUCCEEDED else 1

    after = node.current_pose(timeout=5.0)
    if after:
        ax, ay, ayaw = after
        err = math.hypot(ax - gx, ay - gy)
        dyaw = math.degrees(math.atan2(math.sin(ayaw - gyaw),
                                       math.cos(ayaw - gyaw)))
        print(f'  ended : x={ax:+.3f} y={ay:+.3f} yaw={math.degrees(ayaw):+.1f} deg')
        print(f'  error : {err*1000:.0f} mm from goal, {dyaw:+.1f} deg of heading')
        print(f'  moved : {math.hypot(ax-x0, ay-y0):.3f} m '
              f'(commanded {math.hypot(args.forward, args.left):.3f} m)')

    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
