#!/usr/bin/env python3
"""Dry-run test of person_follower's state machine with a scripted virtual person.

    python3 tools/test_follower_dry.py
"""

import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class Fake(Node):
    def __init__(self):
        super().__init__('fake_person_tracker')
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.pub_state = self.create_publisher(String, '/labrobot/target_state', 10)
        self.pub_target = self.create_publisher(Odometry, '/labrobot/follow_target', 10)
        self.pub_cmd = self.create_publisher(String, '/labrobot/follow_cmd', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status = []
        self.goals = []
        self.forbidden = []
        self.plans = 0
        self.create_subscription(String, '/labrobot/follow_status',
                                 lambda m: self.status.append((time.monotonic(), m.data)), latched)
        self.create_subscription(PoseStamped, '/labrobot/follow_goal',
                                 lambda m: self.goals.append((time.monotonic(), m)), 10)
        self.create_subscription(PoseStamped, '/goal_update',
                                 lambda m: self.forbidden.append('goal_update'), 10)
        self.create_subscription(SpeedLimit, '/speed_limit',
                                 lambda m: self.forbidden.append('speed_limit'), 10)
        self.create_subscription(Path, '/labrobot/follow_plan', self.on_plan, 1)

    def on_plan(self, m):
        self.plans += 1

    def robot(self):
        tf = self.buf.lookup_transform('odom', 'base_footprint', Time())
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return np.array([t.x, t.y]), yaw

    def send_person(self, xy, v, seen, track_id=7):
        now = self.get_clock().now()
        st = {'state': 'tracking' if seen else 'searching', 'event': 'tracking' if seen else 'searching',
              'track_id': track_id if seen else None, 'gallery': 10, 'seen_age_s': 0.0 if seen else 5.0,
              'speed': float(np.hypot(*v)) if seen else 0.0}
        self.pub_state.publish(String(data=json.dumps(st)))
        if seen:
            o = Odometry()
            o.header.frame_id = 'odom'
            o.header.stamp = now.to_msg()
            o.pose.pose.position.x, o.pose.pose.position.y = float(xy[0]), float(xy[1])
            o.pose.pose.orientation.w = 1.0
            o.twist.twist.linear.x, o.twist.twist.linear.y = float(v[0]), float(v[1])
            self.pub_target.publish(o)


def last_status(n, since):
    s = [x for t, x in n.status if t >= since]
    return s[-1] if s else ''


def main():
    rclpy.init()
    n = Fake()
    end = time.monotonic() + 5
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            r0, yaw0 = n.robot()
            break
        except Exception:
            continue
    else:
        print('no odom -> base_footprint TF: is the stack up?')
        return 1
    fwd = np.array([math.cos(yaw0), math.sin(yaw0)])
    left = np.array([-math.sin(yaw0), math.cos(yaw0)])

    def rel(ahead, lft):
        return r0 + ahead * fwd + lft * left

    proc = subprocess.Popen(
        ['ros2', 'run', 'labrobot_follow', 'person_follower.py', '--ros-args',
         '-p', 'dry_run:=true', '-p', 'search_wait_s:=4.0', '-p', 'lost_hold_s:=1.0',
         '-p', 'goal_free_cost:=256'],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(f'{"PASS" if ok else "FAIL"}  {name}  {detail}')

    try:
        t_end = time.monotonic() + 6
        while time.monotonic() < t_end and not n.status:
            rclpy.spin_once(n, timeout_sec=0.1)
        check('follower up, IDLE, says DRY RUN', bool(n.status) and 'DRY RUN' in n.status[-1][1],
              n.status[-1][1] if n.status else 'no status')

        def run(secs, pos_fn, seen=True, vel=(0, 0)):
            t0 = time.monotonic()
            while time.monotonic() - t0 < secs:
                k = time.monotonic() - t0
                n.send_person(pos_fn(k), vel, seen)
                rclpy.spin_once(n, timeout_sec=0.05)
                time.sleep(0.05)

        run(2.0, lambda k: rel(2.5, 0))     # let its TF buffer fill
        n.pub_cmd.publish(String(data='start'))
        t_start = time.monotonic()
        run(3.0, lambda k: rel(2.5, 0), vel=(1.0, 0.0))
        s = last_status(n, t_start)
        check('start while the person moves -> ACQUIRE, stand still', 'stand still' in s, s)
        t_still = time.monotonic()
        run(3.0, lambda k: rel(2.5, 0))
        s = last_status(n, t_still)
        check('then standing still -> FOLLOW', s.startswith('FOLLOW'), s)
        g = n.goals[-1][1]
        gxy = np.array([g.pose.position.x, g.pose.position.y])
        gh = 2 * math.atan2(g.pose.orientation.z, g.pose.orientation.w)
        check('goal 1.2 m short of the person, on the line', np.linalg.norm(gxy - rel(1.3, 0)) < 0.03,
              f'goal {np.round(gxy - r0, 3)} expected {np.round(rel(1.3, 0) - r0, 3)}')
        check('goal faces the person', abs(math.atan2(math.sin(gh - yaw0), math.cos(gh - yaw0))) < 0.02,
              f'{math.degrees(gh - yaw0):.1f} deg')
        print(f'INFO  planner asked: {n.plans} plans received; status: {s}')

        a, b = rel(2.5, 0), rel(3.0, 1.0)
        run(5.0, lambda k: a + (b - a) * min(k / 5.0, 1.0), vel=tuple((b - a) / 5.0))
        g = n.goals[-1][1]
        gxy = np.array([g.pose.position.x, g.pose.position.y])
        gh = 2 * math.atan2(g.pose.orientation.z, g.pose.orientation.w)
        expect_h = math.atan2(*(b - r0)[::-1])
        exp = b - (b - r0) / np.linalg.norm(b - r0) * 1.2
        check('goal followed the walk', np.linalg.norm(gxy - exp) < 0.2,
              f'goal {np.round(gxy - r0, 2)} expected {np.round(exp - r0, 2)}')
        check('heading turned towards the person', abs(gh - expect_h) < math.radians(12),
              f'{math.degrees(gh - yaw0):.1f} vs {math.degrees(expect_h - yaw0):.1f} deg')
        s = last_status(n, time.monotonic() - 1)
        check('status reports walking speed', 'walking 0.2' in s, s)

        run(3.0, lambda k: rel(1.0, 0))
        g = n.goals[-1][1]
        gxy = np.array([g.pose.position.x, g.pose.position.y])
        check('inside stand-off: goal = robot (turn only)', np.linalg.norm(gxy - r0) < 0.03,
              f'{np.round(gxy - r0, 3)}')
        t_close = time.monotonic()
        run(3.0, lambda k: rel(0.6, 0))
        s = last_status(n, t_close)
        check('0.6 m: holding - too close', 'too close' in s, s)

        # lost while close (camera too close to see): wait, never spin
        t_close_gone = time.monotonic()
        while time.monotonic() - t_close_gone < 5.0:
            n.send_person(None, (0, 0), False)
            rclpy.spin_once(n, timeout_sec=0.05)
            time.sleep(0.05)
        s = last_status(n, t_close_gone)
        seen_close = {x.split(':')[0] for t, x in n.status if t >= t_close_gone}
        check('lost at 0.6 m -> WAIT too close, no SEARCH', 'too close to see' in s
              and 'SEARCH' not in seen_close, f'{s} {sorted(seen_close)}')
        run(2.0, lambda k: rel(2.5, 0))
        t_gone = time.monotonic()
        seen_states = set()
        while time.monotonic() - t_gone < 60:
            n.send_person(None, (0, 0), False)
            rclpy.spin_once(n, timeout_sec=0.05)
            time.sleep(0.05)
            s = last_status(n, t_gone)
            seen_states.add(s.split(':')[0])
            if s.startswith('WAIT'):
                break
        check('gone -> LOST -> SEARCH -> WAIT', {'LOST', 'SEARCH', 'WAIT'} <= seen_states,
              f'{sorted(seen_states)}')
        t_back = time.monotonic()
        run(2.0, lambda k: rel(2.5, 0))
        s = last_status(n, t_back)
        check('re-seen in WAIT -> FOLLOW', s.startswith('FOLLOW'), s)
        t_gone = time.monotonic()
        while time.monotonic() - t_gone < 90:
            n.send_person(None, (0, 0), False)
            rclpy.spin_once(n, timeout_sec=0.05)
            time.sleep(0.05)
            if last_status(n, t_gone).startswith('IDLE'):
                break
        s = last_status(n, t_gone)
        check('gone for good -> IDLE target lost', s.startswith('IDLE') and 'lost' in s,
              f'{s} after {time.monotonic() - t_gone:.0f} s')
        check('nothing sent to /goal_update or /speed_limit (dry run)', not n.forbidden,
              f'{set(n.forbidden)}')
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        n.destroy_node()
        rclpy.shutdown()
    ok = all(results)
    print('ALL PASS' if ok else 'FAILURES')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
