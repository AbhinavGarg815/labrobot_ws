#!/usr/bin/env python3
"""Is the robot moving in smooth arcs, or in straight/turn/straight segments?

    ros2 run labrobot_bringup analyse_motion.py --seconds 60

Passive: run it alongside a Nav2 goal or any other motion.

What it separates
-----------------
"The motion is jerky" has two completely different causes with different fixes,
and they are indistinguishable by eye:

  * THE COMMAND is segmented. Nav2 decides to rotate in place, then translate,
    because its alignment critics reward pointing at the path more than making
    progress along it. Fix: retune the controller.
  * THE COMMAND is a smooth arc but the WHEELS do not follow it. A gentle curve
    needs a small difference between left and right wheel speeds; if that
    difference is lost to quantisation, deadband or drive lag, the robot can only
    go straight or turn sharply. Fix: the drivetrain, not Nav2.

So this reports the commanded twist and the achieved wheel speeds side by side.
A "coordinated" sample is one where the robot is asked to translate AND rotate at
the same time - that is what an arc is. A robot doing straight/turn/straight
shows almost no coordinated samples at all.
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

V_EPS = 0.01      # m/s   below this the robot is not really translating
W_EPS = 0.03      # rad/s below this it is not really rotating


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--cmd-topic',
                    default='/diff_drive_controller/cmd_vel_unstamped')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    n = Node('analyse_motion')
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)

    cmds = []    # (t, v, w)
    wheels = []  # (t, left_vel, right_vel)

    def on_cmd(m):
        cmds.append((time.time(), m.linear.x, m.angular.z))

    def on_js(m):
        try:
            li = m.name.index('left_wheel_joint')
            ri = m.name.index('right_wheel_joint')
        except ValueError:
            return
        if len(m.velocity) > max(li, ri):
            wheels.append((time.time(), m.velocity[li], m.velocity[ri]))

    n.create_subscription(Twist, args.cmd_topic, on_cmd, qos)
    n.create_subscription(JointState, '/joint_states', on_js, qos)

    ex = SingleThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    print(f'  recording {args.seconds:.0f} s - drive the robot now')
    time.sleep(args.seconds)

    print('\n' + '=' * 68)
    print('MOTION CHARACTER')
    print('=' * 68)

    if len(cmds) < 10:
        print(f'  only {len(cmds)} commands on {args.cmd_topic} - was the robot '
              'commanded at all?')
        sys.stdout.flush(); os._exit(1)

    c = np.array(cmds)
    v, w = c[:, 1], c[:, 2]
    moving = (np.abs(v) > V_EPS) | (np.abs(w) > W_EPS)
    n_move = int(moving.sum())

    straight = (np.abs(v) > V_EPS) & (np.abs(w) <= W_EPS)
    spin = (np.abs(v) <= V_EPS) & (np.abs(w) > W_EPS)
    arc = (np.abs(v) > V_EPS) & (np.abs(w) > W_EPS)

    print(f'\n  commands: {len(cmds)} samples, {n_move} of them moving')
    if n_move:
        print(f'    straight only (v, no w) : {100*straight.sum()/n_move:5.1f}%')
        print(f'    spin only (w, no v)     : {100*spin.sum()/n_move:5.1f}%')
        print(f'    COORDINATED arc (v+w)   : {100*arc.sum()/n_move:5.1f}%   '
              '<- this is smooth motion')
    print(f'  peak commanded speed      : {np.abs(v).max():.3f} m/s, '
          f'{np.abs(w).max():.3f} rad/s')

    # Command smoothness: how big are the step changes the controller emits?
    dt = np.diff(c[:, 0])
    ok = dt > 1e-3
    if ok.sum() > 5:
        dv = np.abs(np.diff(v))[ok] / dt[ok]
        dw = np.abs(np.diff(w))[ok] / dt[ok]
        print(f'  command jerk              : dv/dt max {dv.max():6.2f} m/s2, '
              f'dw/dt max {dw.max():6.2f} rad/s2')

    if len(wheels) > 10:
        wl = np.array(wheels)
        L, R = wl[:, 1], wl[:, 2]
        both = (np.abs(L) > 0.05) & (np.abs(R) > 0.05)
        same = both & (np.sign(L) == np.sign(R))
        opp = both & (np.sign(L) != np.sign(R))
        onew = ((np.abs(L) > 0.05) ^ (np.abs(R) > 0.05))
        act = (np.abs(L) > 0.05) | (np.abs(R) > 0.05)
        na = int(act.sum())
        print(f'\n  wheels: {len(wheels)} samples, {na} of them turning')
        if na:
            print(f'    both, same direction    : {100*same.sum()/na:5.1f}%   '
                  '<- driving forward, arcs included')
            print(f'    both, opposite (spin)   : {100*opp.sum()/na:5.1f}%')
            print(f'    only ONE wheel turning  : {100*onew.sum()/na:5.1f}%   '
                  '<- one motor idle while the other drives')
        if same.sum() > 5:
            # In an arc both wheels turn the same way at DIFFERENT speeds. If the
            # ratio is pinned at 1.00 the drivetrain is only ever going straight.
            ratio = np.abs(L[same]) / np.maximum(np.abs(R[same]), 1e-6)
            print(f'    L/R speed ratio while both drive: median {np.median(ratio):.3f}, '
                  f'5-95% {np.percentile(ratio,5):.3f}-{np.percentile(ratio,95):.3f}')

    print()
    if n_move and arc.sum() / n_move < 0.15:
        print('  VERDICT: motion is SEGMENTED - the robot is being commanded to')
        print('           translate and rotate separately, not to follow arcs.')
        print('           That is a CONTROLLER tuning problem, not the drivetrain.')
    elif n_move:
        print('  VERDICT: the controller IS commanding coordinated arcs.')
        print('           If the motion still looks jerky, compare the wheel')
        print('           figures above - the drivetrain may not be following.')
    print()
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
