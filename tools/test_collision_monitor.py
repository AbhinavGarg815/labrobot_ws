#!/usr/bin/env python3
"""Standalone test of the collision monitor config. The robot does NOT move.

Starts nav2_collision_monitor with config/collision_monitor.yaml but every
topic redirected to /test/... (input command, output command, cloud), feeds
it a forward command and synthetic point clouds in base_footprint, and
checks the output:

  1. empty cloud                      -> command passes unchanged
  2. 50 points in the stop zone       -> zero
  3. 50 points in the slowdown zone   -> halved
  4. 5 points in the stop zone        -> unchanged (noise tolerance)
  5. points at 0.15 m height but beside the robot -> unchanged
  6. cloud stops (source timeout)     -> unchanged (fails OPEN - documented)

    python3 tools/test_collision_monitor.py
"""

import os
import signal
import struct
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

CONFIG = os.path.expanduser('~/ros2_ws/src/labrobot_bringup/config/collision_monitor.yaml')


def cloud(node, pts):
    m = PointCloud2()
    m.header.frame_id = 'base_footprint'
    m.header.stamp = node.get_clock().now().to_msg()
    m.height, m.width = 1, len(pts)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate('xyz')]
    m.point_step, m.row_step = 12, 12 * len(pts)
    m.is_dense = True
    m.data = b''.join(struct.pack('fff', *p) for p in pts)
    return m


def block(x0, x1, y0, y1, z, n):
    import random
    random.seed(1)
    return [(random.uniform(x0, x1), random.uniform(y0, y1), z) for _ in range(n)]


def main():
    cmd = ['ros2', 'run', 'nav2_collision_monitor', 'collision_monitor', '--ros-args',
           '-r', '__node:=collision_monitor_test', '--params-file', CONFIG,
           '-p', 'cmd_vel_in_topic:=/test/cm_in', '-p', 'cmd_vel_out_topic:=/test/cm_out',
           '-p', 'depth_cloud.topic:=/test/cm_cloud',
           '-p', 'PolygonStop.polygon_pub_topic:=/test/cm_polygon_stop',
           '-p', 'PolygonSlow.polygon_pub_topic:=/test/cm_polygon_slow']
    # the params file is keyed by node name `collision_monitor`
    cmd[cmd.index('--params-file') + 1] = rewrite_params()
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                            start_new_session=True)
    try:
        time.sleep(3.0)
        for t in ('configure', 'activate'):
            r = subprocess.run(['ros2', 'lifecycle', 'set', '/collision_monitor_test', t],
                               capture_output=True, text=True, timeout=20)
            if 'successful' not in r.stdout:
                print(f'lifecycle {t} failed: {r.stdout} {r.stderr}')
                return 1
        return run_cases()
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)


def rewrite_params():
    """Copy of the config keyed by the test node's name."""
    out = '/tmp/collision_monitor_test.yaml'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(CONFIG) as f:
        txt = f.read()
    with open(out, 'w') as f:
        f.write(txt.replace('collision_monitor:\n', 'collision_monitor_test:\n', 1))
    return out


def run_cases():
    rclpy.init()
    n = Node('collision_monitor_tester')
    pub_cmd = n.create_publisher(Twist, '/test/cm_in', 10)
    pub_cloud = n.create_publisher(PointCloud2, '/test/cm_cloud', 10)
    out = []
    n.create_subscription(Twist, '/test/cm_out', lambda m: out.append(m.linear.x), 10)

    def run(pts, secs=1.5, send_cloud=True):
        out.clear()
        end = time.monotonic() + secs
        while time.monotonic() < end:
            if send_cloud:
                pub_cloud.publish(cloud(n, pts))
            pub_cmd.publish(Twist(linear=Twist().linear.__class__(x=0.10)))
            rclpy.spin_once(n, timeout_sec=0.1)
        return out[-5:]

    cases = [
        ('empty cloud passes', [], lambda v: all(abs(x - 0.10) < 1e-6 for x in v)),
        ('50 pts in stop zone -> 0', block(0.25, 0.33, -0.2, 0.2, 0.6, 50),
         lambda v: all(abs(x) < 1e-6 for x in v)),
        ('50 pts in slow zone -> half', block(0.45, 0.58, -0.25, 0.25, 0.4, 50),
         lambda v: all(abs(x - 0.05) < 1e-3 for x in v)),
        ('5 pts in stop zone (noise) pass', block(0.25, 0.33, -0.2, 0.2, 0.6, 5),
         lambda v: all(abs(x - 0.10) < 1e-6 for x in v)),
        ('pts beside the robot pass', block(-0.1, 0.1, 0.40, 0.6, 0.15, 50),
         lambda v: all(abs(x - 0.10) < 1e-6 for x in v)),
        ('below min_height ignored', block(0.25, 0.33, -0.2, 0.2, 0.05, 50),
         lambda v: all(abs(x - 0.10) < 1e-6 for x in v)),
    ]
    ok = True
    run([], 2.0)                       # warm up
    for name, pts, check in cases:
        v = run(pts)
        good = bool(v) and check(v)
        ok &= good
        print(f'{"PASS" if good else "FAIL"}  {name:36s} out={[round(x, 3) for x in v]}')
    run(block(0.25, 0.33, -0.2, 0.2, 0.6, 50), 1.0)
    v = run([], 2.5, send_cloud=False)
    print(f'INFO  cloud stopped (source timeout)       out={[round(x, 3) for x in v]} '
          '(fails open: expected 0.1)')
    n.destroy_node()
    rclpy.shutdown()
    print('ALL PASS' if ok else 'FAILURES')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
