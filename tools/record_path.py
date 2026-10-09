#!/usr/bin/env python3
"""Record Nav2's planned path and the robot's actual trajectory to JSON.

    python3 tools/record_path.py out.json 90

Kept in the repo rather than /tmp: a previous run lost its recording because
/tmp had been cleaned between invocations, and the loss was only noticed after
the robot had already driven the manoeuvre being measured.
"""
import json
import math
import sys
import time

import rclpy
import tf2_ros
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

out, dur = sys.argv[1], float(sys.argv[2])
rclpy.init()
n = Node('record_path')
st = {'plan': None, 'traj': [], 'odom': [], 'plans': []}
def on_plan(m):
    # Keep EVERY replan, not just the last. Nav2 replans at 1 Hz from wherever
    # the robot is, so the route it was following at the narrowest point can
    # differ from the one probed at the start - and only the history can tell
    # "the planner bent the route" from "the controller cut the corner".
    st['plan'] = m
    entry = {'t': time.time() - t0,
             'pts': [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]}
    # Same plan in the ODOM frame, using map->odom at receipt, so it can be laid
    # directly over the odom trajectory and the odom-frame gap edges.
    try:
        tf = buf.lookup_transform('odom', m.header.frame_id, rclpy.time.Time())
        q, t = tf.transform.rotation, tf.transform.translation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        c, s_ = math.cos(yaw), math.sin(yaw)
        entry['odom_pts'] = [(t.x + c*x - s_*y, t.y + s_*x + c*y) for x, y in entry['pts']]
    except Exception:  # noqa: BLE001
        pass
    st['plans'].append(entry)
n.create_subscription(Path, '/plan', on_plan,
                      QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, n)
t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
    try:
        tr = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        p = tr.transform.translation
        if not st['traj'] or math.hypot(p.x - st['traj'][-1][0],
                                        p.y - st['traj'][-1][1]) > 0.02:
            st['traj'].append((p.x, p.y))
    except Exception:  # noqa: BLE001
        pass
    # Also odom -> base_footprint, with heading. The map-frame trajectory JUMPS
    # whenever RTAB-Map relocalises (a 1.8 m jump ruined the 2026-10-07 61 cm
    # gap analysis); odom is continuous by construction, so clearance can still
    # be measured from it over a short run.
    try:
        to = buf.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        q = to.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        po = to.transform.translation
        if not st['odom'] or math.hypot(po.x - st['odom'][-1][0],
                                        po.y - st['odom'][-1][1]) > 0.02:
            st['odom'].append((po.x, po.y, yaw, time.time() - t0))
    except Exception:  # noqa: BLE001
        pass
plan = ([(ps.pose.position.x, ps.pose.position.y) for ps in st['plan'].poses]
        if st['plan'] else [])
json.dump({'plan': plan, 'traj': st['traj'], 'odom': st['odom'], 'plans': st['plans']}, open(out, 'w'))
print(f'  plan {len(plan)} poses, trajectory {len(st["traj"])} pts (map), '
      f'{len(st["odom"])} pts (odom), {len(st["plans"])} plans -> {out}')
rclpy.shutdown()
