#!/usr/bin/env python3
"""Watch RTAB-Map localisation mode until the robot recognises the saved map.

    python3 tools/watch_relocalise.py [--seconds 150]
"""
import argparse, math, sys, time
import rclpy, tf2_ros
from rclpy.node import Node
from rtabmap_msgs.msg import Info

ap = argparse.ArgumentParser(); ap.add_argument('--seconds', type=float, default=150)
a = ap.parse_args()
rclpy.init(); n = Node('watch_relocalise')
b = tf2_ros.Buffer(); tf2_ros.TransformListener(b, n)
hits = []
def pose():
    try:
        t = b.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        p, q = t.transform.translation, t.transform.rotation
        return p.x, p.y, math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    except Exception:
        return None
def stat(m, key):
    # Exact-key lookup in the statistics arrays.
    for k, v in zip(m.stats_keys, m.stats_values):
        if k == key:
            return v
    return float('nan')
def cb(m):
    if m.loop_closure_id:
        hits.append((time.time(), m.loop_closure_id,
                     stat(m, 'Loop/Highest_hypothesis_value/')))
n.create_subscription(Info, '/info', cb, 10)
t0 = time.time(); seen = 0; first = None
while time.time() - t0 < 4: rclpy.spin_once(n, timeout_sec=0.05)
first = pose()
if first: print(f'  startup guess: x={first[0]:+.2f} y={first[1]:+.2f} yaw={first[2]:+.0f}', flush=True)
while time.time() - t0 < a.seconds:
    rclpy.spin_once(n, timeout_sec=0.05)
    while seen < len(hits):
        ts, nid, h = hits[seen]; seen += 1
        rclpy.spin_once(n, timeout_sec=0.3)
        p = pose()
        print(f'  t={ts-t0:5.1f}s  MATCHED saved node {nid}  hypothesis {h:.2f}  -> '
              + (f'x={p[0]:+.2f} y={p[1]:+.2f} yaw={p[2]:+.0f}' if p else 'no pose'), flush=True)
    if len(hits) >= 3:
        time.sleep(2); break
p = pose()
print(f'  matches: {len(hits)}', flush=True)
if p and first:
    print(f'  final pose: x={p[0]:+.2f} y={p[1]:+.2f} yaw={p[2]:+.0f}   '
          f'(moved {math.hypot(p[0]-first[0], p[1]-first[1]):.2f} m from the startup guess)', flush=True)
rclpy.shutdown()
