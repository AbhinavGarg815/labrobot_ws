#!/usr/bin/env python3
"""Measure what a compressed image topic actually costs on the wire.

    python3 tools/image_bandwidth.py                       # colour, 8 s
    python3 tools/image_bandwidth.py --topic <t> --seconds 15
"""
import argparse, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

ap = argparse.ArgumentParser()
ap.add_argument('--topic', default='/camera/camera/color/image_raw/compressed')
ap.add_argument('--seconds', type=float, default=8.0)
a = ap.parse_args()
rclpy.init(); n = Node('image_bandwidth'); c = {'n': 0, 'b': 0}
def cb(m): c['n'] += 1; c['b'] += len(m.data)
n.create_subscription(CompressedImage, a.topic, cb,
                      QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
t0 = time.time()
while time.time() - t0 < a.seconds: rclpy.spin_once(n, timeout_sec=0.05)
d = time.time() - t0
if not c['n']:
    print(f'  no frames on {a.topic} in {a.seconds:.0f} s'); sys.exit(1)
print(f"  {a.topic}: {c['b']/c['n']/1024:.1f} kB/frame  {c['n']/d:.1f} fps  "
      f"{c['b']*8/d/1e6:.2f} Mbit/s")
rclpy.shutdown()
