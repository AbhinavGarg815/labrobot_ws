#!/usr/bin/env python3
"""Point cloud rate and worst gap - the costmaps go stale on any gap > 1.0 s.

    python3 tools/cloud_rate.py [--seconds 10]
"""
import argparse, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
ap = argparse.ArgumentParser(); ap.add_argument('--seconds', type=float, default=10.0); a = ap.parse_args()
rclpy.init(); n = Node('cloud_rate'); ts = []
n.create_subscription(PointCloud2, '/camera/camera/depth/color/points', lambda m: ts.append(time.time()),
                      QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
t = time.time()
while time.time() - t < a.seconds: rclpy.spin_once(n, timeout_sec=0.02)
g = np.diff(ts) if len(ts) > 1 else np.array([a.seconds])
print(f'  cloud: {len(ts)/a.seconds:.1f} Hz, largest gap {g.max()*1000:.0f} ms, gaps > 1 s: {(g > 1.0).sum()}')
rclpy.shutdown()
