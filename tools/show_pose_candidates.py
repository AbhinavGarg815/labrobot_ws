#!/usr/bin/env python3
"""Show candidate robot poses in Foxglove so a person can say which is real.

    python3 tools/show_pose_candidates.py A:-0.52,0.34,85 B:-0.30,0.24,92 [--minutes 15]
"""
import argparse
import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

COLOURS = [(0.9, 0.1, 0.1), (0.1, 0.8, 0.1), (0.1, 0.3, 0.9)]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('poses', nargs='+'); ap.add_argument('--minutes', type=float, default=15)
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('pose_candidates')
    pub = n.create_publisher(MarkerArray, '/labrobot/pose_candidates',
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    arr = MarkerArray()
    for k, spec in enumerate(a.poses):
        label, xyz = spec.split(':'); x, y, yaw = (float(v) for v in xyz.split(','))
        r, g, b = COLOURS[k % len(COLOURS)]
        for j, kind in enumerate((Marker.ARROW, Marker.TEXT_VIEW_FACING)):
            m = Marker(); m.header.frame_id = 'map'; m.ns = 'candidates'; m.id = 2*k + j; m.type = kind
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.05 if j == 0 else 0.6
            th = math.radians(yaw); m.pose.orientation.z, m.pose.orientation.w = math.sin(th/2), math.cos(th/2)
            m.scale.x, m.scale.y, m.scale.z = (0.5, 0.08, 0.08) if j == 0 else (0.0, 0.0, 0.25)
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            m.text = f'{label} ({x:+.2f},{y:+.2f})'
            arr.markers.append(m)
    t_end = time.time() + 60 * a.minutes
    while time.time() < t_end:
        stamp = n.get_clock().now().to_msg()
        for m in arr.markers: m.header.stamp = stamp
        pub.publish(arr)
        t = time.time()
        while time.time() - t < 1.0: rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
