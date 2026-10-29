#!/usr/bin/env python3
"""Check saved waypoints against the current map: free, and how much clearance.

    python3 tools/check_waypoints.py [names...]

A waypoint is only reachable if the robot's centre can stand there: in mapped
free space and further from obstacles than the robot's inscribed radius
(0.213 m with padding). Under ~0.3 m the planner will often refuse or the
goal will be marked missed.
"""
import math, sys, time
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__file__), '..',
                'src', 'labrobot_bringup', 'scripts'))
import numpy as np, rclpy
from rclpy.node import Node
from scipy.ndimage import distance_transform_edt
import global_localise as gl, waypoints as wp
rclpy.init(); n = Node('check_waypoints'); g = gl.get_map(n)
if g is None: print('  no /map'); sys.exit(1)
res = g.info.resolution; W, H = g.info.width, g.info.height
ox, oy = g.info.origin.position.x, g.info.origin.position.y
G = np.asarray(g.data, np.int16).reshape(H, W)
clear = distance_transform_edt(G == 0) * res
wps = wp.load(); names = sys.argv[1:] or sorted(wps)
for name in names:
    w = wps.get(name)
    if w is None: print(f'  {name}: not saved'); continue
    i = int((w['x'] - ox) / res); j = int((w['y'] - oy) / res)
    if not (0 <= i < W and 0 <= j < H): print(f'  {name}: OUTSIDE the map'); continue
    cell = {0: 'free', -1: 'unknown'}.get(int(G[j, i]), 'OBSTACLE')
    c = clear[j, i]
    verdict = 'ok' if cell == 'free' and c >= 0.30 else ('tight' if cell == 'free' and c >= 0.22 else 'UNREACHABLE')
    print(f'  {name:6s} x={w["x"]:+.2f} y={w["y"]:+.2f}: {cell}, clearance {c:.2f} m -> {verdict}')
rclpy.shutdown()
