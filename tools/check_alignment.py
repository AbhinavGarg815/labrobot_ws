#!/usr/bin/env python3
"""Is the robot localised correctly? Measure it against the saved map.

    python3 tools/check_alignment.py

Projects the live aligned depth image into the MAP frame using the current
map->camera transform, keeps returns that are obstacles (0.15-0.90 m above the
floor, within 2.5 m where depth noise is small), and asks the saved /map what is
at each one. If localisation is right, live obstacles land on cells the map
already marks occupied (or within a cell or two of them). If it is wrong they
land in mapped free space.

This is independent of RTAB-Map's own verdict, which is the point: RTAB-Map
reporting "localised" and the pose being right are two different claims.
"""
import math, sys, time
import numpy as np, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid

def quat_to_R(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

rclpy.init(); n = Node('check_alignment')
b = tf2_ros.Buffer(); tf2_ros.TransformListener(b, n)
st = {}
be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', lambda m: st.__setitem__('d', m), be)
n.create_subscription(CameraInfo, '/camera/camera/color/camera_info', lambda m: st.__setitem__('k', m), be)
n.create_subscription(OccupancyGrid, '/map', lambda m: st.__setitem__('map', m),
    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0 = time.time()
while time.time() - t0 < 10 and not all(k in st for k in ('d', 'k', 'map')):
    rclpy.spin_once(n, timeout_sec=0.05)
missing = [k for k in ('d', 'k', 'map') if k not in st]
if missing: print(f'  missing: {missing}'); sys.exit(1)
d, K, M = st['d'], st['k'], st['map']
# Wait for TF to fill. The image/map messages can all arrive before the TF
# buffer holds the camera chain, and a lookup then raises "frame does not
# exist" - which reads like a broken TF tree rather than a race.
t1 = time.time()
while not b.can_transform(M.header.frame_id, d.header.frame_id, rclpy.time.Time()):
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - t1 > 10:
        print(f'  no TF {M.header.frame_id} -> {d.header.frame_id} after 10 s'); sys.exit(1)
tf = b.lookup_transform(M.header.frame_id, d.header.frame_id, rclpy.time.Time())
R = quat_to_R(tf.transform.rotation); t = tf.transform.translation
z = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000.0
fx, fy, cx, cy = K.k[0], K.k[4], K.k[2], K.k[5]
v, u = np.mgrid[0:d.height:4, 0:d.width:4]
zz = z[::4, ::4]
ok = (zz > 0.3) & (zz < 2.5)
P = np.stack([(u[ok]-cx)*zz[ok]/fx, (v[ok]-cy)*zz[ok]/fy, zz[ok]], 1)
W = P @ R.T + np.array([t.x, t.y, t.z])
W = W[(W[:, 2] > 0.15) & (W[:, 2] < 0.90)]
if len(W) < 50: print(f'  only {len(W)} obstacle returns in view - point the camera at something'); sys.exit(1)
res = M.info.resolution; ox, oy = M.info.origin.position.x, M.info.origin.position.y
grid = np.asarray(M.data, np.int16).reshape(M.info.height, M.info.width)
gx = ((W[:, 0]-ox)/res).astype(int); gy = ((W[:, 1]-oy)/res).astype(int)
inb = (gx >= 0) & (gx < M.info.width) & (gy >= 0) & (gy < M.info.height)
gx, gy = gx[inb], gy[inb]
occ = grid >= 50
# distance (cells) to the nearest occupied map cell, via a small dilation ladder
near = {}
cur = occ.copy()
for r in range(0, 4):
    near[r] = cur[gy, gx].copy()
    pad = np.pad(cur, 1)
    cur = pad[1:-1,1:-1] | pad[:-2,1:-1] | pad[2:,1:-1] | pad[1:-1,:-2] | pad[1:-1,2:]
vals = grid[gy, gx]
N = len(vals)
print(f'  {N} live obstacle points inside the map ({len(W)-N} outside it)')
print(f'  on an occupied cell        : {100*near[0].mean():5.1f}%')
print(f'  within 1 cell  (5 cm)      : {100*near[1].mean():5.1f}%')
print(f'  within 3 cells (15 cm)     : {100*near[3].mean():5.1f}%')
print(f'  in mapped FREE space       : {100*((vals==0)&~near[3]).mean():5.1f}%   <- wrong pose, or something new')
print(f'  in unknown space           : {100*(vals<0).mean():5.1f}%')
score = near[3].mean()
print('  VERDICT: ' + ('localised - live view lines up with the map' if score > 0.6 else
      'uncertain - partial overlap' if score > 0.3 else 'NOT localised - live view does not match the map here'))
rclpy.shutdown()
