#!/usr/bin/env python3
"""Measure a gap ahead of the robot precisely, from live depth.

    python3 tools/gap_edges.py                 # scan 1.4-2.8 m ahead
    python3 tools/gap_edges.py --near 1.0 --far 2.0

Stacks 10 aligned-depth frames (so one noisy frame cannot set an edge), keeps
returns 0.10-0.90 m above the floor (the costmap's band), and for each 10 cm
slice of distance reports the inner edge of the obstacle on each side of the
robot's centreline, the gap width and its centre. Edges use the 2nd/98th
percentile of the points on each side, not the min/max, so stray pixels at an
object's silhouette do not eat into the gap.

Prints the narrowest slice and the clearance per side for this 0.406 m robot,
which is the number that decides whether the test is meaningful.
"""
import argparse, time, numpy as np, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

ROBOT_WIDTH = 0.406
def R_(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
ap = argparse.ArgumentParser()
ap.add_argument('--near', type=float, default=1.4)
ap.add_argument('--far', type=float, default=2.8)
ap.add_argument('--split', type=float, default=None,
                help='lateral position (m) separating left from right obstacle; '
                     'default: the widest empty column band near the centreline')
ap.add_argument('--save', default=None,
                help='write the edges to this JSON file, in the ODOM frame, so a '
                     'later run can be scored against them (tools/gap_run_report.py)')
a = ap.parse_args()
rclpy.init(); n = Node('gap_edges'); b = tf2_ros.Buffer(); tf2_ros.TransformListener(b, n)
frames, st = [], {}
be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', frames.append, be)
n.create_subscription(CameraInfo, '/camera/camera/color/camera_info', lambda m: st.__setitem__('k', m), be)
t0 = time.time()
while time.time() - t0 < 6: rclpy.spin_once(n, timeout_sec=0.05)
K = st['k']; tf = b.lookup_transform('base_footprint', frames[-1].header.frame_id, rclpy.time.Time())
R = R_(tf.transform.rotation); T = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
pts = []
for d in frames[-10:]:
    z = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000
    v, u = np.mgrid[0:d.height, 0:d.width]; ok = (z > 0.3) & (z < a.far + 0.5)
    P = np.stack([(u[ok]-K.k[2])*z[ok]/K.k[0], (v[ok]-K.k[5])*z[ok]/K.k[4], z[ok]], 1) @ R.T + T
    pts.append(P[(P[:, 2] > 0.10) & (P[:, 2] < 0.90)])
P = np.vstack(pts)
band = P[(P[:, 0] >= a.near) & (P[:, 0] < a.far)]
split = a.split
if split is None:
    # centre of the emptiest 1 cm lateral bin within +-0.5 m of the centreline
    h, e = np.histogram(band[:, 1], bins=100, range=(-0.5, 0.5))
    empty = np.where(h == h.min())[0]
    split = float(np.median((e[empty] + e[empty + 1]) / 2))
print(f'  split between left/right obstacle at y = {split:+.2f} m')
print('    depth        left edge    right edge    width    centre    clearance/side')
rows = []
for lo in np.arange(a.near, a.far, 0.1):
    s = P[(P[:, 0] >= lo) & (P[:, 0] < lo + 0.1)]
    L = s[s[:, 1] > split][:, 1]; Rr = s[s[:, 1] < split][:, 1]
    if len(L) > 50 and len(Rr) > 50:
        le, re = np.percentile(L, 2), np.percentile(Rr, 98)
        w = le - re; rows.append((lo, le, re, w))
        print(f'    {lo:.1f}-{lo+0.1:.1f} m   {le:+.3f}       {re:+.3f}       {w:.3f}    {(le+re)/2:+.3f}     {(w-ROBOT_WIDTH)/2*100:5.1f} cm')
    else:
        print(f'    {lo:.1f}-{lo+0.1:.1f} m   {"open" if len(L) <= 50 else "%+.3f" % np.percentile(L, 2):>6}       '
              f'{"open" if len(Rr) <= 50 else "%+.3f" % np.percentile(Rr, 98):>6}')
if rows and a.save:
    import json, math
    to = b.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
    q = to.transform.rotation; yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw); ox, oy = to.transform.translation.x, to.transform.translation.y
    def od(x, y): return (ox + c*x - s*y, oy + s*x + c*y)
    json.dump({'robot_odom': [ox, oy, yaw],
               'slices': [{'x': lo + 0.05, 'left': od(lo + 0.05, le), 'right': od(lo + 0.05, re),
                           'width': w} for lo, le, re, w in rows]}, open(a.save, 'w'))
    print(f'  edges saved (odom frame) -> {a.save}')
if rows:
    lo, le, re, w = min(rows, key=lambda r: r[3])
    print(f'\n  NARROWEST: {w*100:.1f} cm at {lo:.1f}-{lo+0.1:.1f} m, centre y={(le+re)/2:+.3f} m'
          f' -> {(w-ROBOT_WIDTH)/2*100:.1f} cm clearance per side for the {ROBOT_WIDTH*100:.1f} cm robot')
    print(f'  GAP SPAN: {rows[0][0]:.1f} to {rows[-1][0]+0.1:.1f} m ahead; '
          f'mean centre y={np.mean([(r[1]+r[2])/2 for r in rows]):+.3f} m')
rclpy.shutdown()
