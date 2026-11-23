#!/usr/bin/env python3
"""Top-down view of what the camera sees ahead, in the robot frame.

    python3 tools/depth_ahead.py
"""
import time, numpy as np, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

def R_(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
rclpy.init(); n = Node('depth_ahead'); b = tf2_ros.Buffer(); tf2_ros.TransformListener(b, n); st = {}
be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', lambda m: st.__setitem__('d', m), be)
n.create_subscription(CameraInfo, '/camera/camera/color/camera_info', lambda m: st.__setitem__('k', m), be)
t0 = time.time()
while time.time() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.05)
d, K = st['d'], st['k']
tf = b.lookup_transform('base_footprint', d.header.frame_id, rclpy.time.Time())
z = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000
v, u = np.mgrid[0:d.height:2, 0:d.width:2]; zz = z[::2, ::2]; ok = (zz > 0.3) & (zz < 3.5)
P = np.stack([(u[ok]-K.k[2])*zz[ok]/K.k[0], (v[ok]-K.k[5])*zz[ok]/K.k[4], zz[ok]], 1) @ R_(tf.transform.rotation).T
t = tf.transform.translation; P += np.array([t.x, t.y, t.z])
P = P[(P[:, 2] > 0.10) & (P[:, 2] < 0.90)]
print('  left +1.0 m ' + '.' * 8 + ' | ' + '.' * 8 + ' right -1.0 m   (| = robot centreline, 10 cm/col)')
for lo in np.arange(3.0, 0.39, -0.2):
    s = P[(P[:, 0] >= lo) & (P[:, 0] < lo + 0.2)]
    h, _ = np.histogram(-s[:, 1], bins=20, range=(-1.0, 1.0))
    row = ''.join('#' if c > 15 else ('.' if c > 0 else ' ') for c in h)
    print(f'  {lo:.1f}-{lo+0.2:.1f} m  [{row[:10]}|{row[10:]}]')
print('  robot here  [' + ' ' * 7 + 'RRRR|RRRR' + ' ' * 4 + ']   (robot is 41 cm wide)')
rclpy.shutdown()
