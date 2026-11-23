#!/usr/bin/env python3
"""Is the robot's map pose right?

    ros2 run labrobot_bringup localisation_check.py            # report only
    ros2 run labrobot_bringup localisation_check.py --selftest # prove it works
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
INFO_TOPIC = '/camera/camera/color/camera_info'

# Decision thresholds (see evaluate() for the measurements behind them).
MAX_FREE_FAR = 0.25    # correct poses measured 6.7% and 14%; wrong ones 27.8-40%
MIN_NEAR_OBS = 0.60
MIN_POINTS = 300


def _rot(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def gather(node, buf, seconds=6.0):
    """Collect depth, camera info and the map on an EXISTING node."""
    st = {}
    be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs = [node.create_subscription(Image, DEPTH_TOPIC, lambda m: st.__setitem__('d', m), be),
            node.create_subscription(CameraInfo, INFO_TOPIC, lambda m: st.__setitem__('k', m), be),
            node.create_subscription(OccupancyGrid, '/map', lambda m: st.__setitem__('m', m), latched)]
    t0 = time.time()
    while time.time() - t0 < seconds and not all(k in st for k in 'dkm'):
        rclpy.spin_once(node, timeout_sec=0.05)
    for s in subs:
        node.destroy_subscription(s)
    missing = [n for k, n in (('d', 'depth'), ('k', 'camera_info'), ('m', '/map')) if k not in st]
    if missing:
        return None, f'missing {", ".join(missing)}'
    d, K, M = st['d'], st['k'], st['m']
    # Wait for the transforms rather than failing on the first miss.
    cam = pose = None
    err = None
    deadline = time.time() + 5.0
    while time.time() < deadline:
        try:
            cam = buf.lookup_transform('base_footprint', d.header.frame_id, rclpy.time.Time())
            pose = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            break
        except Exception as e:  # noqa: BLE001
            err = e
            rclpy.spin_once(node, timeout_sec=0.1)
    if cam is None or pose is None:
        return None, f'TF unavailable after 5 s: {err}'
    z = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000.0
    v, u = np.mgrid[0:d.height:4, 0:d.width:4]
    zz = z[::4, ::4]
    ok = (zz > 0.3) & (zz < 2.5)
    P = np.stack([(u[ok]-K.k[2])*zz[ok]/K.k[0], (v[ok]-K.k[5])*zz[ok]/K.k[4], zz[ok]], 1)
    t = cam.transform.translation
    P = P @ _rot(cam.transform.rotation).T + np.array([t.x, t.y, t.z])
    P = P[(P[:, 2] > 0.15) & (P[:, 2] < 0.90)][:, :2]   # robot-frame xy
    if len(P) > 4000:
        P = P[np.random.default_rng(0).choice(len(P), 4000, replace=False)]
    p = pose.transform.translation
    return {'pts': P, 'map': M, 'x': p.x, 'y': p.y, 'yaw': _yaw(pose.transform.rotation)}, None


def evaluate(data, offset=(0.0, 0.0, 0.0)):
    """Verdict from two numbers, computed at the current pose only."""
    P, M = data['pts'], data['map']
    x0, y0, yaw0 = data['x'] + offset[0], data['y'] + offset[1], data['yaw'] + offset[2]
    res = M.info.resolution
    ox, oy = M.info.origin.position.x, M.info.origin.position.y
    W, H = M.info.width, M.info.height
    G = np.asarray(M.data, np.int16).reshape(H, W)
    near = G >= 50
    for _ in range(2):                       # dilate 2 cells = 10 cm
        q = np.pad(near, 1)
        near = q[1:-1, 1:-1] | q[:-2, 1:-1] | q[2:, 1:-1] | q[1:-1, :-2] | q[1:-1, 2:]
    c, s = math.cos(yaw0), math.sin(yaw0)
    wx = x0 + c*P[:, 0] - s*P[:, 1]
    wy = y0 + s*P[:, 0] + c*P[:, 1]
    i = np.floor((wx - ox) / res).astype(int); j = np.floor((wy - oy) / res).astype(int)
    k = (i >= 0) & (i < W) & (j >= 0) & (j < H)
    val = np.full(len(P), -1); val[k] = G[j[k], i[k]]
    nr = np.zeros(len(P), bool); nr[k] = near[j[k], i[k]]
    rng = np.hypot(P[:, 0], P[:, 1]); far = rng >= np.median(rng)
    free_far = float(((val == 0) & ~nr & far).sum() / max(1, far.sum()))
    near_obs = float(nr.mean()) if len(P) else 0.0
    if len(P) < MIN_POINTS:
        verdict, why = 'UNSURE', f'only {len(P)} obstacle returns in view - face some structure'
    elif free_far > MAX_FREE_FAR or near_obs < MIN_NEAR_OBS:
        verdict, why = 'WRONG', (f'{free_far:.0%} of far points in mapped free space (max '
                                 f'{MAX_FREE_FAR:.0%}), {near_obs:.0%} near mapped obstacles '
                                 f'(min {MIN_NEAR_OBS:.0%})')
    else:
        verdict, why = 'OK', (f'{free_far:.0%} of far points in mapped free space, '
                              f'{near_obs:.0%} near mapped obstacles')
    return {'verdict': verdict, 'why': why, 'free_far': free_far, 'near_obs': near_obs,
            'n': len(P)}


def check(node, buf):
    data, err = gather(node, buf)
    if err:
        return {'verdict': 'UNSURE', 'why': err}
    return evaluate(data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--selftest', action='store_true',
                    help='also score deliberately wrong poses and show they are caught')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Node('localisation_check')
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)
    t0 = time.time()
    while time.time() - t0 < 3:
        rclpy.spin_once(node, timeout_sec=0.05)
    data, err = gather(node, buf)
    if err:
        print(f'  UNSURE: {err}'); return 2
    r = evaluate(data)
    print(f'  {r["verdict"]}: {r["why"]}  ({r["n"]} points)')
    rc = 0 if r['verdict'] == 'OK' else 1
    if args.selftest:
        print('\n  self-test - this view from the current pose (assumed correct: run it only when\n'
              '  the pose has been confirmed, e.g. against Foxglove) and from deliberately wrong poses:')
        caught = 0
        cases = [(0.0, 0.0, 0.0), (0.0, -0.15, 0.0), (0.0, 0.15, 0.0), (0.0, 0.0, math.radians(5)),
                 (0.0, 0.0, math.radians(12)), (-0.6, 0.4, 0.0), (1.8, 0.0, 0.0)]
        for dx, dy, dth in cases:
            c, s = math.cos(data['yaw']), math.sin(data['yaw'])
            off = (c*dx - s*dy, s*dx + c*dy, dth)     # offset given in the robot frame
            rr = evaluate(data, off)
            expect_wrong = (dx, dy, dth) != (0.0, 0.0, 0.0)
            good = (rr['verdict'] == 'WRONG') == expect_wrong
            caught += good
            print(f'    pose off by fwd {dx:+.2f} m, left {dy:+.2f} m, yaw {math.degrees(dth):+3.0f} deg '
                  f'-> {rr["verdict"]:6s} (free_far {rr["free_far"]:.0%}, near {rr["near_obs"]:.0%})  '
                  f'{"correct" if good else "MISSED" if expect_wrong else "FALSE ALARM"}')
        print(f'  self-test: {caught}/{len(cases)} classified correctly')
        rc = rc or (0 if caught == len(cases) else 3)
    node.destroy_node(); rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
