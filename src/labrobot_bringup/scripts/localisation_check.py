#!/usr/bin/env python3
"""Is the robot's map pose right? Refuse to navigate on a wrong one.

    ros2 run labrobot_bringup localisation_check.py            # report only
    ros2 run labrobot_bringup localisation_check.py --selftest # prove it works

Also imported by nav_goal.py, which will not send a goal unless this passes.

Why this exists
---------------
On 2026-10-07 the robot was moved by hand between tests. Neither wheels nor
gyro saw it, so the map pose was 1.8 m wrong when the next goal was issued.
RTAB-Map corrected it 13 s into the run - after the goal had already been
computed from the wrong pose - and the robot drove 2.5 m past where it should
have stopped. Nothing checked the pose before moving.

How it decides
--------------
Live depth returns 0.15-0.90 m above the floor (the costmap's band, within
2.5 m where depth noise is small) are projected into the map and scored as the
fraction landing within one cell of a mapped obstacle.

A low score alone does NOT mean a wrong pose: objects placed after mapping (two
boxes forming a test gap) land in mapped free space even when the pose is
perfect - a correct pose scored ~50% that way. So the decision is relative:
search nearby poses (+-2 m, +-15 deg) for one that explains the view much
better.

  OK       no nearby pose is clearly better -> unmatched points are new objects
  WRONG    a pose > 0.25 m or > 8 deg away scores >= 0.15 better -> refuse
  UNSURE   too few obstacle returns, or too little matches anywhere -> refuse
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

# Decision thresholds. A pose this far away scoring this much better is a wrong
# pose, not noise.
WRONG_DIST_M = 0.25
WRONG_YAW_DEG = 8.0
WRONG_GAIN = 0.15
MIN_POINTS = 300
MIN_BEST_SCORE = 0.35


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
    # Wait for the transforms rather than failing on the first miss. Called from
    # nav_goal.py the node is only a moment old: map->base_footprint arrives
    # quickly on /tf, but the camera's fixed frames come from /tf_static and can
    # lag behind. The first version looked up once and reported UNSURE ("two or
    # more unconnected trees") on a perfectly good pose.
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
    """Score the pose, then search nearby poses. offset perturbs the pose
    (used by --selftest to simulate a wrong one).

    A point counts only if it is CONSISTENT with the map, not merely near an
    obstacle: it must land within one cell of a mapped obstacle AND the ray
    from the robot to it must cross no mapped obstacle on the way. The first
    version scored "near an obstacle" alone, and the search then always found
    a pose scoring 100% - shift the cloud into a dense block of furniture and
    every point is near something. Real surfaces have free space in front of
    them; the ray test is what rules those impossible poses out.
    """
    P, M = data['pts'], data['map']
    if len(P) > 2000:
        P = P[np.random.default_rng(1).choice(len(P), 2000, replace=False)]
    x0, y0, yaw0 = data['x'] + offset[0], data['y'] + offset[1], data['yaw'] + offset[2]
    res = M.info.resolution
    ox, oy = M.info.origin.position.x, M.info.origin.position.y
    W, H = M.info.width, M.info.height
    occ = np.asarray(M.data, np.int16).reshape(H, W) >= 50
    pad = np.pad(occ, 1)
    near = (pad[1:-1, 1:-1] | pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:])
    m = 45   # cells of margin so translated lookups stay in bounds (outside = free)
    NEAR, OCC = np.pad(near, m), np.pad(occ, m)
    # Ray samples, as fractions of each point's range. They stop short of the
    # point itself (and the robot's own body) so the target surface and the
    # mapped obstacle it sits on cannot veto their own ray.
    rng = np.hypot(P[:, 0], P[:, 1])
    fracs = [f for f in (0.2, 0.4, 0.6, 0.8)]
    def keep(f):   # only sample where it is > 1.5 cells short of the point
        return (rng * (1 - f)) > 1.5 * res

    def indices(pts, dyaw):
        th = yaw0 + dyaw
        c, s = math.cos(th), math.sin(th)
        wx = x0 + c*pts[:, 0] - s*pts[:, 1]
        wy = y0 + s*pts[:, 0] + c*pts[:, 1]
        return (np.floor((wx-ox)/res).astype(int) + m, np.floor((wy-oy)/res).astype(int) + m)

    def look(grid, ij, di, dj):
        i, j = ij[0] + di, ij[1] + dj
        out = np.zeros(len(i), bool)
        k = (i >= 0) & (i < grid.shape[1]) & (j >= 0) & (j < grid.shape[0])
        out[k] = grid[j[k], i[k]]
        return out

    def prepare(dyaw):
        return (indices(P, dyaw),
                [(indices(P * f, dyaw), keep(f)) for f in fracs],
                indices(np.zeros((1, 2)), dyaw))

    def score(prep, di, dj):
        tgt, rays, origin = prep
        if look(OCC, origin, di, dj)[0]:
            return 0.0                          # robot inside a mapped obstacle
        good = look(NEAR, tgt, di, dj)
        for ij, kp in rays:
            good &= ~(look(OCC, ij, di, dj) & kp)
        return good.mean()

    here = score(prepare(0.0), 0, 0)
    best = (here, 0.0, 0.0, 0.0)
    step = 2                       # cells (10 cm) for the coarse search
    span = int(round(2.0 / res))   # +-2 m
    for dyaw in np.radians(np.arange(-15, 15.1, 3)):
        prep = prepare(dyaw)
        for di in range(-span, span + 1, step):
            for dj in range(-span, span + 1, step):
                sc = score(prep, di, dj)
                if sc > best[0]:
                    best = (sc, di*res, dj*res, dyaw)
    dist = math.hypot(best[1], best[2])
    dyaw_deg = math.degrees(best[3])
    edge = dist > 1.9 or abs(dyaw_deg) > 14.0
    if len(P) < MIN_POINTS:
        verdict, why = 'UNSURE', f'only {len(P)} obstacle returns in view - face some structure'
    elif best[0] < MIN_BEST_SCORE:
        verdict, why = 'UNSURE', f'nothing matches the map anywhere nearby (best {best[0]:.0%})'
    elif best[0] - here >= WRONG_GAIN and (dist > WRONG_DIST_M or abs(dyaw_deg) > WRONG_YAW_DEG):
        verdict, why = 'WRONG', (f'a pose {dist:.2f} m / {dyaw_deg:+.0f} deg away matches '
                                 f'{best[0]:.0%} vs {here:.0%} here'
                                 + (' (at the search edge - may be further)' if edge else ''))
    else:
        verdict, why = 'OK', (f'{here:.0%} of live obstacles consistent with the map; no nearby '
                              f'pose clearly better (best {best[0]:.0%} at {dist:.2f} m / {dyaw_deg:+.0f} deg)')
    return {'verdict': verdict, 'why': why, 'here': here, 'best': best[0],
            'shift': (best[1], best[2], dyaw_deg), 'n': len(P)}


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
        print('\n  self-test - the same view scored from deliberately wrong poses:')
        caught = 0
        cases = [(1.8, 0.0, 0.0), (-0.6, 0.4, 0.0), (0.0, 0.0, math.radians(12)), (0.3, 0.0, 0.0)]
        for dx, dy, dth in cases:
            c, s = math.cos(data['yaw']), math.sin(data['yaw'])
            off = (c*dx - s*dy, s*dx + c*dy, dth)     # offset given in the robot frame
            rr = evaluate(data, off)
            expect_wrong = math.hypot(dx, dy) > WRONG_DIST_M or abs(math.degrees(dth)) > WRONG_YAW_DEG
            good = (rr['verdict'] != 'OK') == expect_wrong
            caught += good
            print(f'    pose off by fwd {dx:+.1f} m, left {dy:+.1f} m, yaw {math.degrees(dth):+.0f} deg '
                  f'-> {rr["verdict"]:6s} ({rr["here"]:.0%} here, best {rr["best"]:.0%})  '
                  f'{"correct" if good else "MISSED"}')
        print(f'  self-test: {caught}/{len(cases)} classified correctly')
        rc = rc or (0 if caught == len(cases) else 3)
    node.destroy_node(); rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
