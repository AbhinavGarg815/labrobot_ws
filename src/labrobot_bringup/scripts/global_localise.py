#!/usr/bin/env python3
"""Find the robot anywhere in the map from the room's GEOMETRY, not its looks.

    ros2 run labrobot_bringup global_localise.py --spin      # turn 360, then match
    ros2 run labrobot_bringup global_localise.py             # match what is in view

The kidnapped-robot problem, without markers
--------------------------------------------
RTAB-Map relocalises by appearance. A week after mapping, with furniture moved
and different light, 20 of 22 candidates came back with 0 inliers (2026-10-14).
Walls, pillars, doorways and fixed benches do not move, and their SHAPE does not
depend on lighting. So:

1. During one slow 360 deg turn in place, stitch depth into a full ring of the
   room's shape around the robot. The camera sees 87 deg at a time; over a
   turn on the spot odometry is accurate (yaw comes from the gyro), so the views
   join up. Only returns 0.10-0.90 m above the floor are kept - the same band
   the occupancy map was built from.
2. Correlative scan matching (Olson, "Real-time correlative scan matching",
   ICRA 2009): for every heading (2 deg steps) the ring is rasterised and
   cross-correlated with a likelihood field of the map's occupied cells by FFT,
   which scores EVERY position at once. The map is ~12 x 11 m, so the whole
   search takes about a second. Positions inside obstacles or unknown space
   are excluded - the robot must be standing in mapped free space.
3. Confidence is DISTINCTNESS, not score: the best pose must clearly beat the
   best pose that is a genuinely different place (> 1 m or > 20 deg away). If
   two places fit almost equally (symmetric room, too much has moved), the
   answer is AMBIGUOUS and the caller should move to a new viewpoint and retry.

The ring is reduced to a virtual 2D laser scan (nearest return per 0.5 deg
bearing) and scored against obstacle SURFACES (occupied cells bordering free
space) - see ring_in_robot_frame() and match() for why.

Moved furniture appears as a minority of outliers; a full ring is dominated by
walls and fixed structure, which is what makes this robust where appearance is
not.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from builtin_interfaces.msg import Duration
from nav2_msgs.action import Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
INFO_TOPIC = '/camera/camera/color/camera_info'
Z_MIN, Z_MAX = 0.10, 0.90          # same height band as the occupancy grid
R_MIN, R_MAX = 0.40, 4.0           # depth noise grows with range^2; 4 m is the grid's range
SIGMA = 0.10                       # likelihood-field width (m)
ANGLE_STEP = 2.0                   # deg
DIFFERENT_PLACE_M = 1.0
DIFFERENT_PLACE_DEG = 20.0
MAX_RUNNER_UP = 0.80               # runner-up must score below 80% of the best
FREE_WEIGHT = 2.0
BEARING_BIN = 0.5                  # deg per virtual laser beam                  # penalty per ray sample landing on a mapped obstacle


def _rot(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class RingCollector:
    """Accumulates depth returns in the ODOM frame while the robot turns."""

    def __init__(self, node, buf, every_deg=8.0):
        self.node, self.buf, self.every = node, buf, math.radians(every_deg)
        self.K = None
        self.pts = []               # list of (N,2) odom-frame xy arrays
        self.last_yaw = None
        self.turned = 0.0
        self.frames = 0
        be = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(CameraInfo, INFO_TOPIC, lambda m: setattr(self, 'K', m), be)
        node.create_subscription(Image, DEPTH_TOPIC, self.on_depth, be)

    def on_depth(self, d):
        if self.K is None:
            return
        stamp = Time.from_msg(d.header.stamp)
        try:
            odom_base = self.buf.lookup_transform('odom', 'base_footprint', stamp)
            odom_cam = self.buf.lookup_transform('odom', d.header.frame_id, stamp)
        except Exception:  # noqa: BLE001  (frame older/newer than the TF cache)
            return
        yaw = _yaw(odom_base.transform.rotation)
        if self.last_yaw is not None:
            dy = abs(math.atan2(math.sin(yaw - self.last_yaw), math.cos(yaw - self.last_yaw)))
            if dy < self.every:
                return               # only keep a frame every ~8 deg of turn
            self.turned += dy
        self.last_yaw = yaw
        z = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000.0
        v, u = np.mgrid[0:d.height:3, 0:d.width:3]
        zz = z[::3, ::3]
        ok = (zz > R_MIN) & (zz < R_MAX)
        K = self.K.k
        P = np.stack([(u[ok]-K[2])*zz[ok]/K[0], (v[ok]-K[5])*zz[ok]/K[4], zz[ok]], 1)
        t = odom_cam.transform.translation
        P = P @ _rot(odom_cam.transform.rotation).T + np.array([t.x, t.y, t.z])
        P = P[(P[:, 2] > Z_MIN) & (P[:, 2] < Z_MAX)]
        if len(P):
            self.pts.append(P[:, :2].astype(np.float32))
        self.frames += 1

    def ring_in_robot_frame(self, res):
        """All points, in the robot frame at its CURRENT pose, one per grid cell."""
        if not self.pts:
            return np.zeros((0, 2), np.float32)
        P = np.vstack(self.pts)
        tf = self.buf.lookup_transform('odom', 'base_footprint', Time())
        x0, y0 = tf.transform.translation.x, tf.transform.translation.y
        th = _yaw(tf.transform.rotation); c, s = math.cos(th), math.sin(th)
        dx, dy = P[:, 0] - x0, P[:, 1] - y0
        R = np.stack([c*dx + s*dy, -s*dx + c*dy], 1)
        # A virtual 2D LASER SCAN: per 0.5 deg bearing keep only the NEAREST
        # return - the surface the camera actually sees. Keeping every point
        # (first version, 2026-10-14) smeared each wall into a radial band of
        # range noise and of points behind the first surface, so whole regions
        # matched and the answer was ambiguous.
        rng = np.hypot(R[:, 0], R[:, 1])
        bearing = np.floor((np.degrees(np.arctan2(R[:, 1], R[:, 0])) + 180.0) / BEARING_BIN).astype(int)
        order = np.lexsort((rng, bearing))           # by bearing, then range
        b_sorted = bearing[order]
        first = np.r_[True, b_sorted[1:] != b_sorted[:-1]]
        return R[order[first]]


def likelihood_field(occ, res):
    """exp(-d^2 / 2 sigma^2), d = distance to the nearest occupied cell."""
    try:
        from scipy.ndimage import distance_transform_edt
        d = distance_transform_edt(~occ) * res
    except ImportError:              # pragma: no cover - scipy is present on the Jetson
        import cv2
        d = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res
    return np.exp(-(d ** 2) / (2 * SIGMA ** 2)).astype(np.float32)


def match(ring, grid_msg):
    """Score every (x, y, heading) in the map. Returns a dict with the best pose
    (map frame), its score, and the best score at a genuinely different place."""
    res = grid_msg.info.resolution
    W, H = grid_msg.info.width, grid_msg.info.height
    ox, oy = grid_msg.info.origin.position.x, grid_msg.info.origin.position.y
    G = np.asarray(grid_msg.data, np.int16).reshape(H, W)
    occ = G >= 50
    free = G == 0
    # Score against the obstacle SURFACES only - occupied cells that border
    # free space. A camera never sees inside a block of furniture, and the
    # map's furniture is mostly solid blocks: scoring against whole blocks made
    # large areas look equally good.
    from scipy.ndimage import binary_dilation
    surface = occ & binary_dilation(free, iterations=1)
    L = likelihood_field(surface, res)
    n = len(ring)
    if n < 100:
        return {'ok': False, 'why': f'only {n} ring points - too little structure in view'}

    # FFT sizes: map plus the scan's extent, so the correlation does not wrap.
    ext = int(math.ceil(R_MAX / res)) + 2
    FH, FW = H + 2 * ext, W + 2 * ext
    Lp = np.zeros((FH, FW), np.float32); Lp[ext:ext+H, ext:ext+W] = L
    FL = np.fft.rfft2(Lp)
    Op = np.zeros((FH, FW), np.float32); Op[ext:ext+H, ext:ext+W] = occ
    FO = np.fft.rfft2(Op)
    # Robot must stand in mapped free space, away from walls.
    from scipy.ndimage import binary_erosion
    standable = binary_erosion(free, iterations=3)       # >= 15 cm from non-free

    best = []                                             # per angle: (score, x, y, th)
    angles = np.radians(np.arange(0, 360, ANGLE_STEP))
    for th in angles:
        c, s = math.cos(th), math.sin(th)
        Rx = c*ring[:, 0] - s*ring[:, 1]; Ry = s*ring[:, 0] + c*ring[:, 1]
        S = np.zeros((FH, FW), np.float32)
        # Scan image indexed relative to the robot at (ext, ext) in a small patch,
        # placed at the FFT origin so correlation peak index = robot cell.
        i = np.round(Rx / res).astype(int); j = np.round(Ry / res).astype(int)
        np.add.at(S, (j % FH, i % FW), 1.0)
        C = np.fft.irfft2(FL * np.conj(np.fft.rfft2(S)), s=(FH, FW))
        C = C[ext:ext+H, ext:ext+W] / n                   # C[r, c] = mean L at robot cell (r, c)
        # Free-space term: each ray also says the space before its surface is
        # EMPTY. Sample every ray at 30/55/80% of its length; any of those
        # samples landing on a mapped obstacle is evidence against the pose.
        # This is what separates a rotated/mirrored alias, which tends to put a
        # mapped wall inside the ring, from the true pose.
        F = np.zeros((FH, FW), np.float32)
        for frac in (0.30, 0.55, 0.80):
            fi = np.round(frac * Rx / res).astype(int); fj = np.round(frac * Ry / res).astype(int)
            np.add.at(F, (fj % FH, fi % FW), 1.0)
        Cf = np.fft.irfft2(FO * np.conj(np.fft.rfft2(F)), s=(FH, FW))
        C = C - FREE_WEIGHT * Cf[ext:ext+H, ext:ext+W] / (3 * n)
        C[~standable] = -1.0
        k = int(np.argmax(C)); r, col = divmod(k, W)
        best.append((float(C[r, col]), ox + (col + 0.5) * res, oy + (r + 0.5) * res, th, C))

    order = sorted(range(len(best)), key=lambda a: -best[a][0])
    b = best[order[0]]
    # Runner-up: the best score anywhere that is a DIFFERENT place - search all
    # angles' score maps, masking out the neighbourhood of the winner.
    runner = -1.0; runner_pose = None
    rad = int(DIFFERENT_PLACE_M / res)
    for a in range(len(best)):
        sc, x, y, th, C = best[a]
        dth = abs(math.degrees(math.atan2(math.sin(th - b[3]), math.cos(th - b[3]))))
        Cm = C.copy()
        if dth <= DIFFERENT_PLACE_DEG:
            r0 = int((b[2] - oy) / res); c0 = int((b[1] - ox) / res)
            Cm[max(0, r0-rad):r0+rad+1, max(0, c0-rad):c0+rad+1] = -1.0
        k = int(np.argmax(Cm)); r, col = divmod(k, W)
        if Cm[r, col] > runner:
            runner = float(Cm[r, col])
            runner_pose = (ox + (col + 0.5) * res, oy + (r + 0.5) * res, th)
    ratio = runner / b[0] if b[0] > 0 else 1.0
    return {'ok': ratio < MAX_RUNNER_UP, 'score': b[0], 'runner_up': runner,
            'ratio': ratio, 'pose': (b[1], b[2], b[3]), 'runner_pose': runner_pose,
            'points': n,
            'why': ('distinct' if ratio < MAX_RUNNER_UP else
                    f'AMBIGUOUS - a different place scores {ratio:.0%} of the best')}


def save_debug_png(path, grid_msg, ring, pose):
    """Map in grey, ring overlaid in red at the matched pose, robot in blue."""
    try:
        import cv2
    except ImportError:
        return False
    res = grid_msg.info.resolution; W, H = grid_msg.info.width, grid_msg.info.height
    ox, oy = grid_msg.info.origin.position.x, grid_msg.info.origin.position.y
    G = np.asarray(grid_msg.data, np.int16).reshape(H, W)
    img = np.full((H, W, 3), 200, np.uint8); img[G == 0] = 255; img[G >= 50] = 0
    x, y, th = pose; c, s = math.cos(th), math.sin(th)
    wx = x + c*ring[:, 0] - s*ring[:, 1]; wy = y + s*ring[:, 0] + c*ring[:, 1]
    i = ((wx - ox) / res).astype(int); j = ((wy - oy) / res).astype(int)
    k = (i >= 0) & (i < W) & (j >= 0) & (j < H)
    img[j[k], i[k]] = (0, 0, 255)
    ri, rj = int((x - ox) / res), int((y - oy) / res)
    cv2.circle(img, (ri, rj), 4, (255, 0, 0), -1)
    cv2.line(img, (ri, rj), (int(ri + 8*c), int(rj + 8*s)), (255, 0, 0), 1)
    img = cv2.flip(img, 0)                               # map y up
    img = cv2.resize(img, (W * 3, H * 3), interpolation=cv2.INTER_NEAREST)
    return cv2.imwrite(path, img)


def get_map(node, timeout=15.0):
    st = {}
    q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(OccupancyGrid, '/map', lambda m: st.__setitem__('m', m), q)
    t0 = time.time()
    while 'm' not in st and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return st.get('m')


def spin_360(node, collector, allowance=120):
    cli = ActionClient(node, Spin, 'spin')
    if not cli.wait_for_server(timeout_sec=20):
        return 'no Spin server'
    g = Spin.Goal(); g.target_yaw = 2 * math.pi; g.time_allowance = Duration(sec=allowance)
    f = cli.send_goal_async(g); rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    h = f.result()
    if h is None or not h.accepted:
        return 'Spin rejected'
    r = h.get_result_async()
    while not r.done():
        rclpy.spin_once(node, timeout_sec=0.02)
    return 'ok' if r.result().status == 4 else f'Spin ended with status {r.result().status}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--spin', action='store_true', help='turn 360 deg (Nav2 Spin) to build the ring')
    ap.add_argument('--debug-png', default=os.path.expanduser('~/global_localise.png'))
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(); n = Node('global_localise')
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=120))
    tf2_ros.TransformListener(buf, n)
    grid = get_map(n)
    if grid is None:
        print('  no /map - is RTAB-Map running?'); return 1
    col = RingCollector(n, buf)
    t0 = time.time()
    while time.time() - t0 < 3:
        rclpy.spin_once(n, timeout_sec=0.05)
    if args.spin:
        print('  turning 360 deg to build the ring - ROBOT TURNS', flush=True)
        r = spin_360(n, col)
        print(f'  spin: {r}, turned {math.degrees(col.turned):.0f} deg, {col.frames} frames', flush=True)
    t1 = time.time()
    ring = col.ring_in_robot_frame(grid.info.resolution)
    out = match(ring, grid)
    dt = time.time() - t1
    if 'pose' not in out:
        print(f'  {out["why"]}'); return 1
    x, y, th = out['pose']
    print(f'  ring: {out["points"]} points; search took {dt:.1f} s')
    print(f'  best pose: x={x:+.2f} y={y:+.2f} yaw={math.degrees(th):+.0f} deg  score {out["score"]:.3f}')
    rx, ry, rth = out['runner_pose']
    print(f'  runner-up (different place): x={rx:+.2f} y={ry:+.2f} yaw={math.degrees(rth):+.0f} '
          f'score {out["runner_up"]:.3f} = {out["ratio"]:.0%} of best')
    print(f'  verdict: {"DISTINCT" if out["ok"] else out["why"]}')
    try:
        tf = buf.lookup_transform('map', 'base_footprint', Time())
        p = tf.transform.translation
        print(f'  (RTAB-Map currently says: x={p.x:+.2f} y={p.y:+.2f} '
              f'yaw={math.degrees(_yaw(tf.transform.rotation)):+.0f} deg)')
    except Exception:  # noqa: BLE001
        pass
    if save_debug_png(args.debug_png, grid, ring, out['pose']):
        print(f'  picture: {args.debug_png}')
    n.destroy_node(); rclpy.shutdown()
    return 0 if out['ok'] else 2


if __name__ == '__main__':
    sys.exit(main())
