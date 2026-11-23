#!/usr/bin/env python3
"""Full floor capture: verify the camera model and measure what it can actually see.

    ros2 run labrobot_bringup analyse_floor.py
"""

import argparse
import math
import sys

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


class FloorAnalyser(Node):
    def __init__(self, args):
        super().__init__('analyse_floor')
        self.args = args
        self.frames = []
        self.info = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, args.depth_topic, self._on_depth, qos)
        self.create_subscription(CameraInfo, args.info_topic, self._on_info, qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_depth(self, msg):
        if len(self.frames) < self.args.frames:
            self.frames.append(msg)

    def _on_info(self, msg):
        self.info = msg

    def collect(self, timeout=25.0):
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.frames) >= self.args.frames and self.info is not None:
                return True
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                return len(self.frames) > 0 and self.info is not None
        return False

    def to_base(self, msg):
        info = self.info
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        z = d.astype(np.float32) * 0.001
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]

        step = self.args.step
        vs, us = np.mgrid[0:msg.height:step, 0:msg.width:step]
        zz = z[::step, ::step]
        valid = (zz > 0.15) & (zz < self.args.max_range)
        if valid.sum() < 300:
            return None, None

        zc = zz[valid]
        xc = (us[valid] - cx) * zc / fx
        yc = (vs[valid] - cy) * zc / fy
        pts_opt = np.stack([xc, yc, zc], axis=1)

        try:
            tf = self.tf_buffer.lookup_transform(
                'base_footprint', msg.header.frame_id, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None, None
        R = quat_to_matrix(tf.transform.rotation)
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])
        return pts_opt @ R.T + t, t


def fit_floor(pts, tol=0.03, iters=400):
    rng = np.random.default_rng(0)
    best = None
    cand = pts[(np.abs(pts[:, 2]) < 0.35) & (pts[:, 0] > 0.3)]
    if len(cand) < 500:
        return None
    for _ in range(iters):
        p = cand[rng.choice(len(cand), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        if abs(n[2]) < 0.95:
            continue
        d = -n @ p[0]
        inl = np.abs(cand @ n + d) < tol
        if best is None or inl.sum() > best[0].sum():
            best = (inl, n, d)
    if best is None or best[0].sum() < 500:
        return None
    inl = cand[best[0]]
    cen = inl.mean(axis=0)
    _, _, vh = np.linalg.svd(inl - cen)
    n = vh[2]
    if n[2] < 0:
        n = -n
    return n, -n @ cen, inl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--depth-topic', default='/camera/camera/depth/image_rect_raw')
    ap.add_argument('--info-topic', default='/camera/camera/depth/camera_info')
    ap.add_argument('--frames', type=int, default=10)
    ap.add_argument('--max-range', type=float, default=6.0)
    ap.add_argument('--step', type=int, default=3)
    args = ap.parse_args()

    rclpy.init()
    node = FloorAnalyser(args)
    try:
        if not node.collect():
            print('ERROR: no depth/camera_info/TF. Is the camera up?', file=sys.stderr)
            return 1

        results, cam_pos = [], None
        allpts = []
        for msg in node.frames:
            pts, t = node.to_base(msg)
            if pts is None:
                continue
            cam_pos = t
            fit = fit_floor(pts)
            if fit is None:
                continue
            n, d, inl = fit
            results.append((n, d, inl))
            allpts.append(pts)

        if not results:
            print('ERROR: could not fit a floor plane in any frame. Point the robot '
                  'at open, flat floor.', file=sys.stderr)
            return 1

        print(f'captured {len(node.frames)} frames, {len(results)} usable\n')

        # geometry, averaged over frames
        heights, pitches, rolls, rmss = [], [], [], []
        for n, d, inl in results:
            heights.append(-d / n[2])
            pitches.append(math.degrees(math.atan2(n[0], n[2])))
            rolls.append(math.degrees(math.atan2(-n[1], n[2])))
            rmss.append(float(np.sqrt(((inl @ n + d) ** 2).mean())))

        def stat(v):
            return float(np.mean(v)), float(np.std(v))

        hm, hs = stat(heights)
        pm, ps = stat(pitches)
        rm, rs = stat(rolls)

        print('MOUNT POSE (residual errors - the URDF is right when these are 0)')
        print(f'  floor height : {hm*1000:+7.1f} +/- {hs*1000:.1f} mm')
        print(f'  pitch error  : {pm:+7.3f} +/- {ps:.3f} deg')
        print(f'  roll error   : {rm:+7.3f} +/- {rs:.3f} deg')
        print()

        n0, d0, _ = results[0]
        cam_h = abs(n0 @ cam_pos + d0)
        print('CAMERA HEIGHT, measured independently of the URDF')
        print(f'  perpendicular distance from camera origin to the floor plane')
        print(f'    measured : {cam_h*1000:.1f} mm')
        print(f'    URDF says: {cam_pos[2]*1000:.1f} mm')
        print(f'    agreement: {abs(cam_h-cam_pos[2])*1000:.1f} mm')
        print()

        pts = np.vstack(allpts)
        floor = pts[np.abs(pts[:, 2]) < 0.06]

        print(f'DEPTH NOISE vs RANGE  (mean RMS about the plane: {np.mean(rmss)*1000:.1f} mm)')
        print(f'  {"range band":>14}  {"points":>8}  {"RMS":>8}')
        for lo, hi in ((0.4, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 6.0)):
            band = floor[(floor[:, 0] >= lo) & (floor[:, 0] < hi)]
            if len(band) < 50:
                continue
            rms = float(np.sqrt((band[:, 2] ** 2).mean()))
            print(f'  {lo:5.1f}-{hi:4.1f} m  {len(band):8d}  {rms*1000:6.1f} mm')
        print('  (stereo depth error grows with range^2, so expect this to climb)')
        print()

        print('WHAT THE CAMERA CAN ACTUALLY SEE')
        if len(floor):
            near = np.percentile(floor[:, 0], 0.5)
            far = np.percentile(floor[:, 0], 99.5)
            print(f'  nearest floor seen : {near:.2f} m ahead of base_footprint')
            print(f'  furthest floor seen: {far:.2f} m')
            for r in (0.8, 1.5, 2.5):
                band = floor[(floor[:, 0] > r - 0.1) & (floor[:, 0] < r + 0.1)]
                if len(band) > 50:
                    print(f'  lateral coverage at {r:.1f} m : '
                          f'{band[:,1].min():+.2f} to {band[:,1].max():+.2f} m '
                          f'({band[:,1].max()-band[:,1].min():.2f} m wide)')
            print()
            print('  BLIND ZONE - this is the number Nav2 needs:')
            print(f'    the floor is invisible closer than {near:.2f} m, and the robot')
            print(f'    front edge is at 0.21 m, so there is roughly '
                  f'{max(0.0, near-0.21):.2f} m of')
            print('    ground ahead of the bumper that the camera cannot see at all.')
            print('    Anything entering there must be remembered from earlier frames,')
            print('    which is exactly what a Nav2 costmap does and why clearing it')
            print('    too eagerly is dangerous on a camera-only robot.')
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
