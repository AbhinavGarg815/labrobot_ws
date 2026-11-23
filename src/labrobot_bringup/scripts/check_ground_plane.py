#!/usr/bin/env python3
"""Verify the camera mount pose and depth quality from the floor itself.

    ros2 run labrobot_bringup check_ground_plane.py
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


class GroundPlaneCheck(Node):
    def __init__(self, args):
        super().__init__('check_ground_plane')
        self.args = args
        self.depth = None
        self.info = None

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, args.depth_topic, self._on_depth, qos)
        self.create_subscription(CameraInfo, args.info_topic, self._on_info, qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg

    def wait(self, timeout=20.0):
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.depth is not None and self.info is not None:
                return True
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                return False
        return False

    def points_in_base(self):
        """Deprojects the depth image and transforms it into base_footprint."""
        msg, info = self.depth, self.info
        if msg.encoding != '16UC1':
            self.get_logger().error(f'unexpected depth encoding {msg.encoding}')
            return None

        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        # RealSense Z16 is in millimetres.
        z = d.astype(np.float32) * 0.001

        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]

        step = self.args.step
        vs, us = np.mgrid[0:msg.height:step, 0:msg.width:step]
        zz = z[::step, ::step]

        valid = (zz > 0.2) & (zz < self.args.max_range)
        if valid.sum() < 500:
            self.get_logger().error(
                f'only {valid.sum()} valid depth points - is the camera looking at anything?')
            return None

        zc = zz[valid]
        xc = (us[valid] - cx) * zc / fx
        yc = (vs[valid] - cy) * zc / fy
        pts_optical = np.stack([xc, yc, zc], axis=1)

        try:
            tf = self.tf_buffer.lookup_transform(
                'base_footprint', msg.header.frame_id, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

        R = quat_to_matrix(tf.transform.rotation)
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])
        pts = pts_optical @ R.T + t

        # Pre-filter to points that could plausibly BE the floor.
        plausible = (np.abs(pts[:, 2]) < 0.35) & (pts[:, 0] > 0.4)
        if plausible.sum() < 500:
            self.get_logger().error(
                f'only {plausible.sum()} points near the ground plane - point the '
                'robot at open floor')
            return None
        return pts[plausible]

    def fit_plane(self, pts):
        """RANSAC plane fit, then least-squares refit on the inliers."""
        rng = np.random.default_rng(0)
        best_inliers, best_n, best_d = None, None, None
        tol = self.args.inlier_tol

        for _ in range(self.args.iterations):
            idx = rng.choice(len(pts), 3, replace=False)
            p0, p1, p2 = pts[idx]
            n = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(n)
            if norm < 1e-9:
                continue
            n = n / norm
            # The floor's normal is near vertical.
            if abs(n[2]) < 0.95:
                continue
            d = -n @ p0
            dist = np.abs(pts @ n + d)
            inliers = dist < tol
            if best_inliers is None or inliers.sum() > best_inliers.sum():
                best_inliers, best_n, best_d = inliers, n, d

        if best_inliers is None or best_inliers.sum() < 500:
            return None

        # Refit on inliers via SVD about their centroid.
        inl = pts[best_inliers]
        centroid = inl.mean(axis=0)
        _, _, vh = np.linalg.svd(inl - centroid)
        n = vh[2]
        if n[2] < 0:
            n = -n
        d = -n @ centroid
        resid = inl @ n + d
        return dict(n=n, d=d, inliers=inl, centroid=centroid,
                    rms=float(np.sqrt((resid**2).mean())),
                    n_inliers=int(best_inliers.sum()), n_total=len(pts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--depth-topic', default='/camera/camera/depth/image_rect_raw')
    ap.add_argument('--info-topic', default='/camera/camera/depth/camera_info')
    ap.add_argument('--max-range', type=float, default=3.0,
                    help='ignore points beyond this; depth noise grows with range^2')
    ap.add_argument('--step', type=int, default=4, help='pixel subsample stride')
    ap.add_argument('--iterations', type=int, default=300)
    ap.add_argument('--inlier-tol', type=float, default=0.03)
    args = ap.parse_args()

    rclpy.init()
    node = GroundPlaneCheck(args)
    rc = 0
    try:
        if not node.wait():
            print('ERROR: no depth/camera_info. Is camera.launch.py running?',
                  file=sys.stderr)
            return 1

        pts = node.points_in_base()
        if pts is None:
            return 1

        fit = node.fit_plane(pts)
        if fit is not None:
            frac = fit['n_inliers'] / fit['n_total']
            cz = abs(fit['centroid'][2])
            bad = []
            if frac < 0.30:
                bad.append(f'only {frac*100:.0f}% of points are inliers (want >30%)')
            if cz > 0.30:
                bad.append(f'patch centroid is {cz*1000:.0f} mm off the ground plane')
            if bad:
                print('RESULT INVALID - the dominant plane is not the floor:')
                for b in bad:
                    print(f'  * {b}')
                print()
                print('Point the robot at open, flat floor with no furniture or wall')
                print('inside ~3 m and try again.')
                return 1
        if fit is None:
            print('ERROR: no dominant near-horizontal plane found. Is the robot '
                  'on flat floor with clear ground ahead?', file=sys.stderr)
            return 1

        n, cen = fit['n'], fit['centroid']
        # Height of the plane directly under the robot centre.
        height = -fit['d'] / n[2]
        tilt = math.degrees(math.acos(min(1.0, abs(n[2]))))
        # Decompose the tilt: pitch is about y, roll about x.
        pitch_err = math.degrees(math.atan2(n[0], n[2]))
        roll_err = math.degrees(math.atan2(-n[1], n[2]))

        print('Ground-plane check')
        print(f'  points used     : {fit["n_inliers"]} inliers of {fit["n_total"]}')
        print(f'  patch centroid  : x={cen[0]:+.2f} y={cen[1]:+.2f} z={cen[2]:+.3f} m')
        print()
        print('  DEPTH QUALITY')
        print(f'    RMS about the fitted plane : {fit["rms"]*1000:6.1f} mm')
        print('      < 10 mm  excellent, factory depth calibration is fine')
        print('      10-25 mm normal at these ranges')
        print('      > 25 mm  suspect - consider the depth-quality tool')
        print()
        print('  CAMERA MOUNT POSE (errors, not absolutes)')
        print(f'    floor height in base_footprint : {height*1000:+7.1f} mm   (want 0)')
        print(f'    plane tilt                     : {tilt:7.2f} deg (want 0)')
        print(f'      of which pitch error         : {pitch_err:+7.2f} deg')
        print(f'      of which roll error          : {roll_err:+7.2f} deg')
        print()
        print('  How to read this: the floor IS flat and IS at z=0. Any offset or')
        print('  tilt here is the camera mount pose in the URDF being wrong by')
        print('  that amount, so correct camera_z / camera_pitch by these values.')
        print()
        for r in (1.0, 2.0, 4.0):
            print(f'    at {r:.0f} m this tilt displaces a point by '
                  f'{r*math.sin(math.radians(tilt))*1000:5.1f} mm')
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
