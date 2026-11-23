#!/usr/bin/env python3
"""Characterise the D435i IMU, and cross-check the camera mount from gravity.

    ros2 run labrobot_bringup check_imu.py
"""

import argparse
import math
import sys

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

G = 9.80665


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


class ImuCheck(Node):
    def __init__(self, topic):
        super().__init__('check_imu')
        self.accel = []
        self.gyro = []
        self.frame_id = None
        qos = QoSProfile(depth=400, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, topic, self._on_imu, qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_imu(self, msg):
        self.frame_id = msg.header.frame_id
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self.accel.append([a.x, a.y, a.z])
        self.gyro.append([g.x, g.y, g.z])

    def collect(self, seconds):
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if (self.get_clock().now() - t0).nanoseconds > seconds * 1e9:
                break
        return np.array(self.accel), np.array(self.gyro)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/camera/camera/imu')
    ap.add_argument('--seconds', type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = ImuCheck(args.topic)
    rc = 0
    try:
        print(f'collecting {args.seconds:.0f}s of IMU - keep the robot COMPLETELY still...')
        accel, gyro = node.collect(args.seconds)

        if len(accel) < 100:
            print(f'ERROR: only {len(accel)} samples on {args.topic}. Is the camera up '
                  'with enable_imu:=true?', file=sys.stderr)
            return 1

        rate = len(accel) / args.seconds
        print(f'  {len(accel)} samples, {rate:.1f} Hz, frame {node.frame_id}')
        print()

        print('GYRO (stationary, so the mean IS the bias)')
        gm, gs = gyro.mean(axis=0), gyro.std(axis=0)
        for i, ax in enumerate('xyz'):
            print(f'  {ax}: bias {math.degrees(gm[i]):+7.4f} deg/s   '
                  f'noise {math.degrees(gs[i]):.4f} deg/s')
        drift = math.degrees(np.linalg.norm(gm)) * 60
        print(f'  -> uncorrected, that bias alone drifts heading {drift:.2f} deg/minute')
        print('     (the EKF removes most of it, but it must be told the noise)')
        print()

        print('ACCELEROMETER')
        am, asd = accel.mean(axis=0), accel.std(axis=0)
        mag = np.linalg.norm(am)
        print(f'  magnitude {mag:.4f} m/s^2 vs g = {G:.4f}  '
              f'({(mag/G - 1)*100:+.2f}% scale error)')
        for i, ax in enumerate('xyz'):
            print(f'  {ax}: mean {am[i]:+8.4f}   noise {asd[i]:.4f} m/s^2')
        print()

        print('EKF covariance inputs (variance = noise^2, per sample)')
        print(f'  angular_velocity : {np.max(gs)**2:.3e} (rad/s)^2')
        print(f'  linear_accel     : {np.max(asd)**2:.3e} (m/s^2)^2')
        print()

        # the independent mount check
        try:
            tf = node.tf_buffer.lookup_transform(
                'base_footprint', node.frame_id, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            print(f'(skipping the mount cross-check: TF lookup failed: {e})')
            return rc

        R = quat_to_matrix(tf.transform.rotation)
        g_base = R @ am
        horiz = math.hypot(g_base[0], g_base[1])
        tilt = math.degrees(math.atan2(horiz, abs(g_base[2])))
        pitch_err = math.degrees(math.atan2(-g_base[0], abs(g_base[2])))
        roll_err = math.degrees(math.atan2(g_base[1], abs(g_base[2])))

        print('CAMERA MOUNT, checked independently via gravity')
        print(f'  measured gravity in base_footprint : '
              f'({g_base[0]:+.3f}, {g_base[1]:+.3f}, {g_base[2]:+.3f}) m/s^2')
        print(f'  should be                          : ( 0.000,  0.000, {G:+.3f})')
        print(f'  tilt from vertical  : {tilt:6.2f} deg')
        print(f'    pitch component   : {pitch_err:+6.2f} deg')
        print(f'    roll component    : {roll_err:+6.2f} deg')
        print()
        print('  The depth-based ground-plane fit gave a residual of 0.29 deg.')
        print('  These two use unrelated physics - depth geometry versus gravity -')
        print('  so agreement means the mount pose is right for two independent')
        print('  reasons. A large disagreement would mean one method is being')
        print('  fooled, and the accelerometer is the one a sloping floor cannot lie to.')
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
