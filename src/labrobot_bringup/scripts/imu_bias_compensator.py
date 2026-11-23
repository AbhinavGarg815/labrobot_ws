#!/usr/bin/env python3
"""Estimate and remove the gyro bias, then republish the IMU.

    ros2 run labrobot_bringup imu_bias_compensator.py
"""

import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class BiasCompensator(Node):
    def __init__(self, args):
        super().__init__('imu_bias_compensator')
        self.args = args
        self.samples = []
        self.bias = None
        self.published = 0

        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Imu, args.input_topic, self._on_imu, qos)
        self.pub = self.create_publisher(Imu, args.output_topic, qos)

        self.out = Imu()
        var = args.gyro_variance
        self.out.angular_velocity_covariance = [var, 0.0, 0.0,
                                                0.0, var, 0.0,
                                                0.0, 0.0, var]

        self.get_logger().info(
            f'estimating gyro bias from {args.calibration_seconds:.0f}s of '
            f'{args.input_topic} - KEEP THE ROBOT STILL')
        self.start = self.get_clock().now()

    def _on_imu(self, msg):
        if self.bias is None:
            g = msg.angular_velocity
            self.samples.append([g.x, g.y, g.z])
            elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
            if elapsed >= self.args.calibration_seconds:
                self._finish_calibration()
            return

        # angular_velocity_covariance was set once in __init__ and never changes.
        out = self.out
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.angular_velocity.x = msg.angular_velocity.x - self.bias[0]
        out.angular_velocity.y = msg.angular_velocity.y - self.bias[1]
        out.angular_velocity.z = msg.angular_velocity.z - self.bias[2]
        self.pub.publish(out)
        self.published += 1

    def _finish_calibration(self):
        arr = np.array(self.samples)
        self.bias = arr.mean(axis=0)
        noise = arr.std(axis=0)
        self.get_logger().info(
            f'bias from {len(arr)} samples: '
            f'x={math.degrees(self.bias[0]):+.4f} '
            f'y={math.degrees(self.bias[1]):+.4f} '
            f'z={math.degrees(self.bias[2]):+.4f} deg/s')
        self.get_logger().info(
            f'noise: x={math.degrees(noise[0]):.4f} y={math.degrees(noise[1]):.4f} '
            f'z={math.degrees(noise[2]):.4f} deg/s')
        drift = math.degrees(abs(self.bias[2])) * 60
        self.get_logger().info(
            f'uncorrected this yaw bias would drift {drift:.1f} deg/minute; '
            f'now removed')
        if math.degrees(abs(self.bias[2])) > self.args.max_plausible_bias:
            self.get_logger().error(
                f'measured yaw bias {math.degrees(self.bias[2]):+.3f} deg/s exceeds '
                f'{self.args.max_plausible_bias} deg/s - was the robot moving? '
                'Restart this node with the robot stationary.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input-topic', default='/camera/camera/imu')
    ap.add_argument('--output-topic', default='/imu/unbiased')
    ap.add_argument('--calibration-seconds', type=float, default=10.0)
    # Measured: 0.098-0.140 deg/s per axis -> 5.9e-6 (rad/s)^2.
    ap.add_argument('--gyro-variance', type=float, default=8.0e-6)
    ap.add_argument('--max-plausible-bias', type=float, default=2.0,
                    help='deg/s; above this the robot was probably moving')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = BiasCompensator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
