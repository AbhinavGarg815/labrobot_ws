#!/usr/bin/env python3
"""Estimate and remove the gyro bias, then republish the IMU.

    ros2 run labrobot_bringup imu_bias_compensator.py

Why this node exists
--------------------
The EKF needs a bias-free yaw rate, and nothing else in the stack provides one:

  * robot_localization's 15-state model is x y z / roll pitch yaw / velocities /
    accelerations. There are NO gyro-bias states, so it cannot estimate or remove
    a bias - it just integrates whatever it is given.
  * imu_filter_madgwick DOES estimate a bias, but only applies it internally to
    its ORIENTATION output; it republishes angular_velocity untouched. Measured
    on this robot: raw +0.1072 deg/s in, filtered +0.1072 deg/s out. 0% removed.
  * Fusing Madgwick's yaw orientation instead does not help either. With no
    magnetometer, yaw has no absolute reference and drifts at the bias rate.

Left uncorrected the measured +0.107 deg/s yaw bias makes the fused heading
rotate ~6.4 deg per minute while the robot is standing perfectly still - worse
than using wheel odometry alone, which is exactly backwards.

How it works
------------
On startup, while the robot is known to be stationary, average the gyro for a
few seconds and treat that mean as the bias. Then subtract it from every sample.

Re-estimating whenever the robot is stationary would also correct thermal drift,
but it is deliberately NOT done here: a robot that is "stationary" because it is
stuck against a wall while its wheels slip would poison the estimate with real
rotation. Startup-only is the honest, predictable choice, and the node reports
the value so drift can be spotted by re-running it.
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

        # Reuse ONE output message and set the constant covariance once, rather
        # than allocating a fresh Imu() per callback. Safe because publish()
        # serialises synchronously, so the message is fully consumed before the
        # next callback can touch it.
        #
        # HONEST NOTE ON THE PAYOFF: this was done to cut the node's cost and it
        # essentially did not. Measured 34.0% of a core before, 32.4% after -
        # inside the run-to-run noise. At 200 Hz the ~1.6 ms per message is
        # rclpy's callback and DDS overhead, not message construction, so no
        # amount of Python micro-optimisation here will move it. If this node's
        # cost ever actually matters, the fix is a C++ rewrite (expect ~2-3%),
        # not more of this. Kept because it is marginally leaner and no less
        # clear, but do not expect it to buy headroom.
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
        # Reporting the MEASURED noise there matters: the driver leaves it at
        # zero, which an EKF reads as "infinitely trustworthy", so the filter
        # would believe the gyro absolutely.
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
        # A large "bias" means the robot was moving during calibration, which
        # would bake real rotation into the correction and be worse than nothing.
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
    # Measured: 0.098-0.140 deg/s per axis -> 5.9e-6 (rad/s)^2. Rounded up.
    ap.add_argument('--gyro-variance', type=float, default=8.0e-6)
    ap.add_argument('--max-plausible-bias', type=float, default=2.0,
                    help='deg/s; above this the robot was probably moving')
    # parse_known_args, not parse_args: when launched by ros2 launch this node
    # is handed "--ros-args -r __node:=..." which argparse would reject with
    # exit code 2 - the node dies instantly and the only clue is a bare
    # "process has died" in the launch log.
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
