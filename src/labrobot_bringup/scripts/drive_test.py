#!/usr/bin/env python3
"""Drive-test and odometry-calibration tool for the labrobot."""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a: float, b: float) -> float:
    """Shortest signed difference a - b, wrapped to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class DriveTest(Node):
    CMD_TOPIC = '/diff_drive_controller/cmd_vel_unstamped'
    # The EKF, NOT /diff_drive_controller/odom.
    ODOM_TOPIC = '/odometry/filtered'

    def __init__(self, args):
        super().__init__('labrobot_drive_test')
        self.args = args
        self.pub = self.create_publisher(Twist, self.CMD_TOPIC, 10)
        if getattr(args, 'odom_topic', None):
            self.ODOM_TOPIC = args.odom_topic
        # diff_drive_controller publishes BEST_EFFORT and robot_localization RELIABLE.
        self.sub = self.create_subscription(
            Odometry, self.ODOM_TOPIC, self._on_odom,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.odom = None
        self.have_odom = False
        # How far the robot kept going after the loop stopped commanding.
        self.last_coast = 0.0

    def _on_odom(self, msg):
        self.odom = msg
        self.have_odom = True

    # primitives

    def spin_once(self, timeout=0.05):
        rclpy.spin_once(self, timeout_sec=timeout)

    def wait_for_odom(self, timeout=10.0):
        t0 = self.get_clock().now()
        while rclpy.ok() and not self.have_odom:
            self.spin_once()
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                return False
        return True

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def publish(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.pub.publish(msg)

    def stop(self, settle=0.6):
        """Command zero and hold it, so the controller ramps down properly."""
        t0 = self.get_clock().now()
        while (self.get_clock().now() - t0).nanoseconds < settle * 1e9:
            self.publish(0.0, 0.0)
            self.spin_once(0.02)
        self.publish(0.0, 0.0)

    # Slowing down before the target is not cosmetic.
    RAMP_ZONE_M = 0.15      # start easing off this far out
    CREEP_MPS = 0.02        # slow enough that the latency term is ~10 mm
    RAMP_ZONE_RAD = 0.35    # ~20 deg
    CREEP_RADPS = 0.08      # slow enough that the latency term is ~2 deg

    @staticmethod
    def _approach(remaining, zone, full, creep):
        """Full speed until `zone` from the target, then ease down to `creep`."""
        if remaining >= zone:
            return full
        return max(creep, full * remaining / zone)

    def drive_straight(self, distance, speed):
        """Drives until odometry says `distance` has been covered."""
        x0, y0, _ = self.pose()
        sign = 1.0 if distance >= 0 else -1.0
        target = abs(distance)
        # Generous time limit: allows for the creep phase and a stalled robot.
        limit_s = target / max(abs(speed), 1e-3) * 3.0 + target / self.CREEP_MPS + 5.0
        t0 = self.get_clock().now()

        while rclpy.ok():
            self.spin_once(0.02)
            x, y, _ = self.pose()
            travelled = math.hypot(x - x0, y - y0)
            if travelled >= target:
                break
            if (self.get_clock().now() - t0).nanoseconds > limit_s * 1e9:
                self.get_logger().warn(
                    f'straight leg timed out at {travelled:.3f} of {target:.3f} m')
                break
            v = self._approach(target - travelled, self.RAMP_ZONE_M,
                               abs(speed), self.CREEP_MPS)
            self.publish(sign * v, 0.0)

        x, y, _ = self.pose()
        at_stop = math.hypot(x - x0, y - y0)
        self.stop()
        x, y, _ = self.pose()
        final = math.hypot(x - x0, y - y0)
        self.last_coast = final - at_stop
        return final

    def rotate(self, angle_rad, omega):
        """Rotates until odometry says `angle_rad` has been swept."""
        _, _, yaw0 = self.pose()
        sign = 1.0 if angle_rad >= 0 else -1.0
        target = abs(angle_rad)
        limit_s = (target / max(abs(omega), 1e-3) * 3.0
                   + target / self.CREEP_RADPS + 5.0)
        t0 = self.get_clock().now()
        swept = 0.0
        prev = yaw0

        while rclpy.ok():
            self.spin_once(0.02)
            _, _, yaw = self.pose()
            swept += abs(ang_diff(yaw, prev))
            prev = yaw
            if swept >= target:
                break
            if (self.get_clock().now() - t0).nanoseconds > limit_s * 1e9:
                self.get_logger().warn(
                    f'rotation timed out at {math.degrees(swept):.1f} of '
                    f'{math.degrees(target):.1f} deg')
                break
            w = self._approach(target - swept, self.RAMP_ZONE_RAD,
                               abs(omega), self.CREEP_RADPS)
            self.publish(0.0, sign * w)

        _, _, yaw_at_stop = self.pose()
        self.stop()
        _, _, yaw_final = self.pose()
        coast = abs(ang_diff(yaw_final, yaw_at_stop))
        self.last_coast = coast
        return swept + coast


def run(node, args):
    if not node.wait_for_odom():
        print(f'ERROR: no messages on {DriveTest.ODOM_TOPIC}. Is the drivetrain up?',
              file=sys.stderr)
        return 1

    x0, y0, yaw0 = node.pose()
    print(f'start pose: x={x0:.4f} y={y0:.4f} yaw={math.degrees(yaw0):+.2f} deg')

    if args.straight is not None:
        print(f'\ndriving {args.straight:.3f} m at {args.speed:.3f} m/s ...')
        got = node.drive_straight(args.straight, args.speed)
        x, y, yaw = node.pose()
        # Resolve displacement in the START frame.
        dx, dy = x - x0, y - y0
        fwd = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        lat = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        print(f'  commanded : {abs(args.straight):.4f} m')
        print(f'  odometry  : {got:.4f} m   (of which {node.last_coast * 1000:+.0f} mm '
              f'coasted after the stop)')
        print(f'  forward   : {fwd:+.4f} m   (along the START heading)')
        print(f'  lateral   : {lat:+.4f} m   (perpendicular to it - real drift)')
        print(f'  heading   : {math.degrees(ang_diff(yaw, yaw0)):+.2f} deg')
        print('\nMeasure the REAL distance with a tape. odometry/real gives the '
              'wheel-radius scale error.')

    if args.rotate is not None:
        target = math.radians(args.rotate)
        print(f'\nrotating {args.rotate:.1f} deg at {args.omega:.2f} rad/s ...')
        got = node.rotate(target, args.omega)
        print(f'  commanded : {args.rotate:+.2f} deg')
        print(f'  odometry  : {math.degrees(got):+.2f} deg   '
              f'(of which {math.degrees(node.last_coast):+.2f} deg coasted after the stop)')
        print('\nCompare against the REAL angle turned. If the robot physically '
              'turns more than odometry reports, wheel_separation is too wide.')
        print('The coast figure is the tool overshooting, NOT a calibration '
              'error - subtract it before judging the separation.')

    if args.square is not None:
        side = args.square
        turn = -90.0 if args.direction == 'cw' else 90.0
        print(f'\nUMBmark {side:.2f} m square, {args.direction} ...')
        for leg in range(4):
            d = node.drive_straight(side, args.speed)
            print(f'  leg {leg + 1}: {d:.3f} m (coast {node.last_coast * 1000:+.0f} mm)')
            a = node.rotate(math.radians(turn), args.omega)
            print(f'  turn {leg + 1}: {math.degrees(a):+.2f} deg '
                  f'(coast {math.degrees(node.last_coast):+.2f} deg)')
        x, y, yaw = node.pose()
        print(f'\nodometry says it returned to: x={x - x0:+.4f} y={y - y0:+.4f} '
              f'yaw={math.degrees(ang_diff(yaw, yaw0)):+.2f} deg')
        print('Now measure the REAL final position relative to the start mark, '
              f'and record it as the {args.direction} result.')

    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--straight', type=float, help='metres to drive forward')
    ap.add_argument('--rotate', type=float, help='degrees to rotate in place')
    ap.add_argument('--square', type=float, help='UMBmark square side in metres')
    ap.add_argument('--direction', choices=['cw', 'ccw'], default='cw')
    ap.add_argument('--odom-topic', default=None,
                    help='override the feedback source; defaults to the EKF. Pass '
                         '/diff_drive_controller/odom to drive on raw wheel '
                         'odometry, which under-turns by 4-12% (Problem 25)')
    ap.add_argument('--speed', type=float, default=0.08, help='m/s')
    ap.add_argument('--omega', type=float, default=0.4, help='rad/s')
    args = ap.parse_args()

    if args.straight is None and args.rotate is None and args.square is None:
        ap.error('pick one of --straight, --rotate or --square')

    rclpy.init()
    node = DriveTest(args)
    try:
        rc = run(node, args)
    except KeyboardInterrupt:
        print('\ninterrupted - stopping')
        rc = 130
    finally:
        # Belt and braces: whatever happened above, leave the robot stopped.
        try:
            node.stop(settle=0.4)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
