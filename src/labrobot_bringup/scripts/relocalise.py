#!/usr/bin/env python3
"""Get the robot back to a trustworthy map pose after it has been moved by hand.

    ros2 run labrobot_bringup relocalise.py            # check only, never moves
    ros2 run labrobot_bringup relocalise.py --spin     # rotate in 45 deg steps
                                                       # until the pose checks out

Then clears both costmaps.

Why both steps
--------------
A hand move is invisible to the wheels and the gyro, so after one:
  1. the MAP pose is wrong until RTAB-Map recognises the surroundings - seen
     1.1-1.8 m out, which nav_goal.py now refuses to navigate on;
  2. the LOCAL costmap still holds obstacles marked before the move, drawn in
     odom coordinates that no longer correspond to the world. Seen 2026-10-09:
     a "lethal obstacle 72 cm ahead" when the camera saw nothing nearer than
     1.6 m.
Clearing is done only AFTER the pose is confirmed, so the costmaps refill
around a correct pose. The camera faces forward only, so anything beside or
behind the robot is forgotten by the clear until it is seen again - which is
why this is a deliberate post-hand-move step, not something done every goal.

--spin rotates in place: the robot needs ~0.3 m clear ALL ROUND (circumscribed
radius 0.29 m) and it cannot see its own sides - only use it when a person has
confirmed the space.
"""
import argparse
import math
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from rtabmap_msgs.msg import Info

import localisation_check


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Reloc(Node):
    def __init__(self):
        super().__init__('relocalise')
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.accepted = 0          # RTAB-Map loop closures / localisations accepted
        self.create_subscription(Info, '/info', self._on_info, 10)

    def _on_info(self, m):
        # GLOBAL loop closures only. A proximity detection is searched for
        # around RTAB-Map's CURRENT pose guess, so it cannot confirm that guess;
        # counting it once declared success while the log showed RTAB-Map
        # rejecting the very next localisations.
        if m.loop_closure_id:
            self.accepted += 1

    def spin_for(self, s):
        t = time.time()
        while time.time() - t < s:
            rclpy.spin_once(self, timeout_sec=0.05)

    def odom_yaw(self):
        return yaw_of(self.buf.lookup_transform('odom', 'base_footprint', rclpy.time.Time()).transform.rotation)

    def turn(self, deg, rate=0.3):
        prev = self.odom_yaw(); done = 0.0; t0 = time.time()
        while done < math.radians(deg) and time.time() - t0 < deg / math.degrees(rate) + 8:
            m = Twist(); m.angular.z = rate; self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)
            y = self.odom_yaw(); done += abs(math.atan2(math.sin(y - prev), math.cos(y - prev))); prev = y
        for _ in range(5):
            self.pub.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.05)
        return math.degrees(done)

    def clear(self):
        ok = True
        for name in ('local_costmap/clear_entirely_local_costmap',
                     'global_costmap/clear_entirely_global_costmap'):
            cli = self.create_client(ClearEntireCostmap, name)
            if not cli.wait_for_service(timeout_sec=5.0):
                print(f'  could not reach {name}'); ok = False; continue
            f = cli.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
            ok &= f.result() is not None
        return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--spin', action='store_true',
                    help='rotate in place between checks (needs ~0.3 m clear all round)')
    ap.add_argument('--settle', type=float, default=3.0,
                    help='seconds to hold still before each check (RTAB-Map runs at 1 Hz)')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(); n = Reloc(); n.spin_for(3.0)

    # Relocalised means BOTH: RTAB-Map has accepted a visual match since we
    # started, AND the independent far-structure check agrees with the pose.
    # Either alone has been wrong: RTAB-Map "localised" at startup by restoring
    # the last session's pose although the robot had been moved; the check
    # once passed on a 14-point margin that a 15-point threshold let through.
    steps = 9 if args.spin else 1          # 8 x 45 deg = one full turn, plus the start
    for k in range(steps):
        if k:
            print(f'  turned {n.turn(45):.0f} deg (step {k}/8)'); n.spin_for(args.settle)
        r = localisation_check.check(n, n.buf)
        print(f'  check {k}: {r["verdict"]} - {r["why"]}   [RTAB-Map accepted: {n.accepted}]')
        if r['verdict'] == 'OK' and n.accepted > 0:
            print('  costmaps cleared' if n.clear() else '  WARNING: costmap clear failed')
            print('  RELOCALISED - RTAB-Map matched and the check agrees; safe to send goals')
            n.destroy_node(); rclpy.shutdown(); return 0
        if r['verdict'] == 'OK':
            print('     (check agrees, but RTAB-Map has not confirmed with a match yet)')
            n.spin_for(args.settle)
            if n.accepted > 0:
                print('  costmaps cleared' if n.clear() else '  WARNING: costmap clear failed')
                print('  RELOCALISED - RTAB-Map matched and the check agrees; safe to send goals')
                n.destroy_node(); rclpy.shutdown(); return 0
    print('  NOT relocalised.' + ('' if args.spin else ' Re-run with --spin if there is '
          '~0.3 m clear all round, or point the robot at structure that was mapped.'))
    n.destroy_node(); rclpy.shutdown(); return 1


if __name__ == '__main__':
    sys.exit(main())
