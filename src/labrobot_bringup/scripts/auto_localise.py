#!/usr/bin/env python3
"""Localise automatically at startup: spin in place until the map pose is trusted.

    ros2 run labrobot_bringup auto_localise.py            # run once, then exit
    ros2 run labrobot_bringup auto_localise.py --turns 1  # give up sooner

Started by scripts/bringup_nav.sh after Nav2 is active. Publishes, latched:
    /labrobot/localised            std_msgs/Bool    true only on success
    /labrobot/localisation_status  std_msgs/String  human-readable, for Foxglove
nav_goal.py refuses to send goals unless /labrobot/localised is true.

THE ROBOT SPINS AT EVERY STARTUP: leave ~0.35 m clear all round (circumscribed
radius 0.29 m). The spin is one slow continuous turn using Nav2's Spin behaviour, which collision-checks each
step against the local costmap and aborts rather than turning into something -
but the camera faces forward only, so it cannot see what is beside the robot
until it has turned towards it.

Why a spin
----------
At startup RTAB-Map assumes the robot is where it last shut down. If it was
moved while off, that pose is wrong, and RTAB-Map must recognise the room again.
From one heading it often cannot (2026-10-13: a cluttered view of moved
furniture gave 30 candidates, 23 with exactly 0/20 inliers). Turning shows it
walls, shelving and other structure in every direction.

When is it "localised" - the threshold, for a changing room
-----------------------------------------------------------
All of:
  1. at least --min-closures RTAB-Map loop closures to nodes that EXISTED
     BEFORE this run started. Closures among the nodes created during the spin
     itself prove nothing about the saved map, and in SLAM mode they happen
     naturally as the robot turns back over its own views.
  2. those closures agree: the map->odom correction after each is within
     --agree-m and --agree-deg of the others. One closure on a chair that has
     moved can be wrong; two independent ones pointing at the same pose are
     very unlikely to be wrong together.
  3. localisation_check does not say WRONG (gross-error guard; see that file).
Gives up after --turns full turns and reports NOT localised rather than guess.

Why localisation mode first, then mapping (--then-mapping)
---------------------------------------------------------
Tried 2026-10-13: running in SLAM mode from the start, RTAB-Map's best
loop-closure hypothesis was always one of the CURRENT session's own nodes
(hypothesis 0.93-0.98 to views taken minutes earlier in the same spin), so
the saved map's places were never even tested and the new session stayed
disconnected - it had to be discarded. In localisation mode only the saved map
competes. So: localise against the saved map first, then switch RTAB-Map to
mapping mode (/rtabmap/set_mode_mapping), and the map updates during navigation
already joined to the saved one.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
import tf2_ros
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import Spin
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import Info
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

import localisation_check


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class AutoLocalise(Node):
    def __init__(self):
        super().__init__('auto_localise')
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_ok = self.create_publisher(Bool, '/labrobot/localised', latched)
        self.pub_status = self.create_publisher(String, '/labrobot/localisation_status', latched)
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.first_id = None      # first node id RTAB-Map reports after we start
        self.pending = []         # old-map closure ids waiting for TF to settle
        self.corrections = []     # (closure id, x, y, yaw) of map->odom after each
        self.create_subscription(Info, '/info', self.on_info, 10)

    def status(self, ok, text):
        self.pub_ok.publish(Bool(data=ok))
        self.pub_status.publish(String(data=text))
        print(f'  [{"LOCALISED" if ok else "not localised"}] {text}', flush=True)

    def on_info(self, m):
        if self.first_id is None and m.ref_id > 0:
            self.first_id = m.ref_id
        if (m.loop_closure_id and self.first_id is not None
                and m.loop_closure_id < self.first_id):
            self.pending.append((time.time(), m.loop_closure_id))

    def spin_for(self, s):
        t = time.time()
        while time.time() - t < s:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.record_pending()

    def record_pending(self):
        # Read map->odom a moment AFTER the closure, once RTAB-Map has applied it.
        now = time.time()
        keep = []
        for t, cid in self.pending:
            if now - t < 1.5:
                keep.append((t, cid)); continue
            try:
                tf = self.buf.lookup_transform('map', 'odom', rclpy.time.Time())
                p = tf.transform.translation
                self.corrections.append((cid, p.x, p.y, yaw_of(tf.transform.rotation)))
                print(f'    closure to saved node {cid}: map->odom '
                      f'x={p.x:+.2f} y={p.y:+.2f} yaw={math.degrees(yaw_of(tf.transform.rotation)):+.1f}',
                      flush=True)
            except Exception:  # noqa: BLE001
                keep.append((t, cid))
        self.pending = keep

    def agreeing(self, agree_m, agree_deg):
        """Largest set of closures whose corrections all agree with its newest member."""
        best = []
        for _, x, y, th in self.corrections:
            grp = [c for c in self.corrections
                   if math.hypot(c[1]-x, c[2]-y) <= agree_m
                   and abs(math.degrees(math.atan2(math.sin(c[3]-th), math.cos(c[3]-th)))) <= agree_deg]
            if len(grp) > len(best):
                best = grp
        return best

    def spin_step(self, deg):
        g = Spin.Goal(); g.target_yaw = math.radians(deg)
        g.time_allowance = Duration(sec=30)
        f = self.spin_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        h = f.result()
        if h is None or not h.accepted:
            return False
        r = h.get_result_async()
        while not r.done():
            rclpy.spin_once(self, timeout_sec=0.05); self.record_pending()
        return r.result().status == GoalStatus.STATUS_SUCCEEDED

    def spin_continuous(self, deg, allowance_s, stop_when):
        """One slow continuous Spin of `deg`, cancelled as soon as stop_when()
        is true. Returns 'stopped', 'succeeded' or 'aborted'.

        Continuous rather than 45 deg steps (user observation 2026-10-14:
        wheels slipped). Each step started and stopped the turn, and the starts
        are where the drive wheels lose grip on this castor chassis. One ramp-up
        and a steady 0.15 rad/s sweep also gives RTAB-Map (1 Hz) a view every
        ~9 deg instead of a few widely spaced still frames."""
        g = Spin.Goal(); g.target_yaw = math.radians(deg)
        g.time_allowance = Duration(sec=int(allowance_s))
        fb = {'deg': 0.0}
        def on_fb(m):
            fb['deg'] = math.degrees(m.feedback.angular_distance_traveled)
        f = self.spin_client.send_goal_async(g, feedback_callback=on_fb)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        h = f.result()
        if h is None or not h.accepted:
            return 'aborted'
        r = h.get_result_async(); last = -45.0
        while not r.done():
            rclpy.spin_once(self, timeout_sec=0.05); self.record_pending()
            if fb['deg'] - last >= 45.0:
                last = fb['deg']
                print(f'  {fb["deg"]:4.0f} deg turned: {len(self.corrections)} closures to the '
                      f'saved map', flush=True)
            if stop_when(fb['deg']):
                c = h.cancel_goal_async()
                rclpy.spin_until_future_complete(self, c, timeout_sec=5)
                while not r.done():
                    rclpy.spin_once(self, timeout_sec=0.05)
                return 'stopped'
        return 'succeeded' if r.result().status == GoalStatus.STATUS_SUCCEEDED else 'aborted'

    def set_rtabmap_param(self, name, value):
        """Set an RTAB-Map parameter on the running node and apply it.
        RTAB-Map parameters are declared as STRINGS on the ROS side."""
        cli = self.create_client(SetParameters, '/rtabmap/set_parameters')
        if not cli.wait_for_service(timeout_sec=10.0):
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=str(value)))]
        f = cli.call_async(req); rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        if f.result() is None or not f.result().results[0].successful:
            return False
        up = self.create_client(Empty, '/rtabmap/update_parameters')
        if not up.wait_for_service(timeout_sec=10.0):
            return False
        g = up.call_async(Empty.Request()); rclpy.spin_until_future_complete(self, g, timeout_sec=10.0)
        return g.result() is not None

    def set_mapping_mode(self):
        cli = self.create_client(Empty, '/rtabmap/set_mode_mapping')
        if not cli.wait_for_service(timeout_sec=10.0):
            return False
        f = cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        return f.result() is not None

    def clear_costmaps(self):
        for name in ('local_costmap/clear_entirely_local_costmap',
                     'global_costmap/clear_entirely_global_costmap'):
            cli = self.create_client(ClearEntireCostmap, name)
            if cli.wait_for_service(timeout_sec=5.0):
                f = cli.call_async(ClearEntireCostmap.Request())
                rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--turns', type=float, default=2.0, help='full turns before giving up')
    ap.add_argument('--rate', type=float, default=0.15,
                    help='rad/s - must match behavior_server max_rotational_vel; only used '
                         'to size the time allowance')
    ap.add_argument('--saved-below', type=int, default=None,
                    help='node ids below this belong to the saved map: the FIRST node id of '
                         'the current RTAB-Map run (bringup_nav.sh reads it from the log). '
                         'Without it, the first id seen by THIS node is used - wrong if '
                         'RTAB-Map has been running a while, since nodes it already made '
                         'this session would count as the saved map.')
    ap.add_argument('--min-closures', type=int, default=2)
    ap.add_argument('--agree-m', type=float, default=0.15)
    ap.add_argument('--agree-deg', type=float, default=5.0)
    ap.add_argument('--localise-min-inliers', type=int, default=15,
                    help='Vis/MinInliers while localising (restored to --map-min-inliers after). '
                         'Measured 2026-10-14: the true place kept scoring 19, 12, 12 inliers '
                         'against the default 20, while wrong places scored 0-9. The agreement '
                         'rule (two closures to the same pose) is a stronger guard than the '
                         'count, so a lower count is safe for LOCALISING - but the map itself '
                         'is only ever built with the stricter value.')
    ap.add_argument('--map-min-inliers', type=int, default=20)
    ap.add_argument('--then-mapping', action='store_true',
                    help='after localising, switch RTAB-Map to mapping mode so the map keeps '
                         'updating during navigation (lifelong SLAM). Start RTAB-Map in '
                         'localisation mode for this - see the module docstring.')
    ap.add_argument('--once', action='store_true',
                    help='exit after deciding. By default the node stays up so its latched '
                         'status survives - latched messages vanish with their publisher.')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    # No rclpy signal handler: by default Ctrl-C shuts the ROS context down
    # BEFORE Python unwinds, so run()'s `finally` could not reach RTAB-Map to
    # restore Vis/MinInliers (it failed with "context is invalid" on
    # 2026-10-14, leaving RTAB-Map on the lowered value). Without the handler,
    # Ctrl-C is a plain KeyboardInterrupt and the restore still runs.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); n = AutoLocalise()
    try:
        rc = run(n, args)
    except KeyboardInterrupt:
        n.destroy_node(); rclpy.shutdown(); return 130
    if not args.once:
        try:
            while rclpy.ok():
                rclpy.spin_once(n, timeout_sec=0.5)
        except KeyboardInterrupt:
            pass
    n.destroy_node(); rclpy.shutdown()
    return rc


def run(n, args):
    lowered = n.set_rtabmap_param('Vis/MinInliers', args.localise_min_inliers)
    print(f'  Vis/MinInliers -> {args.localise_min_inliers} for localising'
          + ('' if lowered else ' FAILED (keeping the configured value)'), flush=True)
    try:
        return _run(n, args)
    finally:
        # Always restore before anything maps with it - including on failure.
        ok = n.set_rtabmap_param('Vis/MinInliers', args.map_min_inliers)
        print(f'  Vis/MinInliers restored to {args.map_min_inliers}'
              + ('' if ok else ' - FAILED, check /rtabmap parameters!'), flush=True)


def _run(n, args):
    n.status(False, 'starting: waiting for RTAB-Map and the Spin behaviour')
    if not n.spin_client.wait_for_server(timeout_sec=60):
        n.status(False, 'Nav2 Spin behaviour not available - is navigation running?'); return 1
    if args.saved_below:
        n.first_id = args.saved_below
    t0 = time.time()
    while n.first_id is None and time.time() - t0 < 30:
        n.spin_for(0.5)
    if n.first_id is None:
        n.status(False, 'no /info from RTAB-Map - is it running?'); return 1
    print(f'  nodes before {n.first_id} belong to the saved map', flush=True)

    state = {'checked_at': 0, 'result': None}

    def localised(_deg):
        grp = n.agreeing(args.agree_m, args.agree_deg)
        # Run the (few-second) gross-error check only when the agreeing set grows.
        if len(grp) >= args.min_closures and len(grp) > state['checked_at']:
            state['checked_at'] = len(grp)
            r = localisation_check.check(n, n.buf)
            if r['verdict'] == 'WRONG':
                n.status(False, f'{len(grp)} agreeing closures but the check says WRONG '
                                f'({r["why"]}) - continuing')
                return False
            state['result'] = (grp, r)
            return True
        return False

    total = args.turns * 360.0
    allowance = total / math.degrees(args.rate) + 30.0
    outcome = n.spin_continuous(total, allowance, localised)
    n.spin_for(1.0)
    if outcome == 'aborted' and state['result'] is None:
        n.status(False, 'NOT localised: the spin was refused/aborted (obstacle within ~0.3 m?) - '
                        'clear around the robot or move it, then restart auto_localise')
        return 1
    if state['result'] is None:
        n.status(False, f'NOT localised after {args.turns:g} turns: {len(n.corrections)} closures '
                        f'to the saved map, at most {len(n.agreeing(args.agree_m, args.agree_deg))} '
                        'agreeing. Point the robot at walls/shelving and restart auto_localise.')
        return 1
    grp, r = state['result']
    n.clear_costmaps()
    # Restore the strict threshold BEFORE mapping starts using it.
    n.set_rtabmap_param('Vis/MinInliers', args.map_min_inliers)
    if args.then_mapping:
        if n.set_mapping_mode():
            print('  RTAB-Map switched to MAPPING mode: the map now updates while '
                  'navigating, joined to the saved map', flush=True)
        else:
            print('  WARNING: could not switch RTAB-Map to mapping mode - it stays in '
                  'localisation mode (map frozen)', flush=True)
    n.status(True, f'{len(grp)} agreeing closures to the saved map; check {r["verdict"]}: {r["why"]}')
    return 0

if __name__ == '__main__':
    sys.exit(main())
