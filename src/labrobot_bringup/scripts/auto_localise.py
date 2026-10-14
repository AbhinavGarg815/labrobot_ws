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

When is it "localised" - geometry first, for a kidnapped robot in a changing room
--------------------------------------------------------------------------------
1. One slow 360 deg turn builds a depth ring; global_localise.match() searches
   the whole map for where the room's SHAPE fits (walls and fixed structure do
   not move; appearance does). See global_localise.py.
2. The match must be DISTINCT: the best genuinely different place scores
   < 80% of the best. If not (symmetric spot, too much moved), the robot drives
   --move metres towards the most open direction (forward only, collision-
   checked) and turns again, up to --max-spots viewpoints.
3. Whole-room confirmation: the ring must fit the map (score >= 0.35), and any
   RTAB-Map visual closures made during the turn must AGREE with the geometric
   pose (0.3 m / 10 deg) - if they disagree it refuses. Then the pose goes to
   RTAB-Map on /initialpose, which must adopt it. (The single-view check is
   logged only - see _run for why it cannot veto whole-room evidence.)
Only then does it map. Appearance-only relocalisation (the earlier design: two
agreeing RTAB-Map closures) is kept as a logged bonus - a week after mapping it
produced no accepted closure at all (2026-10-14).

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

import numpy as np

import rclpy
from rclpy.signals import SignalHandlerOptions
import tf2_ros
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import DriveOnHeading, Spin
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import Info
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

import global_localise as gl
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
        self.drive_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.first_id = None      # first node id RTAB-Map reports after we start
        self.pending = []         # old-map closure ids waiting for TF to settle
        self.corrections = []     # (closure id, x, y, yaw) of map->odom after each
        self.closure_poses = []   # (closure id, x, y, yaw) of the ROBOT in the map, per closure
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
                ob = self.buf.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                p = tf.transform.translation
                self.corrections.append((cid, p.x, p.y, yaw_of(tf.transform.rotation)))
                # Implied robot pose in the map: map->odom composed with odom->base.
                a = yaw_of(tf.transform.rotation); c, s_ = math.cos(a), math.sin(a)
                bx, by = ob.transform.translation.x, ob.transform.translation.y
                self.closure_poses.append((cid, p.x + c*bx - s_*by, p.y + s_*bx + c*by,
                                           a + yaw_of(ob.transform.rotation)))
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

    def run_action(self, client, goal):
        f = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        h = f.result()
        if h is None or not h.accepted:
            return False
        r = h.get_result_async()
        while not r.done():
            rclpy.spin_once(self, timeout_sec=0.05); self.record_pending()
        return r.result().status == GoalStatus.STATUS_SUCCEEDED

    def turn_by(self, rad, allowance=60):
        g = Spin.Goal(); g.target_yaw = float(rad); g.time_allowance = Duration(sec=allowance)
        return self.run_action(self.spin_client, g)

    def drive_forward(self, metres, speed=0.05):
        # Forward only: the camera faces forward, so it sees where it drives,
        # and the behaviour server collision-checks every step.
        g = DriveOnHeading.Goal(); g.target.x = float(metres); g.speed = float(speed)
        g.time_allowance = Duration(sec=int(metres / speed) + 20)
        return self.run_action(self.drive_client, g)

    def hint(self, x, y, yaw):
        """Give RTAB-Map the geometric pose (the standard "2D Pose Estimate").
        Replaces its stale startup pose and clears its odometry cache."""
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = math.radians(3) ** 2
        for _ in range(3):
            self.init_pub.publish(m); self.spin_for(0.3)

    def map_pose(self):
        tf = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        p = tf.transform.translation
        return p.x, p.y, yaw_of(tf.transform.rotation)

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


def most_open_bearing(ring, half_width=0.30, step_deg=10):
    """Bearing (rad, robot frame) with the longest clear corridor of robot
    width, from the ring the spin just produced. Returns (bearing, clear_m)."""
    best = (0.0, 0.0)
    for b in np.radians(np.arange(-180, 180, step_deg)):
        c, s = math.cos(b), math.sin(b)
        fx = c * ring[:, 0] + s * ring[:, 1]
        fy = -s * ring[:, 0] + c * ring[:, 1]
        inside = (fx > 0) & (np.abs(fy) < half_width)
        clear = float(fx[inside].min()) if inside.any() else gl.R_MAX
        if clear > best[1]:
            best = (float(b), clear)
    return best


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
    ap.add_argument('--max-spots', type=int, default=3,
                    help='viewpoints to try: if the geometric match is ambiguous, drive to '
                         'the most open direction and spin again')
    ap.add_argument('--move', type=float, default=0.6, help='metres to drive between spots')
    ap.add_argument('--min-turn', type=float, default=240.0,
                    help='degrees of turn needed to attempt a match (a partial ring is fine '
                         'if it covers most of the room)')
    ap.add_argument('--min-ring-score', type=float, default=0.35,
                    help='mean likelihood of the 360 deg ring at the matched pose (good '
                         'matches on hardware: 0.57; the smeared first version: 0.20)')
    ap.add_argument('--match-png', default='docs/logs/autoloc_match.png')
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
    lowered = False
    for _ in range(6):                    # RTAB-Map's parameter service can lag startup
        lowered = n.set_rtabmap_param('Vis/MinInliers', args.localise_min_inliers)
        if lowered:
            break
        n.spin_for(5.0)
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
    """Geometry first (works when the robot was kidnapped and the room has
    changed), RTAB-Map second. See global_localise.py for the matcher."""
    n.status(False, 'starting: waiting for RTAB-Map, the map and the Spin behaviour')
    if not n.spin_client.wait_for_server(timeout_sec=60):
        n.status(False, 'Nav2 Spin behaviour not available - is navigation running?'); return 1
    if args.saved_below:
        n.first_id = args.saved_below
    grid = gl.get_map(n, timeout=30)
    if grid is None:
        n.status(False, 'no /map from RTAB-Map - is it running?'); return 1
    col = gl.RingCollector(n, n.buf)
    n.spin_for(2.0)

    out = None
    for spot in range(args.max_spots):
        col.pts, col.last_yaw, col.turned, col.frames = [], None, 0.0, 0
        n.status(False, f'spot {spot + 1}/{args.max_spots}: turning 360 deg to see the room')
        # Generous allowance: the robot can turn far slower than commanded -
        # 0.045 rad/s against 0.15 at one spot on 2026-10-14 (wheel slip) - and
        # a slow, steady turn still builds a valid ring. Running out of time is
        # not a reason to give up if most of the room has been seen.
        allowance = 240
        n.spin_continuous(360.0, allowance, lambda _d: False)
        n.spin_for(1.0)
        if math.degrees(col.turned) < args.min_turn:
            n.status(False, f'spot {spot + 1}: only turned {math.degrees(col.turned):.0f} deg '
                            '(obstacle within ~0.3 m?) - cannot see enough of the room')
            return 1
        ring = col.ring_in_robot_frame(grid.info.resolution)
        out = gl.match(ring, grid)
        if 'pose' not in out:
            print(f'  spot {spot + 1}: {out["why"]}', flush=True)
        else:
            x, y, th = out['pose']
            print(f'  spot {spot + 1}: {len(ring)} beams, best x={x:+.2f} y={y:+.2f} '
                  f'yaw={math.degrees(th):+.0f}, runner-up {out["ratio"]:.0%} -> '
                  f'{"DISTINCT" if out["ok"] else "ambiguous"}', flush=True)
            if out['ok']:
                break
        if spot + 1 < args.max_spots:
            b, clear = most_open_bearing(ring)
            if clear < args.move + 0.5:
                n.status(False, f'ambiguous here and no open direction to move '
                                f'(best {clear:.1f} m) - move the robot to open space')
                return 1
            print(f'  ambiguous: moving {args.move:.1f} m towards the most open direction '
                  f'({math.degrees(b):+.0f} deg, {clear:.1f} m clear) - ROBOT DRIVES', flush=True)
            if not (n.turn_by(b) and n.drive_forward(args.move)):
                n.status(False, 'could not reach the next viewpoint (blocked)'); return 1
            n.spin_for(1.0)

    if out is None or not out.get('ok'):
        n.status(False, f'NOT localised: the room geometry stayed ambiguous after '
                        f'{args.max_spots} viewpoints')
        return 1

    x, y, th = out['pose']
    gl.save_debug_png(args.match_png, grid, ring, out['pose'])
    print(f'  picture of the match: {args.match_png}', flush=True)

    # Final gate - WHOLE-ROOM evidence only.
    # 1. The 360 deg ring must fit the map well at the matched pose.
    if out['score'] < args.min_ring_score:
        n.status(False, f'ring fits the map poorly (score {out["score"]:.2f} < '
                        f'{args.min_ring_score}) - not trusting it'); return 1
    # 2. Visual (RTAB-Map) closures made BEFORE the hint are independent
    #    evidence. If there are any, they must agree; a disagreement is a real
    #    conflict between two methods and nothing is mapped.
    before = list(n.closure_poses)
    agree = [c for c in before if math.hypot(c[1]-x, c[2]-y) <= 0.30
             and abs(math.degrees(math.atan2(math.sin(c[3]-th), math.cos(c[3]-th)))) <= 10]
    if before and not agree:
        n.status(False, f'CONFLICT: {len(before)} visual closures disagree with the geometric pose '
                        f'(nearest {min(math.hypot(c[1]-x, c[2]-y) for c in before):.2f} m) - '
                        'not mapping'); return 1
    # The single-view check is logged but does NOT gate here: on 2026-10-14 it
    # called a pose wrong (1% near obstacles) that the 360 deg geometric match
    # and three independent RTAB-Map closures agreed on within 12 cm - its view
    # held only a chair and a person, with the fixed structure beyond its 2.5 m
    # range. It still guards nav_goal.py, for a robot carried after startup.

    # Hand the geometric pose to RTAB-Map, then confirm it took it.
    n.hint(x, y, th)
    n.spin_for(4.0)
    try:
        mx, my, mth = n.map_pose()
    except Exception:  # noqa: BLE001
        n.status(False, 'no map->base_footprint after the hint'); return 1
    dpos = math.hypot(mx - x, my - y)
    dyaw = abs(math.degrees(math.atan2(math.sin(mth - th), math.cos(mth - th))))
    if dpos > 0.30 or dyaw > 10:
        n.status(False, f'RTAB-Map did not adopt the geometric pose ({dpos:.2f} m / {dyaw:.0f} deg off)')
        return 1
    r = localisation_check.check(n, n.buf)
    print(f'  single-view check (logged, not gating): {r["verdict"]} - {r["why"]}', flush=True)

    n.clear_costmaps()
    n.set_rtabmap_param('Vis/MinInliers', args.map_min_inliers)    # strict before mapping
    if args.then_mapping:
        if n.set_mapping_mode():
            print('  RTAB-Map switched to MAPPING mode: the map now updates while '
                  'navigating, joined to the saved map', flush=True)
        else:
            print('  WARNING: could not switch RTAB-Map to mapping mode - it stays in '
                  'localisation mode (map frozen)', flush=True)
    n.status(True, f'geometric match x={x:+.2f} y={y:+.2f} yaw={math.degrees(th):+.0f} deg '
                   f'(runner-up {out["ratio"]:.0%}, ring score {out["score"]:.2f}); '
                   f'{len(agree)}/{len(before)} independent visual closures agree; RTAB-Map adopted it')
    return 0

if __name__ == '__main__':
    sys.exit(main())
