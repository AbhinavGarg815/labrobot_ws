#!/usr/bin/env python3
"""Named waypoints and waypoint missions.

    ros2 run labrobot_bringup waypoints.py save door          # robot's CURRENT pose
    ros2 run labrobot_bringup waypoints.py pick bench         # pose clicked in Foxglove
    ros2 run labrobot_bringup waypoints.py add desk 1.2 -0.5 90   # explicit map pose
    ros2 run labrobot_bringup waypoints.py list
    ros2 run labrobot_bringup waypoints.py delete door
    ros2 run labrobot_bringup waypoints.py go door bench shelf          # ROBOT MOVES
    ros2 run labrobot_bringup waypoints.py go door bench --return       # ... and back
    ros2 run labrobot_bringup waypoints.py go door bench --loop 3
"""
import argparse
import datetime
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap, SpeedLimit
from std_msgs.msg import String
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

FILE = os.environ.get('LABROBOT_WAYPOINTS',
                      os.path.expanduser('~/ros2_ws/docs/maps/waypoints.yaml'))


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def load():
    if not os.path.exists(FILE):
        return {}
    with open(FILE) as f:
        data = yaml.safe_load(f) or {}
    return data.get('waypoints', {}) or {}


def store(wps):
    os.makedirs(os.path.dirname(FILE), exist_ok=True)
    tmp = FILE + '.tmp'
    with open(tmp, 'w') as f:
        f.write('# labrobot named waypoints - map frame. Managed by waypoints.py.\n'
                '# Re-save after building a NEW map; lifelong-SLAM updates keep the frame.\n')
        yaml.safe_dump({'waypoints': wps}, f, sort_keys=True)
    os.replace(tmp, FILE)          # atomic: a crash mid-write cannot lose the file


def pose_msg(node, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class Tool(Node):
    def __init__(self):
        super().__init__('waypoints')
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)

    def robot_pose(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                p = tf.transform.translation
                return p.x, p.y, yaw_of(tf.transform.rotation)
            except Exception:  # noqa: BLE001
                continue
        return None


def cmd_save(tool, args, picked=None):
    wps = load()
    if args.name in wps and not args.force:
        print(f'  "{args.name}" exists - use --force to overwrite'); return 1
    pose = picked or tool.robot_pose()
    if pose is None:
        print('  no map -> base_footprint transform (is the robot localised?)'); return 1
    x, y, yaw = pose
    wps[args.name] = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(math.degrees(yaw), 1),
                      'saved': datetime.datetime.now().isoformat(timespec='seconds')}
    store(wps)
    print(f'  saved "{args.name}": x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):+.0f} deg -> {FILE}')
    return 0


def cmd_pick(tool, args):
    got = {}
    tool.create_subscription(PoseStamped, '/waypoint_pick', lambda m: got.__setitem__('p', m), 10)
    print('  in Foxglove: 3D panel -> "Publish pose", topic /waypoint_pick, click on the map '
          f'(waiting up to {args.timeout:.0f} s)', flush=True)
    t0 = time.time()
    while 'p' not in got and time.time() - t0 < args.timeout:
        rclpy.spin_once(tool, timeout_sec=0.2)
    if 'p' not in got:
        print('  nothing received on /waypoint_pick'); return 1
    m = got['p']
    frame = m.header.frame_id or 'map'
    x, y, yaw = m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)
    if frame != 'map':
        # Convert into the map frame with TF.
        try:
            tf = tool.buf.lookup_transform('map', frame, rclpy.time.Time(),
                                           timeout=rclpy.duration.Duration(seconds=3.0))
        except Exception as e:  # noqa: BLE001
            print(f'  cannot convert the click from "{frame}" to map ({e})'); return 1
        t = tf.transform.translation; th = yaw_of(tf.transform.rotation)
        c, s_ = math.cos(th), math.sin(th)
        x, y, yaw = t.x + c*x - s_*y, t.y + s_*x + c*y, th + yaw
        print(f'  click was in "{frame}" - converted to map', flush=True)
    return cmd_save(tool, args, picked=(x, y, yaw))


def cmd_add(tool, args):
    return cmd_save(tool, args, picked=(args.x, args.y, math.radians(args.yaw)))


def cmd_list(tool, args):
    wps = load()
    if not wps:
        print(f'  no waypoints in {FILE}'); return 0
    here = tool.robot_pose(timeout=3.0)
    print(f'  {len(wps)} waypoints in {FILE}:')
    for name, w in sorted(wps.items()):
        d = f'{math.hypot(w["x"]-here[0], w["y"]-here[1]):5.2f} m away' if here else ''
        print(f'    {name:16s} x={w["x"]:+6.2f} y={w["y"]:+6.2f} yaw={w["yaw_deg"]:+5.0f}  {d}')
    return 0


def cmd_delete(tool, args):
    wps = load()
    if args.name not in wps:
        print(f'  no waypoint "{args.name}"'); return 1
    del wps[args.name]; store(wps)
    print(f'  deleted "{args.name}"'); return 0


LETHAL = 253          # costmap: the robot CENTRE may not be on cells >= this
MAX_SHIFT_M = 0.50    # how far a blocked waypoint may be moved (user decision)


def feasible_target(costmap, robot_xy, x, y, max_shift=MAX_SHIFT_M):
    from scipy.ndimage import label
    md = costmap.metadata
    W, H, res = md.size_x, md.size_y, md.resolution
    ox, oy = md.origin.position.x, md.origin.position.y
    C = np.asarray(costmap.data, np.uint16).reshape(H, W)
    free = C < LETHAL
    lab, _ = label(free)
    rj, ri = int((robot_xy[1] - oy) / res), int((robot_xy[0] - ox) / res)
    if not (0 <= ri < W and 0 <= rj < H) or lab[rj, ri] == 0:
        return x, y, 0.0              # robot cell itself unknown here: let Nav2 judge
    reach = lab == lab[rj, ri]
    gj, gi = int((y - oy) / res), int((x - ox) / res)
    if 0 <= gi < W and 0 <= gj < H and reach[gj, gi]:
        return x, y, 0.0
    r = int(math.ceil(max_shift / res))
    j0, j1, i0, i1 = max(gj - r, 0), min(gj + r + 1, H), max(gi - r, 0), min(gi + r + 1, W)
    if j0 >= j1 or i0 >= i1:
        return None
    jj, ii = np.mgrid[j0:j1, i0:i1]
    cx, cy = ox + (ii + 0.5) * res, oy + (jj + 0.5) * res
    d = np.hypot(cx - x, cy - y)
    ok = reach[j0:j1, i0:i1] & (d <= max_shift)
    if not ok.any():
        return None
    # nearest, and among near-equal ones the lowest cost (further from obstacles)
    score = np.where(ok, d + 0.002 * C[j0:j1, i0:i1], np.inf)
    k = np.unravel_index(np.argmin(score), score.shape)
    return float(cx[k]), float(cy[k]), float(d[k])


def cancel(tool, h, why):
    print(f'  {why} - stopping the robot', flush=True)
    c = h.cancel_goal_async()
    rclpy.spin_until_future_complete(tool, c, timeout_sec=5.0)
    ok = c.result() is not None and len(c.result().goals_canceling) > 0
    return ok


def cmd_go(tool, args):
    wps = load()
    missing = [n for n in args.names if n not in wps]
    if missing:
        print(f'  unknown waypoint(s): {", ".join(missing)} - see "waypoints.py list"'); return 1
    if not args.skip_localisation_check:
        import nav_gate
        ok, why = nav_gate.ready_to_navigate(tool, tool.buf)
        print(f'  localisation: {why}')
        if not ok:
            print('  NOT starting the mission.'); return 1
    route = [(n, wps[n]['x'], wps[n]['y'], math.radians(wps[n]['yaw_deg'])) for n in args.names]
    if args.return_home:
        start = tool.robot_pose()
        if start is None:
            print('  no current pose to return to'); return 1
        route.append(('start', *start))
    route = route * args.loop

    status = tool.create_publisher(String, '/labrobot/mission_status',
                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    speed_reset = tool.create_publisher(SpeedLimit, '/speed_limit', 10)

    def say(text):
        print(text, flush=True)
        status.publish(String(data=text.strip()))

    cmd = {'last': None}
    tool.create_subscription(String, '/labrobot/mission_cmd',
                             lambda m: cmd.__setitem__('last', m.data.strip().lower()), 10)

    def stop_requested():
        return cmd['last'] in ('stop', 'cancel')

    cm = {}
    tool.create_subscription(Costmap, '/global_costmap/costmap_raw', lambda m: cm.__setitem__('c', m),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = ActionClient(tool, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=15):
        print('  navigate_to_pose not available - is navigation.launch.py running?'); return 1
    t_mission = time.time()
    while 'c' not in cm and time.time() - t_mission < 10:
        rclpy.spin_once(tool, timeout_sec=0.1)
    plan = f'mission: {" -> ".join(r[0] for r in route)}  ({len(route)} waypoints)'
    if args.wait_start:
        say(f'  {plan} - waiting for "go" on /labrobot/mission_cmd')
        cmd['last'] = None
        while cmd['last'] not in ('go', 'start'):
            rclpy.spin_once(tool, timeout_sec=0.1)
            if stop_requested():
                say('  mission cancelled before start'); return 130
        cmd['last'] = None
        t_mission = time.time()
    say(f'  {plan} - ROBOT WILL MOVE')

    results = []
    for k, (name, x, y, yaw) in enumerate(route):
        outcome = None
        for attempt in (1, 2):                 # second try only after a failed leg
            for _ in range(5):
                rclpy.spin_once(tool, timeout_sec=0.05)   # freshest costmap
            here = tool.robot_pose(timeout=3.0)
            tgt = (feasible_target(cm['c'], here[:2], x, y)
                   if ('c' in cm and here) else (x, y, 0.0))
            if tgt is None:
                outcome = ('BLOCKED', f'no reachable spot within {MAX_SHIFT_M:.1f} m - skipped')
                break
            tx, ty, shift = tgt
            note = '' if shift == 0 else f' (blocked - approaching {shift*100:.0f} cm away)'
            say(f'  [{time.time()-t_mission:6.1f}s] {k+1}/{len(route)} {name}{note}'
                + (' - retry' if attempt == 2 else ''))
            speed_reset.publish(SpeedLimit(percentage=False, speed_limit=0.0))
            goal = NavigateToPose.Goal(); goal.pose = pose_msg(tool, tx, ty, yaw)
            f = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(tool, f, timeout_sec=15)
            h = f.result()
            if h is None or not h.accepted:
                outcome = ('FAILED', 'goal rejected'); continue
            r = h.get_result_async()
            try:
                while not r.done():
                    rclpy.spin_once(tool, timeout_sec=0.1)
                    if stop_requested():
                        ok = cancel(tool, h, 'stop from /labrobot/mission_cmd')
                        say('  mission cancelled' if ok else
                            '  WARNING: cancel not confirmed - stop the robot manually')
                        return 130
                    if args.timeout and time.time() - t_mission > args.timeout:
                        print('  mission TIMEOUT - cancelling', flush=True)
                        c = h.cancel_goal_async(); rclpy.spin_until_future_complete(tool, c, timeout_sec=5)
                        return 1
            except KeyboardInterrupt:
                ok = cancel(tool, h, 'cancelled by user')
                say('  mission cancelled' if ok else
                    '  WARNING: cancel not confirmed - stop the robot manually')
                return 130
            if r.result().status == GoalStatus.STATUS_SUCCEEDED:
                end = tool.robot_pose(timeout=2.0)
                err = math.hypot(end[0]-x, end[1]-y) if end else float('nan')
                outcome = (('APPROACHED' if shift else 'REACHED'),
                           f'{err*100:.0f} cm from {name}' + (f' (blocked; target moved {shift*100:.0f} cm)'
                                                              if shift else ''))
                break
            outcome = ('FAILED', f'navigation status {r.result().status}')
        results.append((name, *outcome))
        say(f'           -> {name} {outcome[0]}: {outcome[1]}')
        if k + 1 < len(route):
            t_pause = time.time()
            while time.time() - t_pause < 2.0:     # settle, and still frames for RTAB-Map
                rclpy.spin_once(tool, timeout_sec=0.1)
            if stop_requested():
                say('  mission cancelled between waypoints'); return 130

    n_ok = sum(1 for r in results if r[1] in ('REACHED', 'APPROACHED'))
    say(f'\n  mission finished after {time.time()-t_mission:.0f} s: {n_ok}/{len(results)} waypoints '
        f'reached or approached')
    for name, kind, detail in results:
        print(f'    {name:8s} {kind:10s} {detail}')
    return 0 if n_ok == len(results) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('save', help="save the robot's current pose")
    s.add_argument('name'); s.add_argument('--force', action='store_true')
    s = sub.add_parser('pick', help='save a pose clicked in Foxglove (topic /waypoint_pick)')
    s.add_argument('name'); s.add_argument('--force', action='store_true')
    s.add_argument('--timeout', type=float, default=120.0)
    s = sub.add_parser('add', help='save an explicit map pose')
    s.add_argument('name'); s.add_argument('x', type=float); s.add_argument('y', type=float)
    s.add_argument('yaw', type=float, nargs='?', default=0.0, help='degrees')
    s.add_argument('--force', action='store_true')
    sub.add_parser('list')
    s = sub.add_parser('delete'); s.add_argument('name')
    s = sub.add_parser('go', help='run a mission through named waypoints')
    s.add_argument('names', nargs='+')
    s.add_argument('--return', dest='return_home', action='store_true',
                   help='come back to where the mission started')
    s.add_argument('--loop', type=int, default=1, help='repeat the route this many times')
    s.add_argument('--timeout', type=float, default=0.0, help='seconds; 0 = none')
    s.add_argument('--skip-localisation-check', action='store_true')
    s.add_argument('--wait-start', action='store_true',
                   help='wait for "go" on /labrobot/mission_cmd (e.g. a Foxglove Publish panel)')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); tool = Tool()
    try:
        rc = {'save': cmd_save, 'pick': cmd_pick, 'add': cmd_add, 'list': cmd_list,
              'delete': cmd_delete, 'go': cmd_go}[args.cmd](tool, args)
    except KeyboardInterrupt:
        rc = 130
    finally:
        tool.destroy_node(); rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
