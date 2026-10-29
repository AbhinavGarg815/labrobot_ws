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

Waypoints live in docs/maps/waypoints.yaml (tracked in git, small). They are
map-frame poses, so they belong to the map they were saved in: after a fresh
map (not after lifelong-SLAM updates, which keep the frame) re-save them.

`go` uses Nav2's FollowWaypoints action (waypoint_follower): one NavigateToPose
per waypoint, a 2 s pause at each (wait_at_waypoint), and stop_on_failure:
false - a waypoint blocked by a chair is reported as missed and the mission
continues. It refuses to start unless the shared gate (nav_gate.py) passes:
startup localisation confirmed and the current view matches the map.

`pick` saves a pose clicked in Foxglove without driving there: in a 3D panel,
use "Publish pose" with the topic set to /waypoint_pick (NOT /goal_pose - the
navigator subscribes to that and would start driving).
"""
import argparse
import datetime
import math
import os
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node

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
    if m.header.frame_id not in ('map', ''):
        print(f'  pose is in frame "{m.header.frame_id}" - set the Foxglove fixed frame to map'); return 1
    return cmd_save(tool, args, picked=(m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)))


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
    client = ActionClient(tool, FollowWaypoints, 'follow_waypoints')
    if not client.wait_for_server(timeout_sec=15):
        print('  follow_waypoints not available - is navigation.launch.py running?'); return 1

    goal = FollowWaypoints.Goal()
    goal.poses = [pose_msg(tool, x, y, yaw) for _, x, y, yaw in route]
    names = [r[0] for r in route]
    print(f'  mission: {" -> ".join(names)}  ({len(route)} waypoints) - ROBOT WILL MOVE', flush=True)
    state = {'cur': -1, 't': time.time(), 'arrive': {}}

    def on_fb(m):
        i = m.feedback.current_waypoint
        if i != state['cur']:
            if state['cur'] >= 0:
                state['arrive'][state['cur']] = time.time()
            state['cur'] = i
            if i < len(names):
                print(f'  [{time.time()-state["t"]:6.1f}s] heading to {i+1}/{len(names)}: {names[i]}',
                      flush=True)

    t0 = time.time()
    f = client.send_goal_async(goal, feedback_callback=on_fb)
    rclpy.spin_until_future_complete(tool, f, timeout_sec=15)
    h = f.result()
    if h is None or not h.accepted:
        print('  mission rejected by the waypoint follower'); return 1
    r = h.get_result_async()
    try:
        while not r.done():
            rclpy.spin_once(tool, timeout_sec=0.1)
            if args.timeout and time.time() - t0 > args.timeout:
                print('  mission TIMEOUT - cancelling'); h.cancel_goal_async()
                rclpy.spin_once(tool, timeout_sec=1.0); return 1
    except KeyboardInterrupt:
        print('  cancelled by user - stopping the robot', flush=True)
        c = h.cancel_goal_async()
        rclpy.spin_until_future_complete(tool, c, timeout_sec=5.0)
        ok = c.result() is not None and len(c.result().goals_canceling) > 0
        print('  mission cancelled' if ok else '  WARNING: cancel not confirmed - stop the robot manually',
              flush=True)
        return 130
    res = r.result()
    missed = list(res.result.missed_waypoints)
    status = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
              GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(res.status, str(res.status))
    print(f'\n  mission {status} after {time.time()-t0:.0f} s: '
          f'{len(route)-len(missed)}/{len(route)} waypoints reached')
    for i in missed:
        print(f'    MISSED {i+1}: {names[i]}')
    end = tool.robot_pose(timeout=3.0)
    if end:
        _, gx, gy, _ = route[-1]
        print(f'  final pose x={end[0]:+.2f} y={end[1]:+.2f}, {math.hypot(end[0]-gx, end[1]-gy)*100:.0f} cm '
              f'from the last waypoint ({names[-1]})')
    return 0 if status == 'SUCCEEDED' and not missed else 1


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
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    # No rclpy signal handler, so Ctrl-C does NOT shut the ROS context down
    # before cmd_go can send its cancel: with the default handler the context
    # was already gone, the cancel could not be sent, and stopping the robot
    # depended on someone cancelling from outside (2026-10-27).
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
