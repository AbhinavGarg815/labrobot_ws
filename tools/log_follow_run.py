#!/usr/bin/env python3
"""Record a person-following run (live, dry or driving) to a JSON-lines file.

    python3 tools/log_follow_run.py docs/logs/followrun_NAME.jsonl [--start] [--secs 150]
    python3 tools/log_follow_run.py --summary docs/logs/followrun_NAME.jsonl
"""

import argparse
import json
import statistics
import sys
import time


def record(a):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    out = open(a.file, 'w')
    rclpy.init()
    n = rclpy.create_node('follow_run_logger')
    lat = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    st, robot, status = {}, {}, {'s': ''}

    def w(kind, d):
        out.write(json.dumps({'t': round(time.time(), 2), 'k': kind, **d}) + '\n')
        out.flush()

    def on_status(m):
        status['s'] = m.data
        w('status', {'s': m.data})
    n.create_subscription(String, '/labrobot/follow_status', on_status, lat)
    n.create_subscription(String, '/labrobot/target_event', lambda m: w('event', {'s': m.data}), lat)
    n.create_subscription(String, '/labrobot/target_state', lambda m: st.update(json.loads(m.data)), 10)
    n.create_subscription(PoseStamped, '/labrobot/follow_goal', lambda m: w(
        'goal', {'x': round(m.pose.position.x, 3), 'y': round(m.pose.position.y, 3)}), 10)
    n.create_subscription(Path, '/labrobot/follow_plan', lambda m: w('plan', {'n': len(m.poses)}), 10)
    n.create_subscription(Odometry, '/odometry/filtered', lambda m: robot.update(
        x=round(m.pose.pose.position.x, 3), y=round(m.pose.pose.position.y, 3),
        vx=round(m.twist.twist.linear.x, 3), wz=round(m.twist.twist.angular.z, 3)), 10)
    cmd = n.create_publisher(String, '/labrobot/follow_cmd', 10)
    t0 = time.time()
    last = last_start = 0.0
    started = None
    print(f'logging to {a.file}' + (' - will send start when someone is in view' if a.start else ''),
          flush=True)
    try:
        while time.time() - t0 < a.max_wait + a.secs:
            rclpy.spin_once(n, timeout_sec=0.05)
            now = time.time()
            if now - last > 0.5 and st:
                last = now
                w('state', {**{k: st.get(k) for k in ('state', 'track_id', 'range', 'bearing_deg', 'x',
                                                      'y', 'speed', 'reid_score', 'people', 'ms')},
                            'robot': dict(robot)})
            if a.start and started is None and st.get('people', 0) >= 1 and now - t0 > 3 \
                    and status['s'].startswith('IDLE') and now - last_start > 5:
                cmd.publish(String(data='start'))
                w('cmd', {'s': 'start'})
                print('sent start', flush=True)
                last_start = now
            if started is None and status['s'].startswith('FOLLOW'):
                started = now
                w('cmd', {'s': f'following - recording {a.secs:.0f} s'})
                print(f'following - recording {a.secs:.0f} s', flush=True)
            if started is not None and now - started > a.secs:
                break
    except KeyboardInterrupt:
        pass
    w('end', {})
    out.close()
    print('done', flush=True)


def summary(path):
    L = [json.loads(x) for x in open(path)]
    t0 = next((x['t'] for x in L if x['k'] == 'cmd'), L[0]['t'])
    last = None
    for x in L:
        if x['k'] in ('status', 'event', 'cmd', 'end'):
            s = x.get('s', '')
            key = (x['k'], s.split(';')[0].split(' (')[0][:50])
            if key != last:
                print(f"{x['t'] - t0:7.1f} {x['k']:6s} {s[:160]}")
                last = key
    st = [x for x in L if x['k'] == 'state']
    if st:
        ms = [x['ms'] for x in st if x.get('ms')]
        print(f'states: {sorted({x["state"] for x in st})}; tracker ms median '
              f'{statistics.median(ms) if ms else 0:.1f}; goals {sum(x["k"] == "goal" for x in L)}, '
              f'plans {sum(x["k"] == "plan" for x in L)}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('file')
    ap.add_argument('--start', action='store_true')
    ap.add_argument('--secs', type=float, default=150.0)
    ap.add_argument('--max-wait', type=float, default=1200.0,
                    help='give up if following has not begun after this long')
    ap.add_argument('--summary', action='store_true')
    a = ap.parse_args()
    if a.summary:
        summary(a.file)
    else:
        record(a)


if __name__ == '__main__':
    sys.exit(main())
