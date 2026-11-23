#!/usr/bin/env python3
"""Is the map any good?

    ros2 run labrobot_bringup check_map.py             # 15 s sample
    ros2 run labrobot_bringup check_map.py --seconds 40
"""

import argparse
import math
import os
import sys
import threading
import time

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)

try:
    from rtabmap_msgs.msg import Info
    HAVE_INFO = True
except ImportError:  # rtabmap not installed; the grid part still works
    HAVE_INFO = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--map-topic', default='/map')
    ap.add_argument('--info-topic', default='/info')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('check_map')
    state = {'grid': None, 'grids': 0, 'info': None, 'infos': 0}

    # The map is latched (TRANSIENT_LOCAL); match it or nothing arrives.
    map_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_map(m):
        state['grid'] = m
        state['grids'] += 1

    node.create_subscription(OccupancyGrid, args.map_topic, on_map, map_qos)

    # Loop closures must be COUNTED over the window, not sampled.
    state['closures'] = []
    state['max_hyp'] = 0.0

    if HAVE_INFO:
        def on_info(m):
            state['info'] = m
            state['infos'] += 1
            if m.loop_closure_id != 0:
                state['closures'].append(('loop', m.loop_closure_id))
            if m.proximity_detection_id != 0:
                state['closures'].append(('proximity', m.proximity_detection_id))
            for i, k in enumerate(m.stats_keys):
                if k.startswith('Loop/Highest_hypothesis_value/'):
                    state['max_hyp'] = max(state['max_hyp'], m.stats_values[i])
                    break
        node.create_subscription(Info, args.info_topic, on_info,
                                 QoSProfile(depth=10,
                                            reliability=ReliabilityPolicy.RELIABLE))

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    print(f'\n  holding subscriptions for {args.seconds:.0f} s so RTAB-Map '
          'actually builds the grid...')
    time.sleep(args.seconds)

    print('\n' + '=' * 66)
    print('MAP HEALTH')
    print('=' * 66)

    g = state['grid']
    problems = []
    if g is None:
        print(f'  no {args.map_topic} received - is mapping.launch.py running?')
        problems.append('no map')
    else:
        d = g.data
        free = sum(1 for c in d if c == 0)
        occ = sum(1 for c in d if c > 50)
        unk = sum(1 for c in d if c < 0)
        res = g.info.resolution
        area = free * res * res
        print(f'  grid       : {g.info.width} x {g.info.height} @ {res*100:.0f} cm '
              f'= {g.info.width*res:.1f} x {g.info.height*res:.1f} m')
        print(f'  origin     : ({g.info.origin.position.x:+.2f}, '
              f'{g.info.origin.position.y:+.2f})')
        print(f'  cells      : free {free}   occupied {occ}   unknown {unk}')
        print(f'  mapped area: {area:.2f} m2 of free space')
        print(f'  grid msgs  : {state["grids"]} in {args.seconds:.0f} s')
        if free < 100:
            problems.append('almost nothing mapped')

    if HAVE_INFO and state['info'] is not None:
        m = state['info']
        ks, vs = list(m.stats_keys), list(m.stats_values)

        def stat(prefix, default=None):
            for i, k in enumerate(ks):
                if k.startswith(prefix):
                    return vs[i]
            return default

        print()
        n_loop = sum(1 for k, _ in state['closures'] if k == 'loop')
        n_prox = sum(1 for k, _ in state['closures'] if k == 'proximity')
        print(f'  loop closures        : {n_loop} in {args.seconds:.0f} s'
              + (f'   -> nodes {[i for k, i in state["closures"] if k == "loop"][:8]}'
                 if n_loop else ''))
        print(f'  proximity detections : {n_prox}')
        print(f'  best hypothesis seen : {state["max_hyp"]:.3f}   '
              f'(accept threshold 0.11)')
        print(f'  /info messages       : {state["infos"]}')
        wm = stat('Memory/Working_memory_size/')
        lg = stat('Memory/Local_graph_size/')
        if wm is not None:
            print(f'  working memory       : {wm:.0f} nodes')
        if lg is not None:
            print(f'  local graph          : {lg:.0f} nodes')
    elif HAVE_INFO:
        print(f'\n  no {args.info_topic} received')

    # map -> odom is loop closure's correction to accumulated odometry drift.
    try:
        tf = buf.lookup_transform('map', 'odom', rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        dist = math.hypot(t.x, t.y)
        print(f'\n  map->odom correction : {dist*1000:.1f} mm, '
              f'{math.degrees(yaw):+.2f} deg')
        if dist > 1.0:
            problems.append(f'map->odom correction {dist:.2f} m is large - '
                            'suspect a false loop closure')
    except Exception as e:  # noqa: BLE001
        print(f'\n  map->odom not available: {e}')

    print()
    if problems:
        print('  ISSUES: ' + '; '.join(problems))
    else:
        print('  Map is building and consistent.')
    print()

    sys.stdout.flush()
    os._exit(1 if problems else 0)


if __name__ == '__main__':
    main()
