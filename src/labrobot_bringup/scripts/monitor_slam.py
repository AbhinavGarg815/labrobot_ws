#!/usr/bin/env python3
"""Live SLAM quality while you teleop.

    ros2 run labrobot_bringup monitor_slam.py
"""

import argparse
import math
import os
import sys
import threading
import time

import rclpy
import tf2_ros
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:
    from rtabmap_msgs.msg import Info
except ImportError:
    print('rtabmap_msgs not available'); sys.exit(1)


def stat(msg, key, default=None):
    """Exact-key lookup."""
    for i, k in enumerate(msg.stats_keys):
        if k == key:
            return msg.stats_values[i]
    return default


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--period', type=float, default=4.0)
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    n = Node('monitor_slam')
    state = {'info': None, 'loops': 0, 'prox': 0, 'last_corr': None,
             'max_jump': 0.0, 'jumps': []}

    def on_info(m):
        state['info'] = m
        if m.loop_closure_id != 0:
            state['loops'] += 1
        if m.proximity_detection_id != 0:
            state['prox'] += 1

    n.create_subscription(Info, '/info', on_info,
                          QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)

    ex = SingleThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    print('  drive now. Ctrl-C to stop.\n')
    print(f'  {"time":>6} {"nodes":>6} {"loops":>6} {"prox":>5} {"drift":>9} '
          f'{"jump":>8} {"words":>6}  state')
    t0 = time.time()
    try:
        while rclpy.ok():
            time.sleep(args.period)
            m = state['info']
            if m is None:
                print('  waiting for /info - is mapping running?')
                continue

            nodes = stat(m, 'Memory/Working_memory_size/', 0)
            if nodes is None:
                nodes = 0
            words = stat(m, 'Keypoint/Current_frame/words')
            inliers = stat(m, 'Loop/Visual_inliers/', 0)

            # map->odom is the accumulated correction loop closure has applied.
            corr = None
            try:
                tf = buf.lookup_transform('map', 'odom', rclpy.time.Time())
                t = tf.transform.translation
                corr = math.hypot(t.x, t.y)
            except Exception:  # noqa: BLE001
                pass

            jump = 0.0
            if corr is not None and state['last_corr'] is not None:
                jump = abs(corr - state['last_corr'])
                if jump > state['max_jump']:
                    state['max_jump'] = jump
                if jump > 0.5:
                    state['jumps'].append((time.time() - t0, jump))
            if corr is not None:
                state['last_corr'] = corr

            # One-word verdict, so the useful information is glanceable.
            if words is None:
                verdict = 'feature count unavailable'
            elif words < 150:
                verdict = f'FEW FEATURES ({int(words)}) - blank scene, loops will not close'
            elif jump > 0.5:
                verdict = '*** BIG JUMP - suspect a false loop closure ***'
            elif state['loops'] > 0:
                verdict = 'closing loops, healthy'
            elif corr is not None and corr > 0.5:
                verdict = 'drifting, no closures yet - revisit a mapped area'
            else:
                verdict = 'mapping'

            print(f'  {time.time()-t0:6.0f} {int(nodes):6d} {state["loops"]:6d} '
                  f'{state["prox"]:5d} '
                  f'{(f"{corr*1000:7.0f}mm" if corr is not None else "      -"):>9} '
                  f'{jump*1000:7.0f}mm '
                  f'{(int(words) if words is not None else -1):6d}  {verdict}')
    except KeyboardInterrupt:
        pass

    print('\n  ' + '=' * 60)
    print(f'  duration        : {time.time()-t0:.0f} s')
    print(f'  loop closures   : {state["loops"]}   proximity: {state["prox"]}')
    print(f'  final drift     : '
          f'{state["last_corr"]*1000:.0f} mm' if state['last_corr'] else '  final drift: -')
    print(f'  largest jump    : {state["max_jump"]*1000:.0f} mm')
    if state['jumps']:
        print(f'  SUSPECT JUMPS   : {len(state["jumps"])} over 0.5 m '
              f'-> the map may be folded; consider re-mapping')
    else:
        print('  no suspect jumps - no evidence of a false loop closure')
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
