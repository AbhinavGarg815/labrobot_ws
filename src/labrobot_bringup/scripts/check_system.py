#!/usr/bin/env python3
"""One consolidated health check: every topic rate, plus CPU, RAM and thermals.

    ros2 run labrobot_bringup check_system.py            # 10 s sample
    ros2 run labrobot_bringup check_system.py --seconds 30
"""

import argparse
import glob
import os
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, JointState
from tf2_msgs.msg import TFMessage

TOPICS = [
    ('camera', 'colour',       Image,      '/camera/camera/color/image_raw',                  30.0, 0.85),
    ('camera', 'depth_align',  Image,      '/camera/camera/aligned_depth_to_color/image_raw',  30.0, 0.85),
    ('camera', 'depth_raw',    Image,      '/camera/camera/depth/image_rect_raw',              30.0, 0.85),
    ('imu',    'imu_raw',      Imu,        '/camera/camera/imu',                             200.0, 0.90),
    ('imu',    'imu_madgwick', Imu,        '/imu/data',                                      200.0, 0.90),
    ('imu',    'imu_unbiased', Imu,        '/imu/unbiased',                                  200.0, 0.90),
    ('state',  'wheel_odom',   Odometry,   '/diff_drive_controller/odom',                     50.0, 0.90),
    ('state',  'joint_states', JointState, '/joint_states',                                   50.0, 0.90),
    ('state',  'ekf_odom',     Odometry,   '/odometry/filtered',                              50.0, 0.80),
    ('state',  'visual_odom',  Odometry,   '/vo/odom',                                        15.0, 0.70),
    ('state',  'tf',           TFMessage,  '/tf',                                              0.0, 0.0),
]

GROUP_ORDER = ('camera', 'imu', 'state')

# Processes worth costing individually.
NODES = [
    ('realsense',    b'realsense2_camera_node'),
    ('ros2_control', b'ros2_control_node'),
    ('ekf',          b'ekf_node'),
    ('madgwick',     b'imu_filter_madgwick'),
    ('imu_bias',     b'imu_bias_compensator'),
    ('visual_odom',  b'rgbd_odometry'),
    ('rtabmap',      b'rtabmap'),
]


def find_pids():
    """Locate each node's process, ignoring anything that merely MENTIONS it."""
    me = os.getpid()
    found = {}
    for p in os.listdir('/proc'):
        if not p.isdigit():
            continue
        pid = int(p)
        if pid == me:
            continue
        try:
            with open(f'/proc/{p}/cmdline', 'rb') as f:
                cmd = f.read()
        except OSError:
            continue
        # Interpreters running a command line that talks ABOUT a node are not that node.
        if cmd.startswith(b'/bin/bash') or cmd.startswith(b'/bin/sh') \
                or cmd.startswith(b'bash') or b'shell-snapshots' in cmd:
            continue
        for label, needle in NODES:
            if needle in cmd and label not in found:
                found[label] = pid
                break
    return found


def proc_stats(pid):
    """(cpu_ticks, rss_bytes, threads) or None if the process vanished."""
    try:
        with open(f'/proc/{pid}/stat') as f:
            parts = f.read().split()
        ticks = int(parts[13]) + int(parts[14])
        threads = int(parts[19])
        with open(f'/proc/{pid}/statm') as f:
            rss_pages = int(f.read().split()[1])
        return ticks, rss_pages * os.sysconf('SC_PAGE_SIZE'), threads
    except (OSError, IndexError, ValueError):
        return None


def system_cpu_ticks():
    with open('/proc/stat') as f:
        parts = f.readline().split()[1:]
    vals = [int(v) for v in parts]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return sum(vals), idle


def meminfo():
    d = {}
    with open('/proc/meminfo') as f:
        for line in f:
            k, _, v = line.partition(':')
            d[k] = int(v.split()[0]) * 1024
    return d


def thermals():
    out = []
    for zone in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
        try:
            with open(f'{zone}/type') as f:
                name = f.read().strip()
            with open(f'{zone}/temp') as f:
                t = int(f.read().strip()) / 1000.0
            if t > 0:
                out.append((name, t))
        except Exception:  # noqa: BLE001
            continue
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--camera-fps', type=float, default=15.0,
                    help='expected camera rate; must match camera.launch.py fps')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    for i, t in enumerate(TOPICS):
        if t[0] == 'camera':
            TOPICS[i] = (t[0], t[1], t[2], t[3], args.camera_fps, t[5])

    rclpy.init()

    pids = find_pids()
    before = {k: proc_stats(v) for k, v in pids.items()}
    sys_before = system_cpu_ticks()
    mem_before = meminfo()

    rates = {}
    # Wall clock across ALL groups, including the per-group discovery waits.
    wall0 = time.time()
    for group in GROUP_ORDER:
        entries = [t for t in TOPICS if t[0] == group]
        node = Node(f'check_system_{group}')
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        counts = {e[1]: 0 for e in entries}

        def mk(label, c=counts):
            def cb(_m):
                c[label] += 1
            return cb

        for _, label, typ, topic, _, _ in entries:
            node.create_subscription(typ, topic, mk(label), qos)

        ex = SingleThreadedExecutor()
        ex.add_node(node)
        t = threading.Thread(target=ex.spin, daemon=True)
        t.start()

        time.sleep(1.5)
        for k in counts:
            counts[k] = 0
        t0 = time.time()
        time.sleep(args.seconds)
        el = time.time() - t0
        for k, v in counts.items():
            rates[k] = v / el

        ex.shutdown()
        node.destroy_node()

    after = {k: proc_stats(v) for k, v in pids.items()}
    sys_after = system_cpu_ticks()
    mem = meminfo()
    el = time.time() - wall0
    hz_tick = os.sysconf('SC_CLK_TCK')
    ncpu = os.cpu_count()

    print()
    print('=' * 74)
    print(f'LABROBOT SYSTEM HEALTH   ({args.seconds:.0f} s sample, {ncpu} cores)')
    print('=' * 74)

    print(f'\n  {"topic":<14}{"actual":>10}{"expected":>10}   status')
    problems = []
    for group, label, _, topic, want, tol in TOPICS:
        rate = rates.get(label, 0.0)
        if want == 0.0:
            print(f'  {label:<14}{rate:9.1f} Hz{"-":>10}   (informational)')
            continue
        if rate == 0.0:
            status, bad = 'ABSENT', True
        elif rate < want * tol:
            status, bad = f'LOW ({100*rate/want:.0f}%)', True
        else:
            status, bad = 'ok', False
        print(f'  {label:<14}{rate:9.1f} Hz{want:9.0f} Hz   {status}')
        if bad:
            problems.append(f'{label} {rate:.1f}/{want:.0f} Hz')

    print(f'\n  {"node":<14}{"CPU":>9}{"of machine":>12}{"RSS":>10}{"threads":>9}')
    total_cpu = 0.0
    for label, pid in sorted(pids.items()):
        b, a = before.get(label), after.get(label)
        if not b or not a:
            print(f'  {label:<14}   (exited during sample)')
            continue
        cpu = 100.0 * (a[0] - b[0]) / hz_tick / el
        total_cpu += cpu
        print(f'  {label:<14}{cpu:8.1f}%{cpu/ncpu:11.1f}%{a[1]/2**20:9.0f} MB{a[2]:9d}')
    for label, _ in NODES:
        if label not in pids:
            print(f'  {label:<14}   not running')
    print(f'  {"TOTAL (ours)":<14}{total_cpu:8.1f}%{total_cpu/ncpu:11.1f}%')

    st, si = sys_before
    at, ai = sys_after
    busy = 100.0 * (1.0 - (ai - si) / max(1, at - st))
    used = mem['MemTotal'] - mem['MemAvailable']
    print(f'\n  machine CPU busy : {busy:.1f}%   headroom {100-busy:.1f}%')
    print(f'  RAM              : {used/2**30:.2f} / {mem["MemTotal"]/2**30:.2f} GiB '
          f'used ({100*used/mem["MemTotal"]:.0f}%)')
    grew = (used - (mem_before['MemTotal'] - mem_before['MemAvailable'])) / 2**20
    print(f'  RAM change       : {grew:+.0f} MB over the sample')
    if mem.get('SwapTotal', 0):
        swap_used = mem['SwapTotal'] - mem.get('SwapFree', 0)
        print(f'  swap in use      : {swap_used/2**20:.0f} MB'
              + ('   <-- investigate' if swap_used > 64 * 2**20 else ''))

    temps = thermals()
    if temps:
        hottest = max(temps, key=lambda x: x[1])
        print(f'  thermals         : ' + '  '.join(f'{n} {t:.0f}C' for n, t in temps[:5]))
        if hottest[1] > 80:
            problems.append(f'{hottest[0]} at {hottest[1]:.0f}C - likely throttling')

    print()
    if problems:
        print('  ISSUES: ' + '; '.join(problems))
    else:
        print('  All topics at rate, thermals normal.')
    print()

    sys.stdout.flush()
    os._exit(1 if problems else 0)


if __name__ == '__main__':
    main()
