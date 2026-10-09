#!/usr/bin/env python3
"""Print the local costmap as an ASCII picture with the robot footprint on it.

    ros2 run labrobot_bringup check_costmap.py
    ros2 run labrobot_bringup check_costmap.py --span 2.5

Why this exists
---------------
When Nav2 refuses to move, the log says "collision ahead" or "failed to make
progress" but not WHERE. Reading a costmap through `ros2 topic echo` is
hopeless - it is a flat array of a few thousand bytes. Every navigation bug on
this robot so far has been a costmap bug (stale marks frozen by a bad
raytrace_min_range, the camera falling outside its own voxel grid, depth noise
at 3 m painted as a wall), and each one was obvious the moment the grid was
drawn as a picture and invisible before that.

The footprint matters as much as the obstacles. The controller's collision
check is the FOOTPRINT POLYGON against lethal cells, not the robot centre
against inflation. A gap can look blocked in the inflation layer and still be
perfectly drivable, so the picture has to show which cells the robot's own body
would actually occupy.

Reading the picture
-------------------
  R  cells the robot footprint covers right now
  #  lethal (254) - the controller will refuse to put the footprint here
  +  inscribed (253) - centre may not go here, footprint edge may
  :  inflation cost, decaying with distance; discourages, does not forbid
  .  free
  ?  unknown

Read the RAW costmap, not the display one
-----------------------------------------
`/local_costmap/costmap` is an OccupancyGrid that Costmap2DPublisher has
RESCALED to 0..100 for display: 254 becomes 100, 253 becomes 99, 1..252 are
squeezed into 1..98 and 255 becomes -1. Thresholding that at 253/254 finds
nothing, so a costmap packed with inscribed cells reads back as "0% lethal,
footprint clear". `/local_costmap/costmap_raw` (nav2_msgs/Costmap) carries the
true 0..255 values the controller actually tests against, so that is what this
reads.

x is up the page (robot forward), y is to the left, matching REP-103.
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

LETHAL, INSCRIBED = 254, 253


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/local_costmap/costmap_raw')
    ap.add_argument('--span', type=float, default=1.6,
                    help='metres to show either side of the robot')
    ap.add_argument('--seconds', type=float, default=8.0)
    # Half-extents of the footprint in nav2_params.yaml. Kept as arguments so
    # this stays honest if the footprint is ever changed there.
    ap.add_argument('--half-length', type=float, default=0.21)
    ap.add_argument('--half-width', type=float, default=0.203)
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    n = Node('check_costmap')
    grid = {'m': None}
    # Costmaps are published TRANSIENT_LOCAL + RELIABLE. Subscribing with the
    # default volatile QoS silently matches nothing and waits forever.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(Costmap, args.topic,
                          lambda m: grid.__setitem__('m', m), qos)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)

    t0 = time.time()
    while time.time() - t0 < args.seconds and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
    m = grid['m']
    if m is None:
        print(f'  no costmap on {args.topic} in {args.seconds:.0f} s')
        return 1

    try:
        tf = buf.lookup_transform(m.header.frame_id, 'base_footprint',
                                  rclpy.time.Time())
    except Exception as e:  # noqa: BLE001
        print(f'  no TF {m.header.frame_id} -> base_footprint: {e}')
        return 1
    rx = tf.transform.translation.x
    ry = tf.transform.translation.y
    ryaw = yaw_of(tf.transform.rotation)

    md = m.metadata
    res = md.resolution
    W, H = md.size_x, md.size_y
    ox, oy = md.origin.position.x, md.origin.position.y
    # nav2_msgs/Costmap carries uint8 0..255 with no rescaling, so these compare
    # directly against LETHAL/INSCRIBED as the controller does.
    data = np.asarray(m.data, dtype=np.uint16).reshape(H, W)

    # Sample on a robot-centred grid so "up the page" is always robot-forward,
    # independent of how the map frame happens to be oriented.
    step = res
    ticks = int(args.span / step)
    c, s = math.cos(ryaw), math.sin(ryaw)

    rows = []
    counts = {'lethal': 0, 'inscribed': 0, 'inflated': 0, 'free': 0, 'unknown': 0}
    fp_worst, fp_cells = 0, 0
    for i in range(ticks, -ticks - 1, -1):          # forward at the top
        fwd = i * step
        row = []
        for j in range(ticks, -ticks - 1, -1):      # left at the left
            lat = j * step
            wx = rx + c * fwd - s * lat
            wy = ry + s * fwd + c * lat
            gx = int((wx - ox) / res)
            gy = int((wy - oy) / res)
            if not (0 <= gx < W and 0 <= gy < H):
                row.append(' ')
                continue
            v = int(data[gy, gx])
            on_body = abs(fwd) <= args.half_length and abs(lat) <= args.half_width
            if on_body:
                fp_cells += 1
                if v != 255:
                    fp_worst = max(fp_worst, v)

            if v == 255:
                ch, key = '?', 'unknown'
            elif v >= LETHAL:
                ch, key = '#', 'lethal'
            elif v >= INSCRIBED:
                ch, key = '+', 'inscribed'
            elif v > 0:
                ch, key = ':', 'inflated'
            else:
                ch, key = '.', 'free'
            counts[key] += 1
            row.append('R' if on_body and ch in '.:' else ch)
        rows.append(''.join(row))

    print(f'  {args.topic}  {W}x{H} @ {res:.3f} m   robot at '
          f'x={rx:+.2f} y={ry:+.2f} yaw={math.degrees(ryaw):+.0f} deg '
          f'in {m.header.frame_id}')
    print(f'  view {2*args.span:.1f} x {2*args.span:.1f} m, forward is UP, '
          f'left is LEFT, one char = {res*100:.0f} cm\n')
    for k, r in enumerate(rows):
        mark = ' <- robot row' if k == ticks else ''
        print(f'  {r}{mark}')
    tot = max(1, sum(counts.values()))
    print(f'\n  free {100*counts["free"]/tot:.0f}%   inflated '
          f'{100*counts["inflated"]/tot:.0f}%   inscribed '
          f'{100*counts["inscribed"]/tot:.0f}%   lethal '
          f'{100*counts["lethal"]/tot:.0f}%   unknown '
          f'{100*counts["unknown"]/tot:.0f}%')
    verdict = ('FOOTPRINT IN COLLISION - the controller cannot move from here'
               if fp_worst >= INSCRIBED else 'footprint clear')
    print(f'  footprint: {fp_cells} cells, worst cost {fp_worst}  -> {verdict}')

    # Where is the nearest lethal cell, and in which direction? This is the
    # number that decides whether a gap is threadable.
    ly, lx = np.where(data >= LETHAL)
    if len(lx):
        wx = ox + (lx + 0.5) * res
        wy = oy + (ly + 0.5) * res
        dx, dy = wx - rx, wy - ry
        fwd = c * dx + s * dy
        lat = -s * dx + c * dy
        d = np.hypot(dx, dy)
        k = int(np.argmin(d))
        print(f'  nearest lethal cell: {d[k]*100:.0f} cm away, '
              f'{fwd[k]*100:+.0f} cm forward, {lat[k]*100:+.0f} cm left')
        ahead = (fwd > 0.0) & (np.abs(lat) <= args.half_width)
        if ahead.any():
            print(f'  nearest lethal DIRECTLY AHEAD (within body width): '
                  f'{fwd[ahead].min()*100:.0f} cm')
        else:
            print('  no lethal cell directly ahead within body width')
    else:
        print('  no lethal cells anywhere in the local costmap')

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
