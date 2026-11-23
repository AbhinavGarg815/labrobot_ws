#!/usr/bin/env python3
"""Publish a PLY point cloud as a latched PointCloud2, for viewing in Foxglove.

    ros2 run labrobot_bringup publish_cloud.py --ply docs/maps/lab_dense_cloud.ply
"""

import argparse
import os
import struct
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

# PLY scalar type -> (numpy dtype, size in bytes)
_PLY_TYPES = {
    'float': ('<f4', 4), 'float32': ('<f4', 4),
    'double': ('<f8', 8), 'float64': ('<f8', 8),
    'uchar': ('u1', 1), 'uint8': ('u1', 1),
    'char': ('i1', 1), 'int8': ('i1', 1),
    'ushort': ('<u2', 2), 'uint16': ('<u2', 2),
    'short': ('<i2', 2), 'int16': ('<i2', 2),
    'uint': ('<u4', 4), 'uint32': ('<u4', 4),
    'int': ('<i4', 4), 'int32': ('<i4', 4),
}


def read_ply(path):
    """Minimal binary-little-endian PLY reader for the vertex element."""
    with open(path, 'rb') as f:
        if f.readline().strip() != b'ply':
            raise ValueError('not a PLY file')
        fmt = None
        props = []          # (name, dtype, size) for the vertex element
        count = None
        in_vertex = False
        while True:
            line = f.readline()
            if not line:
                raise ValueError('unexpected end of PLY header')
            parts = line.decode('ascii', 'replace').strip().split()
            if not parts:
                continue
            if parts[0] == 'format':
                fmt = parts[1]
            elif parts[0] == 'element':
                in_vertex = (parts[1] == 'vertex')
                if in_vertex:
                    count = int(parts[2])
            elif parts[0] == 'property' and in_vertex:
                if parts[1] == 'list':
                    raise ValueError('list properties in vertex are unsupported')
                dt, sz = _PLY_TYPES[parts[1]]
                props.append((parts[2], dt, sz))
            elif parts[0] == 'end_header':
                break
        if fmt != 'binary_little_endian':
            raise ValueError(f'only binary_little_endian supported, got {fmt}')
        dtype = np.dtype([(n, d) for n, d, _ in props])
        data = np.frombuffer(f.read(dtype.itemsize * count), dtype=dtype, count=count)
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ply', required=True)
    ap.add_argument('--topic', default='/dense_map')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--stride', type=int, default=1,
                    help='publish every Nth point; raise it if the link is slow')
    ap.add_argument('--no-tf', action='store_true',
                    help='do not publish a static transform for the cloud frame')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    path = os.path.abspath(os.path.expanduser(args.ply))
    if not os.path.exists(path):
        print(f'  no such file: {path}')
        return 1

    v = read_ply(path)
    names = v.dtype.names
    for req in ('x', 'y', 'z'):
        if req not in names:
            print(f'  PLY has no {req} property'); return 1
    v = v[::max(1, args.stride)]
    n = len(v)

    xyz = np.stack([v['x'], v['y'], v['z']], axis=1).astype(np.float32)
    if all(c in names for c in ('red', 'green', 'blue')):
        r = v['red'].astype(np.uint32); g = v['green'].astype(np.uint32)
        b = v['blue'].astype(np.uint32)
        # PointCloud2 convention: RGB packed into the low 24 bits of a float32.
        packed = (r << 16) | (g << 8) | b
        rgb = packed.astype(np.uint32).view(np.float32)
        has_rgb = True
    else:
        rgb = np.zeros(n, dtype=np.float32)
        has_rgb = False

    buf = np.zeros((n, 4), dtype=np.float32)
    buf[:, 0:3] = xyz
    buf[:, 3] = rgb

    rclpy.init()
    node = Node('publish_cloud')
    # Latched: send once per subscriber.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PointCloud2, args.topic, qos)

    # Publish a static transform for the cloud's own frame unless told not to.
    tf_bc = None
    if not args.no_tf:
        tf_bc = StaticTransformBroadcaster(node)
        tfm = TransformStamped()
        tfm.header.stamp = node.get_clock().now().to_msg()
        tfm.header.frame_id = args.frame
        tfm.child_frame_id = f'{args.frame}_cloud_origin'
        tfm.transform.rotation.w = 1.0
        tf_bc.sendTransform(tfm)

    msg = PointCloud2()
    msg.header = Header(frame_id=args.frame)
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.height = 1
    msg.width = n
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = buf.tobytes()

    pub.publish(msg)
    print(f'  publishing {n} points on {args.topic} (frame {args.frame}), '
          f'{"with" if has_rgb else "WITHOUT"} colour, {len(msg.data)/2**20:.1f} MB')
    if tf_bc is not None:
        print(f'  static TF published so the "{args.frame}" frame resolves in a viewer')
    print('  latched - open a Foxglove 3D panel on this topic. Ctrl-C to stop.')

    try:
        timer_period = 5.0
        node.create_timer(timer_period, lambda: pub.publish(msg))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
