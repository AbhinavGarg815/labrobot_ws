#!/usr/bin/env python3
"""Convert every goal clicked in Foxglove into the map frame, once, on arrival.

    ros2 run labrobot_bringup goal_relay.py        # started by navigation.launch.py
"""
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time

TARGET = 'map'


def quat_mul(a, b):
    """Hamilton product of (x, y, z, w) quaternions."""
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    x, y, z, w = quat_mul(quat_mul(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return x, y, z


def yaw_deg(q):
    x, y, z, w = q
    return math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


class GoalRelay(Node):
    def __init__(self):
        super().__init__('goal_relay')
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(PoseStamped, '/labrobot/goal_pose_map', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.get_logger().info('relaying /goal_pose -> /labrobot/goal_pose_map (map frame)')

    def on_goal(self, msg):
        src = msg.header.frame_id or TARGET
        p, o = msg.pose.position, msg.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        if src == TARGET:
            pos = (p.x, p.y, p.z)
        else:
            try:
                tf = self.buf.lookup_transform(TARGET, src, Time(),
                                               timeout=rclpy.duration.Duration(seconds=1.0))
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'goal in frame "{src}" DROPPED - no transform to {TARGET}: {e}')
                return
            t, r = tf.transform.translation, tf.transform.rotation
            rq = (r.x, r.y, r.z, r.w)
            rx, ry, rz = rotate(rq, (p.x, p.y, p.z))
            pos = (rx + t.x, ry + t.y, rz + t.z)
            q = quat_mul(rq, q)
        out = PoseStamped()
        out.header.frame_id = TARGET
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x, out.pose.position.y, out.pose.position.z = pos
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = q
        self.pub.publish(out)
        self.get_logger().info(f'goal ({src}) -> map ({pos[0]:+.2f}, {pos[1]:+.2f}, {yaw_deg(q):+.0f} deg)')


def main():
    rclpy.init(); n = GoalRelay()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
