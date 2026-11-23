#!/usr/bin/env python3
"""Ask Nav2's planner for a route WITHOUT moving the robot, and show where it goes.

    python3 tools/plan_probe.py --forward 2.4 --left -0.10
"""
import argparse, math, time, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
ap = argparse.ArgumentParser()
ap.add_argument('--forward', type=float, required=True)
ap.add_argument('--left', type=float, default=0.0)
ap.add_argument('--at', type=str, default='0.5,1.0,1.5,1.7,1.8,1.9,2.0,2.1,2.2,2.4')
a = ap.parse_args()
rclpy.init(); n = Node('plan_probe'); b = tf2_ros.Buffer(); tf2_ros.TransformListener(b, n)
t = time.time()
while time.time() - t < 4: rclpy.spin_once(n, timeout_sec=0.05)
tf = b.lookup_transform('map', 'base_footprint', rclpy.time.Time()); p = tf.transform.translation; q = tf.transform.rotation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)); c, s = math.cos(yaw), math.sin(yaw)
g = PoseStamped(); g.header.frame_id = 'map'; g.header.stamp = n.get_clock().now().to_msg()
g.pose.position.x = p.x + c*a.forward - s*a.left; g.pose.position.y = p.y + s*a.forward + c*a.left; g.pose.orientation = q
ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
if not ac.wait_for_server(timeout_sec=10): print('  planner action not available'); raise SystemExit(1)
goal = ComputePathToPose.Goal(); goal.goal = g; goal.planner_id = 'GridBased'; goal.use_start = False
f = ac.send_goal_async(goal); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
r = f.result().get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=15)
path = r.result().result.path.poses
print(f'  goal {a.forward:.2f} m forward, {a.left:+.2f} m left -> {len(path)} poses')
if not path: raise SystemExit(1)
pts = []
for ps in path:
    dx = ps.pose.position.x - p.x; dy = ps.pose.position.y - p.y
    pts.append((c*dx + s*dy, -s*dx + c*dy))
L = sum(math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]) for i in range(1, len(pts)))
print(f'  route length {L:.2f} m (straight line {math.hypot(a.forward, a.left):.2f} m)')
for want in [float(x) for x in a.at.split(',')]:
    if want > a.forward + 0.01: continue
    fx, ly = min(pts, key=lambda k: abs(k[0]-want))
    print(f'    {want:.1f} m ahead -> route at lateral {ly:+.3f} m')
rclpy.shutdown()
