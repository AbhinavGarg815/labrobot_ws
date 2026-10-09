#!/usr/bin/env python3
"""Measure the camera's YAW mounting error by driving straight at a gap.

    python3 tools/camera_yaw_check.py --drive 0.8

Why
---
The URDF's camera roll and pitch were measured from gravity; yaw cannot be, so
it is assumed 0. If the camera is actually turned by delta on its mount, every
obstacle is reported rotated by -delta about the camera: a point at distance x
appears shifted sideways by about x * delta. Navigation then centres the robot
on a gap that is not where the gap is. Three gap runs (72 cm and 61 cm, two
setups) all ran 4-7 cm RIGHT of centre - systematic, not grid rounding.

Method
------
1. Measure the gap's inner edges per 10 cm slice (10 stacked depth frames),
   and convert each edge to the ODOM frame using the current odom pose.
2. Drive straight forward --drive metres at 0.05 m/s (odometry-closed, with a
   heading hold), stopping early if anything is within the corridor.
3. Measure again from the closer pose, again into odom.
A real, static edge is at the same odom position both times. With a yaw error
it is not: its apparent sideways position changes by (x_A - x_B) * delta, where
x is its distance from the robot at each measurement. The gap CENTRE is used
(average of both edges), so a slightly wrong gap width cancels out.

delta > 0 means the camera points LEFT of the robot's heading; the URDF fix is
a yaw of +delta on the camera mount joint.
"""
import argparse, math, time
import numpy as np, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

def R_(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class Probe(Node):
    def __init__(self):
        super().__init__('camera_yaw_check')
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)
        self.frames, self.K = [], None
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.frames.append, be)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', lambda m: setattr(self, 'K', m), be)
        self.pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
    def spin_for(self, s):
        t = time.time()
        while time.time() - t < s: rclpy.spin_once(self, timeout_sec=0.02)
    def odom_pose(self):
        t = self.buf.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
    def points(self):
        """Robot-frame xy of obstacle returns (0.10-0.90 m high), 10 stacked frames."""
        self.frames.clear(); self.spin_for(1.5)
        d0 = self.frames[-1]
        tf = self.buf.lookup_transform('base_footprint', d0.header.frame_id, rclpy.time.Time())
        R = R_(tf.transform.rotation); T = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        K = self.K; out = []
        for d in self.frames[-10:]:
            z = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000
            v, u = np.mgrid[0:d.height, 0:d.width]; ok = (z > 0.3) & (z < 3.0)
            P = np.stack([(u[ok]-K.k[2])*z[ok]/K.k[0], (v[ok]-K.k[5])*z[ok]/K.k[4], z[ok]], 1) @ R.T + T
            out.append(P[(P[:, 2] > 0.10) & (P[:, 2] < 0.90)][:, :2])
        return np.vstack(out)

def edges_in_odom(P, pose, split):
    """Per 10 cm slice (in ROBOT x), inner gap edges -> odom; plus robot-frame distance."""
    x0, y0, th = pose; c, s = math.cos(th), math.sin(th)
    rows = []
    for lo in np.arange(0.8, 2.8, 0.1):
        S = P[(P[:, 0] >= lo) & (P[:, 0] < lo + 0.1)]
        L = S[S[:, 1] > split]; Rr = S[S[:, 1] < split]
        if len(L) < 80 or len(Rr) < 80: continue
        le = np.percentile(L[:, 1], 2); re = np.percentile(Rr[:, 1], 98)
        if not (0.45 < le - re < 0.95): continue         # only the gap itself
        xm, cm = lo + 0.05, (le + re) / 2
        rows.append({'x_robot': xm, 'centre_robot': cm, 'width': le - re,
                     'odom_x': x0 + c*xm - s*cm, 'odom_y': y0 + s*xm + c*cm})
    return rows

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drive', type=float, default=0.8)
    ap.add_argument('--split', type=float, default=0.0, help='lateral split between the two obstacles')
    a = ap.parse_args()
    rclpy.init(); n = Probe(); n.spin_for(3.0)
    if n.K is None or not n.frames: print('  no depth'); return 1

    A_pose = n.odom_pose(); PA = n.points(); A = edges_in_odom(PA, A_pose, a.split)
    if len(A) < 3: print(f'  gap not found from pose A ({len(A)} slices)'); return 1
    nearest = min(r['x_robot'] for r in A)
    corridor = PA[(np.abs(PA[:, 1]) < 0.26) & (PA[:, 0] > 0.2)]
    free = corridor[:, 0].min() - 0.21 - 0.15 if len(corridor) else 3.0
    D = min(a.drive, free, nearest - 0.21 - 0.35)    # keep the gap 0.35 m ahead of the bumper
    print(f'  pose A: gap seen over {len(A)} slices, nearest at {nearest:.2f} m; driving {D:.2f} m')
    if D < 0.3: print('  not enough room to drive a useful distance'); return 1

    # Drive straight, closed on odometry distance, holding the start heading.
    x0, y0, th0 = n.odom_pose(); t0 = time.time()
    while rclpy.ok():
        x, y, th = n.odom_pose()
        if math.hypot(x - x0, y - y0) >= D or time.time() - t0 > D / 0.05 + 15: break
        m = Twist(); m.linear.x = 0.05
        m.angular.z = max(-0.15, min(0.15, -2.0 * math.atan2(math.sin(th - th0), math.cos(th - th0))))
        n.pub.publish(m); rclpy.spin_once(n, timeout_sec=0.05)
    for _ in range(5): n.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
    n.spin_for(1.5)

    B_pose = n.odom_pose(); B = edges_in_odom(n.points(), B_pose, a.split)
    moved = math.hypot(B_pose[0]-A_pose[0], B_pose[1]-A_pose[1])
    print(f'  pose B: moved {moved:.3f} m, heading change {math.degrees(B_pose[2]-A_pose[2]):+.2f} deg, gap over {len(B)} slices')

    # Pair slices that are the same physical place (odom x within 3 cm).
    cA, sA = math.cos(A_pose[2]), math.sin(A_pose[2])
    est = []
    print('    physical slice   dist A   dist B   centre (odom-lateral) A    B      -> delta')
    for ra in A:
        rb = min(B, key=lambda r: math.hypot(r['odom_x']-ra['odom_x'], r['odom_y']-ra['odom_y']), default=None)
        if rb is None: continue
        # compare along the start heading's lateral axis
        latA = -sA*(ra['odom_x']-A_pose[0]) + cA*(ra['odom_y']-A_pose[1])
        latB = -sA*(rb['odom_x']-A_pose[0]) + cA*(rb['odom_y']-A_pose[1])
        fwdA = cA*(ra['odom_x']-A_pose[0]) + sA*(ra['odom_y']-A_pose[1])
        fwdB = cA*(rb['odom_x']-A_pose[0]) + sA*(rb['odom_y']-A_pose[1])
        if abs(fwdA - fwdB) > 0.05: continue
        dx = ra['x_robot'] - rb['x_robot']
        if dx < 0.2: continue
        # apparent lateral = true - x*delta  ->  latB - latA = (xA - xB)*delta
        d = (latB - latA) / dx
        est.append(d)
        print(f'    {fwdA:5.2f} m         {ra["x_robot"]:5.2f}    {rb["x_robot"]:5.2f}    {latA:+.3f}             {latB:+.3f}   {math.degrees(d):+.2f} deg')
    if len(est) < 2: print('  too few matched slices'); return 1
    e = np.array(est)
    print(f'\n  CAMERA YAW ERROR: {math.degrees(np.median(e)):+.2f} deg (median of {len(e)}, '
          f'spread {math.degrees(e.std()):.2f} deg)')
    print(f'  -> at 2 m this shifts obstacles {abs(np.median(e))*2*100:.1f} cm '
          f'{"right" if np.median(e) > 0 else "left"} of where they are')
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
