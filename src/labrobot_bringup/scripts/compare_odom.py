#!/usr/bin/env python3
"""Record wheel odometry and the EKF side by side during a motion.

    ros2 run labrobot_bringup compare_odom.py --seconds 60

Deliberately PASSIVE: it commands nothing. Run it alongside drive_test.py,
teleop, or Nav2 and it reports on whatever motion happens. That keeps one tool
useful for every validation instead of one tool per manoeuvre, and it means the
thing being measured is never also the thing doing the driving.

What this is actually for
-------------------------
The EKF fuses wheel odometry with the gyro (see config/ekf.yaml). Standing still
it is demonstrably better - measured drift fell from 3.52 to 0.04 deg/min. But
stationary performance says nothing about the case that matters, because the two
sensors only disagree when the robot MOVES:

  * On a straight run neither slips much, so the two should track each other
    closely. Divergence here would mean something is wrong - a covariance that
    makes the filter distrust a sensor it should believe, or a frame error.
  * In a spin the swivel castors slip and the wheels over-report rotation. The
    gyro measures rotation directly and cannot slip, so the EKF should come out
    CLOSER to truth than the wheels. That is the entire reason for fusing.

So "agrees on straights, disagrees the right way in turns" is the pass
condition, and disagreement is not automatically a fault. This tool reports the
divergence; only a tape measure or a floor mark says which source was right.

Note both sources are reported in the odom frame from their own start pose, so
they are directly comparable without any transform.
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_of(q) -> float:
    """Yaw from a quaternion. Planar robot, so roll and pitch are ignored."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a: float, b: float) -> float:
    """Signed a-b wrapped to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class OdomCompare(Node):
    def __init__(self, args):
        super().__init__('compare_odom')
        self.args = args
        self.wheel = []   # (t, x, y, yaw, vx, vyaw)
        self.ekf = []
        self.t0 = None

        # BEST_EFFORT, and the compatibility rule runs the opposite way to what
        # an earlier comment here claimed: a BEST_EFFORT subscriber matches BOTH
        # reliable and best-effort publishers, while a RELIABLE subscriber
        # silently fails to match a best-effort publisher and reports a healthy
        # topic as dead. Best-effort is therefore the safe choice for a tool that
        # must work against any odometry source.
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, args.wheel_topic,
                                 lambda m: self._on(m, self.wheel), qos)
        self.create_subscription(Odometry, args.ekf_topic,
                                 lambda m: self._on(m, self.ekf), qos)

    def _on(self, msg, sink):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        p = msg.pose.pose
        sink.append((t - self.t0, p.position.x, p.position.y, yaw_of(p.orientation),
                     msg.twist.twist.linear.x, msg.twist.twist.angular.z))


def resample(series, times):
    """Linear interpolation onto a common time base.

    The two sources publish at different rates and are not synchronised, so
    comparing sample i of one against sample i of the other would fold a timing
    skew into the answer and report it as a pose difference. That exact mistake
    turned ~35 ms of Modbus sampling skew into a phantom "0.87% wheel asymmetry"
    earlier in this project (Problem 11), so it is worth the interpolation.
    """
    a = np.array(series)
    out = {}
    for i, key in enumerate(('x', 'y', 'yaw', 'vx', 'vyaw'), start=1):
        if key == 'yaw':
            # Unwrap before interpolating: averaging across the +/-pi seam would
            # produce a value pointing the opposite way.
            out[key] = np.interp(times, a[:, 0], np.unwrap(a[:, i]))
        else:
            out[key] = np.interp(times, a[:, 0], a[:, i])
    return out


def to_start_frame(d, i0=0):
    """Express a track as forward/lateral/heading relative to its OWN pose at i0.

    Rotating - not merely translating - is essential. The two sources can hold
    completely different absolute headings: carry the robot across the room by
    hand and the gyro integrates the rotation while the wheels, not turning, see
    nothing. Their odom frames are then yawed apart by that angle forever after.
    Comparing raw x/y at that point measures the frame offset, not the robot: one
    such run showed 580 mm of "divergence" on a dead-straight 1 m drive, which was
    entirely 1 m taken along two headings 34 deg apart. Its heading divergence was
    0.13 deg, which is what gave it away.

    drive_test.py carries a comment about this exact trap for the same reason.
    """
    yaw0 = d['yaw'][i0]
    dx, dy = d['x'] - d['x'][i0], d['y'] - d['y'][i0]
    c, s = math.cos(yaw0), math.sin(yaw0)
    return {'fwd': dx * c + dy * s,
            'lat': -dx * s + dy * c,
            'yaw': d['yaw'] - yaw0,
            'vx': d['vx'], 'vyaw': d['vyaw']}


LA = ['wheel', 'ekf']   # display names, set from --labels in main()


def window_metrics(W, E, times, lo, hi, label):
    """Divergence accumulated strictly between two times.

    Reported separately from the full recording because the two are different
    effects and mixing them is misleading. The EKF drifts at the gyro's residual
    bias (~0.5 deg/min) whenever the robot is STILL, so a 45 s recording of a 15 s
    drive folds ~0.4 deg of stationary drift into what looks like a motion error.
    Trimming to the interval the robot was actually moving isolates the thing the
    fusion is supposed to be good at.
    """
    m = np.flatnonzero((times >= lo) & (times <= hi))
    if len(m) < 5:
        return None
    Ws = to_start_frame({k: v[m] for k, v in W.items()})
    Es = to_start_frame({k: v[m] for k, v in E.items()})
    dfwd = Es['fwd'] - Ws['fwd']
    dlat = Es['lat'] - Ws['lat']
    dyaw = np.degrees(Es['yaw'] - Ws['yaw'])
    dpos = np.hypot(dfwd, dlat)
    path = float(np.sum(np.hypot(np.diff(Ws['fwd']), np.diff(Ws['lat']))))
    print(f'  {label} ({hi - lo:.1f} s, wheel path {path:.3f} m)')
    print(f'    forward   {LA[0]} {Ws["fwd"][-1]:+.4f}  {LA[1]} {Es["fwd"][-1]:+.4f} m'
          f'   diff {dfwd[-1]*1000:+7.1f} mm')
    print(f'    lateral   {LA[0]} {Ws["lat"][-1]:+.4f}  {LA[1]} {Es["lat"][-1]:+.4f} m'
          f'   diff {dlat[-1]*1000:+7.1f} mm')
    print(f'    heading   {LA[0]} {math.degrees(Ws["yaw"][-1]):+.3f}  '
          f'{LA[1]} {math.degrees(Es["yaw"][-1]):+.3f} deg   diff {dyaw[-1]:+7.3f} deg')
    print(f'    position divergence : final {dpos[-1]*1000:7.1f} mm   '
          f'max {dpos.max()*1000:7.1f} mm')

    # WHEN the divergence appears matters as much as how big it is. Slip that
    # accumulates evenly with rotation is a traction property of the robot.
    # Slip that appears in a burst at the end is the test tool's own creep phase
    # grinding against caster stiction, and says nothing about normal driving.
    # Without this split the two are indistinguishable in a single final number.
    print('    profile   ', end='')
    n = len(dyaw)
    for frac in (0.2, 0.4, 0.6, 0.8, 1.0):
        i = min(n - 1, int(frac * (n - 1)))
        print(f'{frac*100:.0f}%: {LA[0]} {math.degrees(Ws["yaw"][i]):+.1f} '
              f'diff {dyaw[i]:+.1f}   ', end='')
    print()
    return dict(dpos=dpos[-1], dyaw=dyaw[-1], path=path)


def report(node, args):
    w, e = node.wheel, node.ekf
    print()
    print('=' * 70)
    global LA
    la, lb = (args.labels.split(',') + ['a', 'b'])[:2]
    LA = [la, lb]
    print(f'{la.upper()}  vs  {lb.upper()}')
    print('=' * 70)

    if len(w) < 10 or len(e) < 10:
        print(f'  insufficient data: wheel {len(w)} samples, ekf {len(e)} samples')
        if len(e) < 10:
            print(f'  Nothing on {args.ekf_topic}.')
        return 1

    # Overlapping window only; the two nodes do not start at the same instant.
    t_lo = max(w[0][0], e[0][0])
    t_hi = min(w[-1][0], e[-1][0])
    if t_hi - t_lo < 1.0:
        print('  less than 1 s of overlap between the two sources')
        return 1
    times = np.linspace(t_lo, t_hi, max(50, int((t_hi - t_lo) * 20)))
    W, E = resample(w, times), resample(e, times)

    # Express each source in ITS OWN start frame before comparing. See
    # to_start_frame: without the rotation this measures the angle between two
    # odom frames rather than anything about the robot.
    Wf, Ef = to_start_frame(W), to_start_frame(E)

    dfwd = Ef['fwd'] - Wf['fwd']
    dlat = Ef['lat'] - Wf['lat']
    dpos = np.hypot(dfwd, dlat)
    dyaw = np.degrees(Ef['yaw'] - Wf['yaw'])

    # Path length from the wheel track, used to normalise the divergence: 20 mm
    # over 10 m is excellent and over 0.2 m is alarming.
    path = float(np.sum(np.hypot(np.diff(Wf['fwd']), np.diff(Wf['lat']))))
    rot = float(np.degrees(np.abs(Wf['yaw'][-1])))

    print(f'  duration {t_hi - t_lo:.1f} s   wheel path {path:.3f} m   '
          f'net rotation {rot:.1f} deg')
    print(f'  samples: wheel {len(w)} ({len(w)/(w[-1][0]-w[0][0]):.0f} Hz)   '
          f'ekf {len(e)} ({len(e)/(e[-1][0]-e[0][0]):.0f} Hz)')
    print()
    print(f'  {"":<12}{la:>12}{lb:>12}{"difference":>14}')
    print(f'  {"forward (m)":<12}{Wf["fwd"][-1]:12.4f}{Ef["fwd"][-1]:12.4f}'
          f'{dfwd[-1]:14.4f}')
    print(f'  {"lateral (m)":<12}{Wf["lat"][-1]:12.4f}{Ef["lat"][-1]:12.4f}'
          f'{dlat[-1]:14.4f}')
    print(f'  {"yaw (deg)":<12}{math.degrees(Wf["yaw"][-1]):12.3f}'
          f'{math.degrees(Ef["yaw"][-1]):12.3f}{dyaw[-1]:14.3f}')
    print()
    print(f'  position divergence : final {dpos[-1]*1000:7.1f} mm   '
          f'max {dpos.max()*1000:7.1f} mm')
    print(f'  heading divergence  : final {dyaw[-1]:7.3f} deg   '
          f'max {np.abs(dyaw).max():7.3f} deg')
    if path > 0.05:
        print(f'  normalised          : {dpos[-1]/path*100:.2f}% of path, '
              f'{abs(dyaw[-1])/path:.3f} deg/m')

    # Velocity agreement is a separate and more sensitive check than pose: pose
    # divergence accumulates and so lags the moment things went wrong, whereas a
    # yaw-rate disagreement shows the instant the wheels start slipping.
    dvyaw = np.degrees(Ef['vyaw'] - Wf['vyaw'])
    print()
    print(f'  yaw-rate difference : rms {np.sqrt((dvyaw**2).mean()):.3f} deg/s   '
          f'max {np.abs(dvyaw).max():.3f} deg/s')

    moving = np.abs(W['vyaw']) > 0.05
    if moving.sum() > 10:
        print(f'  while turning       : rms {np.sqrt((dvyaw[moving]**2).mean()):.3f}'
              f' deg/s over {moving.sum()/len(moving)*100:.0f}% of the run')

    # Split the recording into the interval the robot was actually moving and the
    # rest. Without this the EKF's stationary gyro drift is charged against the
    # manoeuvre, which flatters the wheels and libels the filter.
    print()
    active = (np.abs(W['vx']) > 0.01) | (np.abs(W['vyaw']) > 0.02)
    if active.sum() > 5:
        idx = np.flatnonzero(active)
        lo = max(times[0], times[idx[0]] - 0.5)
        hi = min(times[-1], times[idx[-1]] + 0.5)
        window_metrics(W, E, times, lo, hi, 'MOTION ONLY')
        still = (times[-1] - times[0]) - (hi - lo)
        if still > 5.0:
            print(f'  ({still:.0f} s of the recording was stationary and is '
                  'excluded above)')
    else:
        print('  robot never moved; the figures above are pure stationary drift')

    print()
    if rot > 30.0:
        print('  This run included significant rotation, so the two SHOULD differ:')
        print('  the castors slip and the wheels over-report. Compare both against')
        print('  a floor mark - the EKF should be the closer one.')
    elif path > 0.2:
        print('  Near-straight run: the two should agree closely. Large divergence')
        print('  here points at a covariance or frame problem, not at slip.')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wheel-topic', default='/diff_drive_controller/odom')
    ap.add_argument('--ekf-topic', default='/odometry/filtered')
    # The tool is source-agnostic: it compares any two Odometry topics. Naming
    # them lets the same code compare wheel-vs-EKF and EKF-vs-visual without the
    # output claiming "wheel" for something that is not the wheels.
    ap.add_argument('--labels', default='wheel,ekf',
                    help='comma-separated names for the two sources')
    ap.add_argument('--seconds', type=float, default=0.0,
                    help='auto-stop after this long; 0 means run until Ctrl-C')
    # parse_known_args, not parse_args: "--ros-args ..." would otherwise be
    # rejected with exit code 2 and the node would die with only a bare
    # "process has died" in the launch log.
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = OdomCompare(args)
    node.get_logger().info(
        f'recording {args.wheel_topic} vs {args.ekf_topic} - drive the robot now'
        + (f' ({args.seconds:.0f}s)' if args.seconds else ' (Ctrl-C to finish)'))

    rc = 0
    deadline = time.time() + args.seconds if args.seconds else None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if deadline and time.time() >= deadline:
                break
    except KeyboardInterrupt:
        pass

    try:
        rc = report(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
