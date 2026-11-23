#!/usr/bin/env python3
"""Do the wheels actually do what they were told?

    ros2 run labrobot_bringup verify_wheels.py --seconds 60
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState


def get_params(node, names, defaults):
    """Read the controller's own calibration; fall back if it is not running."""
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, '/diff_drive_controller/get_parameters')
    out = dict(defaults)
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn('controller params unavailable; using defaults')
        return out, False
    req = GetParameters.Request(); req.names = list(names)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    if res is None:
        return out, False
    for name, val in zip(names, res.values):
        if val.type == 3:      # PARAMETER_DOUBLE
            out[name] = val.double_value
    return out, True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--cmd-topic',
                    default='/diff_drive_controller/cmd_vel_unstamped')
    ap.add_argument('--csv', default=None,
                    help='write per-timestamp commanded vs measured wheel '
                         'velocities here, for inspecting the raw feedback')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    n = Node('verify_wheels')

    names = ['wheel_separation', 'wheel_separation_multiplier', 'wheel_radius',
             'left_wheel_radius_multiplier', 'right_wheel_radius_multiplier']
    defaults = {'wheel_separation': 0.274, 'wheel_separation_multiplier': 0.9741,
                'wheel_radius': 0.050, 'left_wheel_radius_multiplier': 1.00480,
                'right_wheel_radius_multiplier': 1.00212}
    p, live = get_params(n, names, defaults)
    ws = p['wheel_separation'] * p['wheel_separation_multiplier']
    rL = p['wheel_radius'] * p['left_wheel_radius_multiplier']
    rR = p['wheel_radius'] * p['right_wheel_radius_multiplier']

    qos = QoSProfile(depth=400, reliability=ReliabilityPolicy.BEST_EFFORT)
    cmds, js = [], []
    n.create_subscription(Twist, args.cmd_topic,
                          lambda m: cmds.append((time.time(), m.linear.x, m.angular.z)),
                          qos)

    def on_js(m):
        try:
            li = m.name.index('left_wheel_joint'); ri = m.name.index('right_wheel_joint')
        except ValueError:
            return
        if len(m.velocity) > max(li, ri):
            js.append((time.time(), m.velocity[li], m.velocity[ri]))

    n.create_subscription(JointState, '/joint_states', on_js, qos)

    gyro = []
    n.create_subscription(Imu, '/imu/unbiased',
                          lambda m: gyro.append((time.time(),
                                                 m.angular_velocity.x,
                                                 m.angular_velocity.y,
                                                 m.angular_velocity.z)),
                          qos)
    imu_frame = {'id': None}
    n.create_subscription(Imu, '/imu/unbiased',
                          lambda m: imu_frame.__setitem__('id', m.header.frame_id),
                          qos)
    tf_buf = tf2_ros.Buffer(); tf2_ros.TransformListener(tf_buf, n)

    ex = SingleThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    print(f'  kinematics {"from the running controller" if live else "(DEFAULTS)"}: '
          f'ws={ws:.5f} m  rL={rL:.5f} m  rR={rR:.5f} m')
    print(f'  recording {args.seconds:.0f} s - drive the robot now')
    time.sleep(args.seconds)

    print('\n' + '=' * 70)
    print('WHEEL TRACKING: commanded vs measured, same timestamp')
    print('=' * 70)
    if len(cmds) < 20 or len(js) < 20:
        print(f'  insufficient data: {len(cmds)} commands, {len(js)} joint states')
        sys.stdout.flush(); os._exit(1)

    C = np.array(cmds); J = np.array(js)
    lo = max(C[0, 0], J[0, 0]); hi = min(C[-1, 0], J[-1, 0])
    if hi - lo < 2.0:
        print('  less than 2 s of overlap'); sys.stdout.flush(); os._exit(1)
    t = np.linspace(lo, hi, int((hi - lo) * 50))

    v = np.interp(t, C[:, 0], C[:, 1])
    w = np.interp(t, C[:, 0], C[:, 2])
    idx = np.searchsorted(C[:, 0], t, side='right') - 1
    idx = np.clip(idx, 0, len(C) - 1)
    v = C[idx, 1]; w = C[idx, 2]

    aL = np.interp(t, J[:, 0], J[:, 1])
    aR = np.interp(t, J[:, 0], J[:, 2])

    eL = (v - w * ws / 2.0) / rL
    eR = (v + w * ws / 2.0) / rR

    active = (np.abs(eL) > 0.1) | (np.abs(eR) > 0.1)
    na = int(active.sum())
    print(f'\n  {na} of {len(t)} samples commanded to move '
          f'({100*na/len(t):.0f}% of the window)')
    if na < 20:
        print('  robot barely moved'); sys.stdout.flush(); os._exit(1)

    # Lag, by cross-correlation on the mean wheel speed.
    def lag_of(exp, act):
        """Cross-correlation lag in seconds, or None when it is not measurable."""
        e = exp - exp.mean(); a = act - act.mean()
        if np.allclose(e, 0) or np.allclose(a, 0):
            return None
        # A command that barely varies carries no timing information.
        if e.std() < 0.05 * max(abs(exp).mean(), 1e-9):
            return None
        cc = np.correlate(a, e, 'full')
        k = int(np.argmax(cc))
        if cc.max() <= 0 or cc.max() < 1.15 * np.abs(cc).mean():
            return None
        return (k - (len(e) - 1)) / 50.0
    lag = lag_of((eL + eR)[active] / 2, (aL + aR)[active] / 2)
    if lag is None:
        print('  drivetrain lag : NOT MEASURABLE (command too steady - '
              'cross-correlation needs a varying signal)')
        lag = 0.0
    else:
        print(f'  drivetrain lag (cross-correlation) : {lag*1000:+.0f} ms')

    sh = int(round(lag * 50))
    if sh > 0:
        eLs, eRs, aLs, aRs, m = eL[:-sh], eR[:-sh], aL[sh:], aR[sh:], active[:-sh]
    elif sh < 0:
        eLs, eRs, aLs, aRs, m = eL[-sh:], eR[-sh:], aL[:sh], aR[:sh], active[-sh:]
    else:
        eLs, eRs, aLs, aRs, m = eL, eR, aL, aR, active

    print(f'\n  {"wheel":<8}{"cmd rms":>10}{"act rms":>10}{"err rms":>10}'
          f'{"err mean":>10}{"scale":>9}')
    for nm, e, a in (('left', eLs[m], aLs[m]), ('right', eRs[m], aRs[m])):
        err = a - e
        scale = float(np.dot(a, e) / np.dot(e, e)) if np.dot(e, e) > 0 else float('nan')
        print(f'  {nm:<8}{np.sqrt((e**2).mean()):10.3f}{np.sqrt((a**2).mean()):10.3f}'
              f'{np.sqrt((err**2).mean()):10.3f}{err.mean():+10.3f}{scale:9.3f}')
    print('  (rad/s at the wheel)')

    straight = m & (np.abs(w[:len(m)] if len(w) >= len(m) else w) < 0.02) \
                 & (np.abs(v[:len(m)] if len(v) >= len(m) else v) > 0.01)
    if straight.sum() > 20:
        sl, sr = aLs[straight], aRs[straight]
        el_, er_ = eLs[straight], eRs[straight]
        print(f'\n  STRAIGHT segments only ({straight.sum()} samples):')
        print(f'    commanded L/R : {el_.mean():.3f} / {er_.mean():.3f} rad/s  '
              f'ratio {el_.mean()/er_.mean() if er_.mean() else float("nan"):.3f}')
        print(f'    measured  L/R : {sl.mean():.3f} / {sr.mean():.3f} rad/s  '
              f'ratio {sl.mean()/sr.mean() if sr.mean() else float("nan"):.3f}')
        asym = (sl.mean() - sr.mean()) / max(abs(sl.mean() + sr.mean()) / 2, 1e-9)
        print(f'    asymmetry     : {100*asym:+.1f}%   '
              '(0% = both wheels equal; + = left faster)')
    else:
        print('\n  no sustained straight segments in this run')

    # separate actuation lag from measurement lag
    print()
    if len(gyro) > 50 and imu_frame['id']:
        try:
            tr = tf_buf.lookup_transform('base_footprint', imu_frame['id'],
                                         rclpy.time.Time())
            q = tr.transform.rotation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            row = np.array([2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw),
                            1 - 2*(qx*qx + qy*qy)])
        except Exception as e:  # noqa: BLE001
            print(f'  cannot transform {imu_frame["id"]} -> base_footprint: {e}')
            sys.stdout.flush(); os._exit(1)
        print(f'  gyro frame {imu_frame["id"]}: yaw = '
              f'{row[0]:+.3f}*gx {row[1]:+.3f}*gy {row[2]:+.3f}*gz')
        G = np.array(gyro)
        gz = (np.interp(t, G[:, 0], G[:, 1]) * row[0]
              + np.interp(t, G[:, 0], G[:, 2]) * row[1]
              + np.interp(t, G[:, 0], G[:, 3]) * row[2])
        # Angular velocity the encoders imply, and the one that was commanded.
        w_enc = (aR * rR - aL * rL) / ws
        w_cmd = w
        turning = np.abs(w_cmd) > 0.05
        if turning.sum() > 40:
            lag_enc = lag_of(w_cmd[turning], w_enc[turning])
            lag_gyro = lag_of(w_cmd[turning], gz[turning])
            print(f'  LAG BREAKDOWN (from {int(turning.sum())} turning samples)')
            if lag_enc is None or lag_gyro is None:
                # Cross-correlation needs a varying signal.
                print('    NOT MEASURABLE - the commanded rotation was too steady.')
                print('    Use a manoeuvre with a CHANGING rate (a Nav2 goal), not')
                print('    a constant spin.')
            else:
                print(f'    command -> GYRO     : {lag_gyro*1000:+5.0f} ms   '
                      '= actuation lag')
                print(f'    command -> ENCODERS : {lag_enc*1000:+5.0f} ms   '
                      '= actuation + measurement')
                print(f'    difference          : {(lag_enc-lag_gyro)*1000:+5.0f} ms   '
                      '= how STALE the encoder feedback is')
                print()
                if lag_enc == 0.0 and lag_gyro == 0.0:
                    print('    -> NO MEASURABLE LAG in either path. Nothing to attribute.')
                elif int(turning.sum()) < 100:
                    print(f'    -> only {int(turning.sum())} turning samples; too few')
                    print('       to attribute confidently.')
                elif abs(lag_gyro) > abs(lag_enc - lag_gyro):
                    print('    -> dominated by ACTUATION. The drivetrain genuinely')
                    print('       responds late; slow the controller to match it.')
                else:
                    print('    -> dominated by MEASUREMENT. The robot moves on time but')
                    print('       odometry reports it late, so the EKF pose lags and')
                    print('       Nav2 overshoots. Fix the feedback path.')
            m2 = turning
            gain = float(np.dot(gz[m2], w_cmd[m2]) / max(np.dot(w_cmd[m2], w_cmd[m2]), 1e-9))
            print(f'    gyro/command gain   : {gain:.3f}  '
                  '(1.00 = robot turns exactly as fast as asked)')
        else:
            print('  not enough turning to separate actuation from measurement lag')
    else:
        print('  no /imu/unbiased - cannot separate actuation from measurement lag')

    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('t_rel,cmd_v,cmd_w,left_cmd,left_meas,left_err,'
                    'right_cmd,right_meas,right_err\n')
            for i in range(len(t)):
                f.write(f'{t[i]-t[0]:.3f},{v[i]:.4f},{w[i]:.4f},'
                        f'{eL[i]:.4f},{aL[i]:.4f},{aL[i]-eL[i]:.4f},'
                        f'{eR[i]:.4f},{aR[i]:.4f},{aR[i]-eR[i]:.4f}\n')
        print(f'  raw per-timestamp wheel feedback -> {args.csv} ({len(t)} rows)')

    print()
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
