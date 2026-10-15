"""Shared pre-motion gate: may the robot navigate on its current map pose?

    import nav_gate
    ok, why = nav_gate.ready_to_navigate(node, buf)

Used by nav_goal.py and waypoints.py so every way of sending the robot
somewhere applies the same rule:

1. auto_localise must have confirmed the startup pose (latched
   /labrobot/localised). Without it there is no evidence the pose was ever
   right - after a restart RTAB-Map assumes the robot is where it shut down.
2. The single-view gross-error check (localisation_check) must not say WRONG,
   in case the robot was carried after auto_localise succeeded. This check is
   weak when the view is dominated by objects that moved since mapping, so it
   can refuse a correct pose; that fails safe - re-run auto_localise.
"""
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

import localisation_check


def ready_to_navigate(node, buf, wait_s=3.0, verbose=True):
    lat = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    got = {}
    subs = [node.create_subscription(Bool, '/labrobot/localised',
                                     lambda m: got.__setitem__('ok', m.data), lat),
            node.create_subscription(String, '/labrobot/localisation_status',
                                     lambda m: got.__setitem__('why', m.data), lat)]
    t_end = time.time() + wait_s
    while time.time() < t_end and 'ok' not in got:
        rclpy.spin_once(node, timeout_sec=0.1)
    for s in subs:
        node.destroy_subscription(s)
    if not got.get('ok', False):
        return False, ('startup localisation not confirmed - '
                       + got.get('why', 'no /labrobot/localised (is auto_localise running?)'))
    if verbose:
        print('  checking the current view against the map ...', flush=True)
    r = localisation_check.check(node, buf)
    if r['verdict'] != 'OK':
        return False, (f'current view does not match the map ({r["verdict"]}: {r["why"]}) - '
                       'if the robot was moved, re-run auto_localise')
    return True, f'localised; view check {r["why"]}'
