"""Shared pre-motion gate: may the robot navigate on its current map pose?"""
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
