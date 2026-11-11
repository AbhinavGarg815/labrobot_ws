#!/usr/bin/env python3
"""Follow the target person with Nav2. States: IDLE, ACQUIRE, FOLLOW, LOST, SEARCH, WAIT.

    ros2 run labrobot_follow person_follower.py                  # drives (via Nav2)
    ros2 run labrobot_follow person_follower.py --ros-args -p dry_run:=true

Commands on /labrobot/follow_cmd (std_msgs/String) - Foxglove Publish panel:
    start    lock the nearest person ahead (person_tracker) and follow them
    stop     stop following now (the target is remembered; `start` resumes)
    release  forget the target (person_tracker)
    select N lock person N (ids in the debug image); then `start`
Status, latched, on /labrobot/follow_status (std_msgs/String).

How it drives
  * FOLLOW: a NavigateToPose goal with behavior_tree = follow_person.xml;
    the goal is the STAND-OFF point (standoff_m short of the person on the
    robot->person line, facing them), moved by /goal_update as they walk.
    Inside standoff_m the goal is the robot's own position facing the
    person (turn only, never reverse). Inside min_distance_m the Nav2 goal
    is cancelled and the robot holds.
  * Speed adapts to the person (user decision 2026-11-11): Nav2 speed limit
    = person speed + speed_gain * (distance - standoff), clamped to
    [min_speed, max_speed]. max_speed is the test cap (0.06 by default);
    the velocity smoother's own cap still applies on top.
  * LOST (target not seen for lost_after_s): hold lost_hold_s, then drive to
    where they were last seen, then SEARCH: a full 360 deg turn starting
    towards the side they were last seen (user decision), then WAIT
    watching for search_wait_s, then IDLE ("lost"). Re-acquisition is the
    tracker's re-ID decision only; the follower just sees the target again.
  * Everything goes through Nav2 (planner, controller, velocity smoother,
    collision monitor). Stop paths: `stop`, Ctrl-C here (cancels the goal),
    scripts/stop_robot.sh; the Arduino watchdog brakes if commands stop.

dry_run: computes and publishes goals (/labrobot/follow_goal) and asks the
PLANNER for a path to each (/labrobot/follow_plan) - the robot never moves:
no NavigateToPose, no Spin, no speed limit.
"""

import json
import math
import os
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from labrobot_follow.follow_logic import Costmap, adaptive_speed, standoff_goal, wrap, yaw_quat

IDLE, ACQUIRE, FOLLOW, LOST, SEARCH, WAIT = 'IDLE', 'ACQUIRE', 'FOLLOW', 'LOST', 'SEARCH', 'WAIT'
NO_SPEED_LIMIT = 0.0


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        p = self.declare_parameter
        p('standoff_m', 1.2)
        p('min_distance_m', 0.8)
        p('min_speed', 0.06)
        p('max_speed', 0.06)
        p('speed_gain', 0.3)
        p('goal_deadband_m', 0.15)
        p('goal_deadband_deg', 10.0)
        p('lost_after_s', 1.0)
        p('lost_hold_s', 2.0)
        p('lost_goto_s', 60.0)
        p('search_wait_s', 60.0)
        p('max_range_m', 5.0)
        p('acquire_timeout_s', 4.0)
        p('seed_views', 4)
        p('goal_free_cost', 60)
        p('rate_hz', 5.0)
        p('dry_run', False)
        p('odom_frame', 'odom')
        p('base_frame', 'base_footprint')
        p('behavior_tree', os.path.join(get_package_share_directory('labrobot_follow'),
                                        'behavior_trees', 'follow_person.xml'))
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        for n in ('standoff_m', 'min_distance_m', 'min_speed', 'max_speed', 'speed_gain',
                  'goal_deadband_m', 'lost_after_s', 'lost_hold_s', 'lost_goto_s', 'search_wait_s',
                  'max_range_m', 'acquire_timeout_s', 'seed_views', 'goal_free_cost',
                  'dry_run', 'odom_frame', 'base_frame', 'behavior_tree'):
            setattr(self, n, g(n))
        self.goal_deadband = math.radians(g('goal_deadband_deg'))
        if self.min_distance_m >= self.standoff_m:
            raise ValueError('min_distance_m must be below standoff_m')
        if not os.path.exists(self.behavior_tree):
            raise FileNotFoundError(self.behavior_tree)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, '/labrobot/follow_status', latched)
        self.pub_update = self.create_publisher(PoseStamped, '/goal_update', 10)
        self.pub_speed = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/labrobot/follow_goal', 10)
        self.pub_plan = self.create_publisher(Path, '/labrobot/follow_plan', 1)
        self.create_subscription(String, '/labrobot/follow_cmd', self.on_cmd, 10)
        self.create_subscription(Odometry, '/labrobot/follow_target', self.on_target, 10)
        self.create_subscription(String, '/labrobot/target_state', self.on_state, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_costmap, 1)

        self.state = IDLE
        self.state_t = time.monotonic()
        self.target = None          # (stamp, xy, v)
        self.tstate = {}
        self.tstate_t = 0.0
        self.costmap = None
        self.nav_handle = None
        self.nav_pending = False
        self.nav_fail = 0
        self.nav_retry_t = 0.0
        self.spin_handle = None
        self.spin_pending = False
        self.spin_done = False
        self.plan_pending = False
        self.last_goal = None       # (xy, heading)
        self.last_goal_t = 0.0
        self.last_speed = None
        self.last_seen = None       # (xy, bearing_sign)
        self.lost_phase = None
        self.status_text = None
        self.status_t = 0.0
        self.create_timer(1.0 / g('rate_hz'), self.tick)
        self.set_status('IDLE: send "start" on /labrobot/follow_cmd' +
                        (' (DRY RUN - the robot will not move)' if self.dry_run else ''))

    # ---------------------------------------------------------------- inputs
    def on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            if self.state in (IDLE,):
                self.enter(ACQUIRE)
                self.set_status('ACQUIRE: locking the person ahead (stand still in front, '
                                f'within 3.5 m)')
        elif cmd == 'stop':
            self.halt('stopped by operator')
        elif cmd in ('release', 'reset'):
            self.halt('target released by operator')

    def on_target(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.target = (Time.from_msg(msg.header.stamp), np.array([p.x, p.y]), np.array([v.x, v.y]))

    def on_state(self, msg):
        try:
            self.tstate = json.loads(msg.data)
            self.tstate_t = time.monotonic()
        except ValueError:
            pass

    def on_costmap(self, msg):
        self.costmap = Costmap(msg)

    # ---------------------------------------------------------------- helpers
    def enter(self, state):
        self.state = state
        self.state_t = time.monotonic()

    def set_status(self, text, force=False):
        now = time.monotonic()
        if text != self.status_text or force or now - self.status_t > 5.0:
            self.pub_status.publish(String(data=text))
            if text.split(':')[0] != (self.status_text or '').split(':')[0]:
                self.get_logger().info(text)
            self.status_text = text
            self.status_t = now

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return np.array([t.x, t.y]), yaw

    def target_seen(self):
        fresh = time.monotonic() - self.tstate_t < 0.5
        st = self.tstate
        return (fresh and st.get('track_id') is not None and st.get('state') in ('tracking', 'occluded')
                and st.get('seen_age_s', 99) < self.lost_after_s and self.target is not None)

    def pose_msg(self, xy, heading, stamp=None):
        m = PoseStamped()
        m.header.frame_id = self.odom_frame
        m.header.stamp = (stamp or self.get_clock().now()).to_msg()
        m.pose.position.x, m.pose.position.y = float(xy[0]), float(xy[1])
        m.pose.orientation.z, m.pose.orientation.w = yaw_quat(heading)
        return m

    def free_goal(self, robot_xy, person_xy, goal_xy, heading):
        """Stand-off goal moved round the person if it is in an obstacle."""
        cm = self.costmap
        if cm is None or cm.frame != self.odom_frame:
            return goal_xy, heading
        c = cm.cost(goal_xy, 0.05)
        if c is None or c < self.goal_free_cost:
            return goal_xy, heading
        base = math.atan2(robot_xy[1] - person_xy[1], robot_xy[0] - person_xy[0])
        for dk in (15, -15, 30, -30, 45, -45, 60, -60):
            a = base + math.radians(dk)
            cand = person_xy + self.standoff_m * np.array([math.cos(a), math.sin(a)])
            cc = cm.cost(cand, 0.05)
            if cc is not None and cc < self.goal_free_cost:
                return cand, math.atan2(person_xy[1] - cand[1], person_xy[0] - cand[0])
        # Nowhere free at the stand-off: stay and face them.
        return robot_xy.copy(), math.atan2(person_xy[1] - robot_xy[1], person_xy[0] - robot_xy[0])

    def publish_speed(self, v):
        if self.dry_run:
            return
        if self.last_speed is None or abs(v - self.last_speed) > 0.005:
            self.pub_speed.publish(SpeedLimit(percentage=False, speed_limit=float(v)))
            self.last_speed = v

    def reset_speed(self):
        if not self.dry_run:
            self.pub_speed.publish(SpeedLimit(percentage=False, speed_limit=NO_SPEED_LIMIT))
        self.last_speed = None

    # ---------------------------------------------------------------- nav
    def nav_active(self):
        return self.nav_pending or self.nav_handle is not None

    def send_or_update_goal(self, xy, heading, stamp):
        msg = self.pose_msg(xy, heading, stamp)
        self.pub_goal.publish(msg)
        changed = (self.last_goal is None or
                   np.linalg.norm(xy - self.last_goal[0]) > self.goal_deadband_m or
                   abs(wrap(heading - self.last_goal[1])) > self.goal_deadband)
        now = time.monotonic()
        if self.dry_run:
            if changed or now - self.last_goal_t > 1.0:
                self.plan_only(msg)
                self.last_goal, self.last_goal_t = (xy.copy(), heading), now
            return
        if not self.nav_active():
            if now < self.nav_retry_t:
                return
            if not self.nav.server_is_ready():
                self.set_status('FOLLOW: Nav2 navigate_to_pose not available')
                return
            goal = NavigateToPose.Goal(pose=msg, behavior_tree=self.behavior_tree)
            self.nav_pending = True
            fut = self.nav.send_goal_async(goal)
            fut.add_done_callback(self.on_nav_accepted)
            self.last_goal, self.last_goal_t = (xy.copy(), heading), now
        elif changed or now - self.last_goal_t > 1.0:
            # fresh stamp at least every second, so the goal never goes stale
            self.pub_update.publish(msg)
            self.last_goal, self.last_goal_t = (xy.copy(), heading), now

    def on_nav_accepted(self, fut):
        self.nav_pending = False
        h = fut.result()
        if not h.accepted:
            self.get_logger().warn('navigate_to_pose goal rejected')
            self.nav_fail += 1
            self.nav_retry_t = time.monotonic() + 1.0
            return
        if self.state not in (FOLLOW, LOST):
            h.cancel_goal_async()
            return
        self.nav_handle = h
        h.get_result_async().add_done_callback(lambda f, hh=h: self.on_nav_result(hh, f))

    def on_nav_result(self, handle, fut):
        if handle is not self.nav_handle:
            return
        self.nav_handle = None
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_fail = 0
            if self.state == LOST and self.lost_phase == 'goto':
                self.lost_phase = 'arrived'
        elif status == GoalStatus.STATUS_ABORTED:
            self.nav_fail += 1
            self.nav_retry_t = time.monotonic() + 1.0
            self.get_logger().warn(f'follow navigation aborted ({self.nav_fail} in a row)')
            if self.state == LOST and self.lost_phase == 'goto':
                self.lost_phase = 'arrived'

    def cancel_nav(self):
        if self.nav_handle is not None:
            self.nav_handle.cancel_goal_async()
            self.nav_handle = None
        self.last_goal = None

    def plan_only(self, goal_msg):
        if self.plan_pending or not self.planner.server_is_ready():
            return
        self.plan_pending = True
        fut = self.planner.send_goal_async(ComputePathToPose.Goal(goal=goal_msg, planner_id='GridBased'))

        def accepted(f):
            h = f.result()
            if not h.accepted:
                self.plan_pending = False
                return
            h.get_result_async().add_done_callback(done)

        def done(f):
            self.plan_pending = False
            r = f.result()
            if r.status == GoalStatus.STATUS_SUCCEEDED:
                path = r.result.path
                self.pub_plan.publish(path)
                self.last_plan = (len(path.poses), 'ok')
            else:
                self.last_plan = (0, 'FAILED')
        fut.add_done_callback(accepted)

    def start_spin(self, direction):
        if self.dry_run:
            self.spin_done = False
            self.spin_dry_until = time.monotonic() + 45.0
            return
        if not self.spin.server_is_ready():
            self.spin_done = True
            return
        goal = Spin.Goal()
        goal.target_yaw = float(direction * 2 * math.pi)
        goal.time_allowance = Duration(seconds=120).to_msg()
        self.spin_pending = True
        self.spin_done = False

        def accepted(f):
            self.spin_pending = False
            h = f.result()
            if not h.accepted:
                self.spin_done = True
                return
            if self.state != SEARCH:
                h.cancel_goal_async()
                return
            self.spin_handle = h
            h.get_result_async().add_done_callback(spun)

        def spun(f):
            self.spin_handle = None
            self.spin_done = True
            st = f.result().status
            if st != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f'search spin ended with status {st}')
        self.spin.send_goal_async(goal).add_done_callback(accepted)

    def cancel_spin(self):
        if self.spin_handle is not None:
            self.spin_handle.cancel_goal_async()
            self.spin_handle = None

    def halt(self, reason):
        self.cancel_nav()
        self.cancel_spin()
        self.reset_speed()
        self.enter(IDLE)
        self.set_status(f'IDLE: {reason}', force=True)

    # ---------------------------------------------------------------- tick
    def tick(self):
        if self.state == IDLE:
            return
        rp = self.robot_pose()
        if rp is None:
            self.halt('no odom -> base_footprint transform')
            return
        robot_xy, robot_yaw = rp
        now = time.monotonic()
        seen = self.target_seen()

        if self.state == ACQUIRE:
            st = self.tstate
            if st.get('track_id') is not None and st.get('gallery', 0) >= self.seed_views and seen:
                self.enter(FOLLOW)
                self.nav_fail = 0
            elif now - self.state_t > self.acquire_timeout_s:
                self.halt('no person to follow (nobody locked within '
                          f'{self.acquire_timeout_s:.0f} s; stand 1-3.5 m in front and send start)')
            return

        if self.state in (LOST, SEARCH, WAIT) and seen:
            # the LOST drive uses the default tree, which ignores /goal_update
            self.cancel_spin()
            self.cancel_nav()
            self.enter(FOLLOW)
            self.get_logger().info('target seen again - following')

        if self.state == FOLLOW:
            if not seen:
                self.lose(robot_xy)
                return
            self.follow(robot_xy, robot_yaw)
        elif self.state == LOST:
            self.lost_tick(robot_xy, now)
        elif self.state == SEARCH:
            done = self.spin_done or (self.dry_run and now > getattr(self, 'spin_dry_until', 0))
            self.set_status('SEARCH: turning 360 deg' + (' (dry run)' if self.dry_run else ''))
            if done and not self.spin_pending:
                self.enter(WAIT)
        elif self.state == WAIT:
            left = self.search_wait_s - (now - self.state_t)
            self.set_status(f'WAIT: watching for the person ({left:.0f} s left)')
            if left <= 0:
                self.halt('target lost (searched and waited); send start to follow someone')

    def follow(self, robot_xy, robot_yaw):
        stamp, pxy, pv = self.target
        d = float(np.linalg.norm(pxy - robot_xy))
        bearing = wrap(math.atan2(pxy[1] - robot_xy[1], pxy[0] - robot_xy[0]) - robot_yaw)
        self.last_seen = (pxy.copy(), 1.0 if bearing >= 0 else -1.0)
        speed = float(np.linalg.norm(pv))
        tid = self.tstate.get('track_id')
        side = f'{abs(math.degrees(bearing)):.0f} deg {"left" if bearing >= 0 else "right"}'
        if d < self.min_distance_m:
            self.cancel_nav()
            self.set_status(f'FOLLOW: holding - person {tid} too close ({d:.2f} m < '
                            f'{self.min_distance_m:.2f})')
            return
        gxy, gh = standoff_goal(robot_xy, pxy, self.standoff_m)
        gxy, gh = self.free_goal(robot_xy, pxy, gxy, gh)
        v = adaptive_speed(speed, d, self.standoff_m, self.speed_gain, self.min_speed, self.max_speed)
        self.publish_speed(v)
        self.send_or_update_goal(gxy, gh, stamp)
        far = ' - FALLING BEHIND, please wait' if d > self.max_range_m else ''
        plan = ''
        if self.dry_run and hasattr(self, 'last_plan'):
            plan = f'; plan {self.last_plan[1]} ({self.last_plan[0]} poses)'
        fails = f'; {self.nav_fail} nav failures in a row' if self.nav_fail else ''
        self.set_status(f'FOLLOW: person {tid} at {d:.1f} m, {side}, walking {speed:.2f} m/s; '
                        f'speed {v:.2f} m/s{far}{plan}{fails}'
                        + (' (DRY RUN)' if self.dry_run else ''))

    def lose(self, robot_xy):
        self.enter(LOST)
        self.lost_phase = 'hold'
        self.cancel_nav()
        self.set_status('LOST: holding' + (f' (last seen {np.linalg.norm(self.last_seen[0] - robot_xy):.1f} m away)'
                                           if self.last_seen else ''))

    def lost_tick(self, robot_xy, now):
        t = now - self.state_t
        if self.lost_phase == 'hold':
            if t < self.lost_hold_s:
                return
            if self.last_seen is None:
                self.lost_phase = 'arrived'
            else:
                pxy = self.last_seen[0]
                d = float(np.linalg.norm(pxy - robot_xy))
                if d <= self.standoff_m + 0.2:
                    self.lost_phase = 'arrived'
                else:
                    # go to where they were last seen, stopping 0.5 m short
                    gxy, gh = standoff_goal(robot_xy, pxy, 0.5)
                    self.lost_phase = 'goto'
                    self.lost_goal = (gxy, gh)
                    self.set_status(f'LOST: going to where the person was last seen ({d:.1f} m)')
        if self.lost_phase == 'goto':
            gxy, gh = self.lost_goal
            msg = self.pose_msg(gxy, gh)
            self.pub_goal.publish(msg)
            if self.dry_run:
                if self.last_goal is None:
                    self.plan_only(msg)
                    self.last_goal = (gxy.copy(), gh)
                if t > self.lost_hold_s + 5.0:
                    self.lost_phase = 'arrived'
            elif not self.nav_active() and self.last_goal is None:
                # a plain NavigateToPose (default tree): it SUCCEEDS on arrival
                self.nav_pending = True
                self.nav.send_goal_async(NavigateToPose.Goal(pose=msg)).add_done_callback(
                    self.on_nav_accepted)
                self.last_goal = (gxy.copy(), gh)
            elif not self.nav_active():
                self.lost_phase = 'arrived'     # finished (succeeded or aborted)
            if np.linalg.norm(gxy - robot_xy) < 0.35 or t > self.lost_hold_s + self.lost_goto_s:
                self.lost_phase = 'arrived'
        if self.lost_phase == 'arrived':
            self.cancel_nav()
            direction = self.last_seen[1] if self.last_seen else 1.0
            self.enter(SEARCH)
            self.start_spin(direction)
            self.set_status(f'SEARCH: turning 360 deg {"left" if direction > 0 else "right"}')

    def shutdown(self):
        self.cancel_nav()
        self.cancel_spin()
        self.reset_speed()
        self.pub_status.publish(String(data='IDLE: follower stopped'))


def main():
    rclpy.init()
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
            # let the cancel and the speed reset leave before the context goes
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
