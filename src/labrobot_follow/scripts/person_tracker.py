#!/usr/bin/env python3
"""Person perception for following: detect, track, identify the target, 3D in odom.

    ros2 run labrobot_follow person_tracker.py --ros-args -p detector_engine:=...

Per colour + aligned-depth frame (GPU for both networks):
  1. YOLO pose (TensorRT FP16): person boxes + 17 keypoints;
  2. OSNet re-ID (TensorRT FP16, batched): one appearance vector per person;
  3. BoT-SORT-style tracker, with camera rotation from TF compensating the
     predicted boxes (tracker.py);
  4. per track: depth mode over the hips/thighs region -> point in `odom`;
  5. target identity: online appearance memory, re-acquisition by re-ID only
     (identity.py); target position filtered by a constant-velocity Kalman
     filter in `odom` (geometry.TargetFilter).
It never commands the robot. The follower (person_follower.py) decides what
to do with the target.

Topics
  in   /camera/camera/color/image_raw                     sensor_msgs/Image rgb8
       /camera/camera/aligned_depth_to_color/image_raw    sensor_msgs/Image 16UC1 (mm)
       /camera/camera/color/camera_info
       /labrobot/follow_cmd     std_msgs/String  "select <id>", "start" (lock the
                                nearest person in front if none), "release"
       /clicked_point           geometry_msgs/PointStamped (Foxglove 3D "Publish
                                point"): lock the person nearest the click
  out  /labrobot/people         vision_msgs/Detection3DArray, odom: every
                                confirmed track; class_id "target" or "person"
       /labrobot/follow_target  nav_msgs/Odometry, odom: target KF state
                                (twist in odom: child_frame_id = odom)
       /labrobot/target_state   std_msgs/String JSON, every frame: state,
                                track id, range, bearing, re-ID score, timing
       /labrobot/follow_debug/compressed  sensor_msgs/CompressedImage (only
                                while subscribed, <= debug_rate Hz)
       /labrobot/follow_markers visualization_msgs/MarkerArray (while subscribed)
"""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from labrobot_follow import geometry as geo
from labrobot_follow.detector import PoseDetector, ReidEncoder, mark_clean
from labrobot_follow.identity import TargetIdentity
from labrobot_follow.tracker import PersonTracker, rotation_homography

CHI2_2D_99 = 9.21


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Stage:
    """Running mean of stage times, for the periodic performance log."""

    def __init__(self):
        self.sum = {}
        self.n = 0

    def add(self, times):
        for k, v in times.items():
            self.sum[k] = self.sum.get(k, 0.0) + v
        self.n += 1

    def report(self):
        r = {k: round(v / max(self.n, 1), 1) for k, v in self.sum.items()}
        self.sum, self.n = {}, 0
        return r


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('person_tracker')
        p = self.declare_parameter
        p('detector_engine', '')
        p('reid_engine', '')
        p('colour_topic', '/camera/camera/color/image_raw')
        p('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        p('info_topic', '/camera/camera/color/camera_info')
        p('odom_frame', 'odom')
        p('base_frame', 'base_footprint')
        p('det_conf', 0.15)
        p('min_box_height_px', 40)
        p('select_max_bearing_deg', 20.0)
        p('select_max_range_m', 3.5)
        p('click_radius_m', 0.8)
        p('reid_gate_base_m', 1.0)
        p('reid_gate_speed_mps', 1.5)
        p('reid_thresh', 0.60)
        p('reid_margin', 0.05)
        p('debug_rate', 5.0)
        p('perf_log_s', 10.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.det_conf = g('det_conf')
        self.min_box_h = g('min_box_height_px')
        self.sel_bearing = math.radians(g('select_max_bearing_deg'))
        self.sel_range = g('select_max_range_m')
        self.click_radius = g('click_radius_m')
        self.gate_base = g('reid_gate_base_m')
        self.gate_speed = g('reid_gate_speed_mps')
        self.debug_period = 1.0 / max(g('debug_rate'), 0.1)
        self.perf_log_s = g('perf_log_s')

        if not g('detector_engine') or not g('reid_engine'):
            raise RuntimeError('detector_engine and reid_engine parameters are required')
        self.detector = PoseDetector(g('detector_engine'), conf_thresh=self.det_conf)
        self.reid = ReidEncoder(g('reid_engine'))
        self.tracker = PersonTracker()
        self.identity = TargetIdentity(reid_thresh=g('reid_thresh'), reid_margin=g('reid_margin'))
        self.kf = geo.TargetFilter()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.K = None
        self.prev_R = None
        self.pos3d = {}           # track id -> (stamp, xy in odom, 2x2 cov, range, bearing)
        self.pending_select = None    # ('nearest',) | ('id', n) | ('point', xy)
        self.last_debug = 0.0
        self.stage = Stage()
        self.last_perf = time.monotonic()
        self.frames = 0
        self.dropped_tf = 0

        self.create_subscription(CameraInfo, g('info_topic'), self.on_info, qos_profile_sensor_data)
        self.sub_rgb = Subscriber(self, Image, g('colour_topic'), qos_profile=qos_profile_sensor_data)
        self.sub_depth = Subscriber(self, Image, g('depth_topic'), qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth], queue_size=4, slop=0.02)
        self.sync.registerCallback(self.on_frame)
        self.create_subscription(String, '/labrobot/follow_cmd', self.on_cmd, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)

        self.pub_people = self.create_publisher(Detection3DArray, '/labrobot/people', 10)
        self.pub_target = self.create_publisher(Odometry, '/labrobot/follow_target', 10)
        self.pub_state = self.create_publisher(String, '/labrobot/target_state', 10)
        self.pub_debug = self.create_publisher(CompressedImage, '/labrobot/follow_debug/compressed', 1)
        self.pub_markers = self.create_publisher(MarkerArray, '/labrobot/follow_markers', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_event = self.create_publisher(String, '/labrobot/target_event', latched)
        self.get_logger().info(f'person_tracker ready: detector {g("detector_engine")}, '
                               f're-ID {g("reid_engine")}')

    # ------------------------------------------------------------------ inputs
    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=float).reshape(3, 3)
            self.get_logger().info(f'camera fx {self.K[0, 0]:.1f} cx {self.K[0, 2]:.1f} '
                                   f'cy {self.K[1, 2]:.1f} ({msg.width}x{msg.height})')

    def on_cmd(self, msg):
        words = msg.data.strip().lower().split()
        if not words:
            return
        if words[0] == 'select' and len(words) == 2 and words[1].isdigit():
            self.pending_select = ('id', int(words[1]))
        elif words[0] == 'start' and self.identity.target_id is None and not self.identity.has_model:
            self.pending_select = ('nearest',)
        elif words[0] in ('release', 'reset'):
            self.identity.reset()
            self.kf.reset()
            self.event('released by operator')

    def on_click(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, msg.header.frame_id, Time(),
                                                 timeout=Duration(seconds=0.2))
        except TransformException as e:
            self.get_logger().warn(f'clicked point not usable: {e}')
            return
        R = quat_to_rot(tf.transform.rotation)
        t = tf.transform.translation
        p = R @ np.array([msg.point.x, msg.point.y, msg.point.z]) + [t.x, t.y, t.z]
        self.pending_select = ('point', p[:2])

    def event(self, text):
        self.get_logger().info(f'target: {text}')
        self.pub_event.publish(String(data=text))

    # ------------------------------------------------------------------ frame
    def lookup_cam(self, frame_id, stamp):
        """odom <- camera optical frame at the image time: (R, t) or None."""
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, frame_id, stamp,
                                                 timeout=Duration(seconds=0.05))
        except TransformException:
            return None
        t = tf.transform.translation
        return quat_to_rot(tf.transform.rotation), np.array([t.x, t.y, t.z])

    def on_frame(self, rgb_msg, depth_msg):
        if self.K is None:
            return
        if rgb_msg.encoding != 'rgb8' or depth_msg.encoding not in ('16UC1', 'mono16'):
            self.get_logger().error(f'unexpected encodings {rgb_msg.encoding} / '
                                    f'{depth_msg.encoding}', throttle_duration_sec=10.0)
            return
        t0 = time.perf_counter()
        times = {}
        stamp = Time.from_msg(rgb_msg.header.stamp)
        ts = stamp.nanoseconds * 1e-9
        h, w = rgb_msg.height, rgb_msg.width
        rgb = np.frombuffer(rgb_msg.data, np.uint8).reshape(h, rgb_msg.step)[:, :w * 3].reshape(h, w, 3)
        depth = np.frombuffer(depth_msg.data, np.uint16).reshape(
            depth_msg.height, depth_msg.step // 2)[:, :depth_msg.width]

        cam = self.lookup_cam(rgb_msg.header.frame_id, stamp)
        if cam is None:
            self.dropped_tf += 1
            return
        R, tcam = cam

        t1 = time.perf_counter()
        dets = [d for d in self.detector(rgb) if d['box'][3] - d['box'][1] >= self.min_box_h]
        times['detect'] = (time.perf_counter() - t1) * 1000
        mark_clean(dets, w)
        t1 = time.perf_counter()
        if dets:
            feats = self.reid(rgb, [d['box'] for d in dets])
            for d, f in zip(dets, feats):
                d['feat'] = f
        times['reid'] = (time.perf_counter() - t1) * 1000

        t1 = time.perf_counter()
        H = None
        if self.prev_R is not None:
            H = rotation_homography(self.K, R.T @ self.prev_R)
        self.prev_R = R
        tracks = self.tracker.update(dets, ts, H)

        # 3D for every track matched in this frame
        self.pos3d = {k: v for k, v in self.pos3d.items() if ts - v[0] < 3.0}
        for tr in tracks:
            if not tr.matched_this_frame:
                continue
            m = self.measure(tr.det, depth, R, tcam, w, h)
            if m is not None:
                self.pos3d[tr.id] = (ts,) + m
        times['track'] = (time.perf_counter() - t1) * 1000

        self.apply_selection(tracks, ts)
        alive = {t.id for t in self.tracker.tracks}
        event, tid = self.identity.step(tracks, ts, plausible=lambda tr: self.plausible(tr, ts),
                                        alive_ids=alive)
        state = self.update_target(event, tid, tracks, ts)
        times['total'] = (time.perf_counter() - t0) * 1000
        latency = (self.get_clock().now() - stamp).nanoseconds * 1e-6
        times['latency'] = latency
        self.stage.add(times)
        self.frames += 1

        self.publish(rgb_msg.header, tracks, state, rgb, times)
        self.perf_log()

    def measure(self, det, depth, R, tcam, w, h):
        region, (u, v) = geo.core_region(det['box'], det.get('kpts'), img_w=w, img_h=h)
        z, n = geo.body_depth(depth, region, scale=0.001)
        if z is None:
            return None
        z += geo.BODY_HALF_DEPTH_M
        p_cam = geo.backproject(u, v, z, self.K)
        p_odom = R @ p_cam + tcam
        ray = p_odom[:2] - tcam[:2]
        rng = float(np.linalg.norm(ray))
        if rng < 1e-3:
            return None
        s_along, s_across = geo.measurement_cov_cam(p_cam, self.K[0, 0])
        cov = geo.ground_cov(ray / rng, s_along, s_across)
        bearing = math.atan2(-p_cam[0], p_cam[2])      # + = person to the robot's left
        return p_odom[:2], cov, float(math.hypot(p_cam[0], p_cam[2])), bearing

    def plausible(self, tr, ts):
        """3D gate for re-acquisition: could the target have walked here?"""
        if not self.kf.initialised:
            return True
        m = self.pos3d.get(tr.id)
        if m is None or ts - m[0] > 0.5:
            return False
        last = self.kf.position_at(self.kf.last_meas_t, 0.0)
        dt = max(ts - self.kf.last_meas_t, 0.0)
        return float(np.linalg.norm(m[1] - last)) <= self.gate_base + self.gate_speed * dt

    def apply_selection(self, tracks, ts):
        sel = self.pending_select
        if sel is None:
            return
        self.pending_select = None
        cand = [(tr, self.pos3d[tr.id]) for tr in tracks
                if tr.id in self.pos3d and ts - self.pos3d[tr.id][0] < 0.5]
        chosen = None
        if sel[0] == 'id':
            chosen = next((tr for tr in tracks if tr.id == sel[1]), None)
            if chosen is None:
                self.event(f'select {sel[1]}: no such person in view')
                return
        elif sel[0] == 'nearest':
            front = [(m[3], tr) for tr, m in cand
                     if abs(m[4]) <= self.sel_bearing and m[3] <= self.sel_range]
            if not front:
                self.event('start: nobody within '
                           f'{self.sel_range:.1f} m and +/-{math.degrees(self.sel_bearing):.0f} deg ahead')
                return
            chosen = min(front, key=lambda x: x[0])[1]
        elif sel[0] == 'point':
            near = [(float(np.linalg.norm(m[1] - sel[1])), tr) for tr, m in cand]
            near = [x for x in near if x[0] <= self.click_radius]
            if not near:
                self.event(f'click: nobody within {self.click_radius:.1f} m of it')
                return
            chosen = min(near, key=lambda x: x[0])[1]
        self.identity.lock(chosen.id)
        self.kf.reset()
        self.event(f'locked on person {chosen.id}')

    def update_target(self, event, tid, tracks, ts):
        """Feed the KF and return the state dict published on target_state."""
        if event == 'switch':
            self.event('identity switch suspected - target dropped, searching by appearance')
        elif event == 'reacquired':
            self.kf.reset()
            self.event(f're-identified as person {tid}')
        st = {'event': event, 'track_id': self.identity.target_id, 'gallery':
              len(self.identity.short) + len(self.identity.long)}
        tgt = None
        if self.identity.target_id is not None:
            tgt = next((t for t in tracks if t.id == self.identity.target_id), None)
        if tgt is not None and tgt.matched_this_frame and tgt.id in self.pos3d \
                and self.pos3d[tgt.id][0] == ts:
            _, xy, cov, rng, bearing = self.pos3d[tgt.id]
            if not self.kf.initialised:
                self.kf.init(xy, cov, ts)
            else:
                self.kf.predict(ts)
                if self.kf.mahalanobis2(xy, cov) <= CHI2_2D_99 * 4:
                    self.kf.update(xy, cov, ts)
                else:
                    st['rejected_measurement'] = True
            st.update(range=round(rng, 3), bearing_deg=round(math.degrees(bearing), 1))
            if tgt.det.get('feat') is not None and self.identity.has_model:
                st['reid_score'] = round(self.identity.score(tgt.det['feat']), 3)
        elif self.kf.initialised:
            self.kf.predict(ts)
        if self.kf.initialised:
            st['seen_age_s'] = round(ts - self.kf.last_meas_t, 2)
        if self.identity.target_id is None:
            st['state'] = 'searching' if self.identity.has_model else 'none'
        elif tgt is not None and tgt.matched_this_frame:
            st['state'] = 'tracking' if event == 'tracking' else 'occluded'
        else:
            st['state'] = 'coasting'
        return st

    # ------------------------------------------------------------------ output
    def publish(self, header, tracks, st, rgb, times):
        odom_header = header.__class__()
        odom_header.stamp = header.stamp
        odom_header.frame_id = self.odom_frame
        arr = Detection3DArray(header=odom_header)
        for tr in tracks:
            m = self.pos3d.get(tr.id)
            if m is None:
                continue
            d = Detection3D(header=odom_header)
            d.id = str(tr.id)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'target' if tr.id == self.identity.target_id else 'person'
            hyp.hypothesis.score = float(tr.score)
            hyp.pose.pose.position.x, hyp.pose.pose.position.y = float(m[1][0]), float(m[1][1])
            hyp.pose.pose.orientation.w = 1.0
            cov = np.zeros((6, 6))
            cov[:2, :2] = m[2]
            cov[2, 2] = cov[3, 3] = cov[4, 4] = cov[5, 5] = 1e3
            hyp.pose.covariance = cov.ravel().tolist()
            d.results.append(hyp)
            d.bbox.center.position = hyp.pose.pose.position
            d.bbox.size.x = d.bbox.size.y = 0.5
            d.bbox.size.z = 1.7
            arr.detections.append(d)
        self.pub_people.publish(arr)

        if self.kf.initialised and self.kf.last_meas_t is not None:
            o = Odometry(header=odom_header)
            o.child_frame_id = self.odom_frame
            x = self.kf.x
            o.pose.pose.position.x, o.pose.pose.position.y = float(x[0]), float(x[1])
            yaw = math.atan2(x[3], x[2]) if self.kf.speed > 0.15 else 0.0
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            pc = np.zeros((6, 6))
            pc[:2, :2] = self.kf.P[:2, :2]
            pc[2, 2] = pc[3, 3] = pc[4, 4] = 1e3
            pc[5, 5] = 1e3
            o.pose.covariance = pc.ravel().tolist()
            o.twist.twist.linear.x, o.twist.twist.linear.y = float(x[2]), float(x[3])
            tc = np.zeros((6, 6))
            tc[:2, :2] = self.kf.P[2:, 2:]
            tc[2, 2] = tc[3, 3] = tc[4, 4] = tc[5, 5] = 1e3
            o.twist.covariance = tc.ravel().tolist()
            self.pub_target.publish(o)
            st['x'], st['y'] = round(float(x[0]), 3), round(float(x[1]), 3)
            st['speed'] = round(self.kf.speed, 2)

        st['people'] = len(tracks)
        st['ms'] = round(times['total'], 1)
        self.pub_state.publish(String(data=json.dumps(st)))

        now = time.monotonic()
        if now - self.last_debug >= self.debug_period:
            if self.pub_debug.get_subscription_count() > 0:
                self.last_debug = now
                self.publish_debug(header, tracks, rgb, st)
            if self.pub_markers.get_subscription_count() > 0:
                self.last_debug = now
                self.publish_markers(odom_header, tracks)

    def publish_debug(self, header, tracks, rgb, st):
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for tr in tracks:
            b = tr.det['box'].astype(int) if tr.matched_this_frame else tr.box.astype(int)
            is_t = tr.id == self.identity.target_id
            col = (0, 0, 255) if is_t else (0, 200, 0)
            cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), col, 3 if is_t else 1)
            label = f'{"TARGET " if is_t else ""}{tr.id}'
            m = self.pos3d.get(tr.id)
            if m is not None:
                label += f' {m[3]:.1f}m'
            cv2.putText(img, label, (b[0] + 2, max(b[1] + 16, 16)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, col, 2)
            k = tr.det.get('kpts')
            if k is not None and tr.matched_this_frame:
                for x, y, c in k:
                    if c > 0.5:
                        cv2.circle(img, (int(x), int(y)), 2, (255, 255, 0), -1)
        cv2.putText(img, f'{st["state"]}  {st.get("ms", 0):.0f} ms', (6, 470),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        img = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            msg = CompressedImage(header=header, format='jpeg', data=jpg.tobytes())
            self.pub_debug.publish(msg)

    def publish_markers(self, header, tracks):
        ma = MarkerArray()
        clear = Marker(header=header, action=Marker.DELETEALL)
        ma.markers.append(clear)
        for tr in tracks:
            m = self.pos3d.get(tr.id)
            if m is None:
                continue
            is_t = tr.id == self.identity.target_id
            c = Marker(header=header, ns='people', id=tr.id, type=Marker.CYLINDER)
            c.pose.position.x, c.pose.position.y, c.pose.position.z = float(m[1][0]), float(m[1][1]), 0.85
            c.pose.orientation.w = 1.0
            c.scale.x = c.scale.y = 0.45
            c.scale.z = 1.7
            c.color.r, c.color.g, c.color.b, c.color.a = (1.0, 0.1, 0.1, 0.8) if is_t else (0.1, 0.8, 0.1, 0.5)
            ma.markers.append(c)
            txt = Marker(header=header, ns='ids', id=tr.id, type=Marker.TEXT_VIEW_FACING)
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = float(m[1][0]), float(m[1][1]), 1.9
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.25
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f'{"TARGET " if is_t else ""}{tr.id}'
            ma.markers.append(txt)
        self.pub_markers.publish(ma)

    def perf_log(self):
        now = time.monotonic()
        if now - self.last_perf < self.perf_log_s:
            return
        dt = now - self.last_perf
        self.last_perf = now
        r = self.stage.report()
        self.get_logger().info(
            f'{self.frames / dt:.1f} fps; ms detect {r.get("detect")} reid {r.get("reid")} '
            f'track {r.get("track")} total {r.get("total")}; image age {r.get("latency")} ms; '
            f'no-TF frames {self.dropped_tf}')
        self.frames = 0
        self.dropped_tf = 0


def main():
    rclpy.init()
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.detector.trt.close()
        node.reid.trt.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
