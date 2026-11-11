"""The per-frame person pipeline, independent of ROS.

Used by the live node (scripts/person_tracker.py) and by the offline bag
evaluator (tools/eval_follow_bag.py), so both run exactly the same code:

    detect (pose) -> re-ID -> track (with camera-rotation compensation)
    -> 3D per track in odom -> target identity -> target Kalman filter

Inputs per frame: RGB uint8, raw depth uint16 (mm, aligned to colour),
camera intrinsics K, the camera pose in odom (R, t) at the image time, and
the image time in seconds.
"""

import math
import time

import numpy as np

from . import geometry as geo
from .detector import mark_clean
from .identity import TargetIdentity
from .tracker import PersonTracker, rotation_homography

CHI2_2D_99 = 9.21


class PersonPipeline:
    def __init__(self, detector, reid, min_box_h=40, select_max_bearing_deg=20.0,
                 select_max_range_m=3.5, click_radius_m=0.8, reid_gate_base_m=1.0,
                 reid_gate_speed_mps=1.5, reid_thresh=0.60, reid_margin=0.05,
                 depth_scale=0.001):
        self.detector = detector
        self.reid = reid
        self.min_box_h = min_box_h
        self.sel_bearing = math.radians(select_max_bearing_deg)
        self.sel_range = select_max_range_m
        self.click_radius = click_radius_m
        self.gate_base = reid_gate_base_m
        self.gate_speed = reid_gate_speed_mps
        self.depth_scale = depth_scale
        self.tracker = PersonTracker()
        self.identity = TargetIdentity(reid_thresh=reid_thresh, reid_margin=reid_margin)
        self.kf = geo.TargetFilter()
        self.prev_R = None
        self.pos3d = {}           # id -> (t, xy, cov2, range, bearing)
        self.pending_select = None
        self.events = []          # human-readable target events since the last frame

    # ------------------------------------------------------------ commands
    def select_nearest(self):
        self.pending_select = ('nearest',)

    def select_id(self, track_id):
        self.pending_select = ('id', int(track_id))

    def select_point(self, xy):
        self.pending_select = ('point', np.asarray(xy, float))

    def release(self):
        self.identity.reset()
        self.kf.reset()
        self.events.append('released by operator')

    # ------------------------------------------------------------ frame
    def process(self, rgb, depth, K, R, tcam, ts):
        """Returns (tracks, state dict, stage times ms)."""
        times = {}
        h, w = rgb.shape[:2]
        t1 = time.perf_counter()
        dets = [d for d in self.detector(rgb) if d['box'][3] - d['box'][1] >= self.min_box_h]
        times['detect'] = (time.perf_counter() - t1) * 1000
        mark_clean(dets, w)
        t1 = time.perf_counter()
        if dets:
            for d, f in zip(dets, self.reid(rgb, [d['box'] for d in dets])):
                d['feat'] = f
        times['reid'] = (time.perf_counter() - t1) * 1000

        t1 = time.perf_counter()
        H = rotation_homography(K, R.T @ self.prev_R) if self.prev_R is not None else None
        self.prev_R = R
        tracks = self.tracker.update(dets, ts, H)
        self.pos3d = {k: v for k, v in self.pos3d.items() if ts - v[0] < 3.0}
        for tr in tracks:
            if tr.matched_this_frame:
                m = self.measure(tr.det, depth, K, R, tcam, w, h)
                if m is not None:
                    self.pos3d[tr.id] = (ts,) + m
        times['track'] = (time.perf_counter() - t1) * 1000

        self._apply_selection(tracks, ts)
        alive = {t.id for t in self.tracker.tracks}
        event, tid = self.identity.step(tracks, ts, plausible=lambda tr: self._plausible(tr, ts),
                                        alive_ids=alive)
        state = self._update_target(event, tid, tracks, ts)
        return tracks, state, times

    def measure(self, det, depth, K, R, tcam, w, h):
        region, (u, v) = geo.core_region(det['box'], det.get('kpts'), img_w=w, img_h=h)
        z, _ = geo.body_depth(depth, region, scale=self.depth_scale)
        if z is None:
            return None
        z += geo.BODY_HALF_DEPTH_M
        p_cam = geo.backproject(u, v, z, K)
        p_odom = R @ p_cam + tcam
        ray = p_odom[:2] - tcam[:2]
        rng = float(np.linalg.norm(ray))
        if rng < 1e-3:
            return None
        s_along, s_across = geo.measurement_cov_cam(p_cam, K[0, 0])
        cov = geo.ground_cov(ray / rng, s_along, s_across)
        bearing = math.atan2(-p_cam[0], p_cam[2])     # + = to the camera's left
        return p_odom[:2], cov, float(math.hypot(p_cam[0], p_cam[2])), bearing

    def _plausible(self, tr, ts):
        """Re-acquisition gate: could the target have walked to this track?"""
        if not self.kf.initialised:
            return True
        m = self.pos3d.get(tr.id)
        if m is None or ts - m[0] > 0.5:
            return False
        dt = max(ts - self.kf.last_meas_t, 0.0)
        return float(np.linalg.norm(m[1] - self.kf.x[:2])) <= self.gate_base + self.gate_speed * dt

    def _apply_selection(self, tracks, ts):
        sel, self.pending_select = self.pending_select, None
        if sel is None:
            return
        cand = [(tr, self.pos3d[tr.id]) for tr in tracks
                if tr.id in self.pos3d and ts - self.pos3d[tr.id][0] < 0.5]
        chosen = None
        if sel[0] == 'id':
            chosen = next((tr for tr in tracks if tr.id == sel[1]), None)
            if chosen is None:
                self.events.append(f'select {sel[1]}: no such person in view')
                return
        elif sel[0] == 'nearest':
            front = [(m[3], tr) for tr, m in cand
                     if abs(m[4]) <= self.sel_bearing and m[3] <= self.sel_range]
            if not front:
                self.events.append(f'start: nobody within {self.sel_range:.1f} m and '
                                   f'+/-{math.degrees(self.sel_bearing):.0f} deg ahead')
                return
            chosen = min(front, key=lambda x: x[0])[1]
        elif sel[0] == 'point':
            near = [(float(np.linalg.norm(m[1] - sel[1])), tr) for tr, m in cand]
            near = [x for x in near if x[0] <= self.click_radius]
            if not near:
                self.events.append(f'click: nobody within {self.click_radius:.1f} m of it')
                return
            chosen = min(near, key=lambda x: x[0])[1]
        self.identity.lock(chosen.id)
        self.kf.reset()
        self.events.append(f'locked on person {chosen.id}')

    def _update_target(self, event, tid, tracks, ts):
        if event == 'switch':
            self.events.append('identity switch suspected - target dropped, searching by appearance')
        elif event == 'reacquired':
            self.kf.reset()
            self.events.append(f're-identified as person {tid}')
        st = {'event': event, 'track_id': self.identity.target_id,
              'gallery': len(self.identity.short) + len(self.identity.long)}
        tgt = None
        if self.identity.target_id is not None:
            tgt = next((t for t in tracks if t.id == self.identity.target_id), None)
        m = self.pos3d.get(tgt.id) if tgt is not None else None
        if tgt is not None and tgt.matched_this_frame and m is not None and m[0] == ts:
            _, xy, cov, rng, bearing = m
            if not self.kf.initialised:
                self.kf.init(xy, cov, ts)
            else:
                self.kf.predict(ts)
                # a measurement far outside the filter's belief is a depth
                # outlier (hit something else), not the person teleporting
                if self.kf.mahalanobis2(xy, cov) <= CHI2_2D_99 * 4:
                    self.kf.update(xy, cov, ts)
                else:
                    st['rejected_measurement'] = True
            st['range'] = round(rng, 3)
            st['bearing_deg'] = round(math.degrees(bearing), 1)
            if tgt.det.get('feat') is not None and self.identity.has_model:
                st['reid_score'] = round(self.identity.score(tgt.det['feat']), 3)
        elif self.kf.initialised:
            self.kf.predict(ts)
        if self.kf.initialised:
            st['seen_age_s'] = round(ts - self.kf.last_meas_t, 2)
            x = self.kf.x
            st['x'], st['y'] = round(float(x[0]), 3), round(float(x[1]), 3)
            st['speed'] = round(self.kf.speed, 2)
        if self.identity.target_id is None:
            st['state'] = 'searching' if self.identity.has_model else 'none'
        elif tgt is not None and tgt.matched_this_frame:
            st['state'] = 'tracking' if event == 'tracking' else 'occluded'
        else:
            st['state'] = 'coasting'
        st['people'] = len(tracks)
        return st
