"""Multi-person tracking by detection, BoT-SORT style."""

import numpy as np
from scipy.optimize import linear_sum_assignment

TRACKED, LOST, REMOVED = 0, 1, 2


def iou_matrix(a, b):
    """IoU between boxes a (N,4) and b (M,4), xyxy."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    union = area_a[:, None] + area_b[None, :] - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)


def xyxy_to_cxcywh(b):
    b = np.asarray(b, float)
    return np.array([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2, b[2] - b[0], b[3] - b[1]])


def cxcywh_to_xyxy(m):
    return np.array([m[0] - m[2] / 2, m[1] - m[3] / 2, m[0] + m[2] / 2, m[1] + m[3] / 2])


class BoxKalman:
    """Constant-velocity Kalman filter on (cx, cy, w, h), BoT-SORT noise model."""

    STD_POS = 1.0 / 20
    STD_VEL = 1.0 / 160

    def __init__(self):
        self.F = np.eye(8)
        self.F[:4, 4:] = np.eye(4)
        self.Hm = np.eye(4, 8)

    def initiate(self, z):
        mean = np.r_[z, np.zeros(4)]
        w, h = z[2], z[3]
        p, v = self.STD_POS, self.STD_VEL
        std = [2 * p * w, 2 * p * h, 2 * p * w, 2 * p * h,
               10 * v * w, 10 * v * h, 10 * v * w, 10 * v * h]
        return mean, np.diag(np.square(std))

    def predict(self, mean, cov):
        w, h = mean[2], mean[3]
        p, v = self.STD_POS, self.STD_VEL
        q = np.diag(np.square([p * w, p * h, p * w, p * h, v * w, v * h, v * w, v * h]))
        mean = self.F @ mean
        cov = self.F @ cov @ self.F.T + q
        return mean, cov

    def update(self, mean, cov, z):
        w, h = mean[2], mean[3]
        p = self.STD_POS
        r = np.diag(np.square([p * w, p * h, p * w, p * h]))
        s = self.Hm @ cov @ self.Hm.T + r
        k = np.linalg.solve(s, (cov @ self.Hm.T).T).T
        mean = mean + k @ (z - self.Hm @ mean)
        cov = cov - k @ s @ k.T
        return mean, cov


def warp_state(mean, cov, H):
    """Apply an image homography (camera rotation) to a box track state."""
    cx, cy = mean[0], mean[1]
    p = H @ np.array([cx, cy, 1.0])
    nx, ny = p[0] / p[2], p[1] / p[2]
    # Jacobian of the projective map at (cx, cy).
    d = p[2]
    J = np.array([[H[0, 0] - nx * H[2, 0], H[0, 1] - nx * H[2, 1]],
                  [H[1, 0] - ny * H[2, 0], H[1, 1] - ny * H[2, 1]]]) / d
    sx, sy = abs(J[0, 0]), abs(J[1, 1])
    out = mean.copy()
    out[0], out[1] = nx, ny
    out[2] *= sx
    out[3] *= sy
    out[4:6] = J @ mean[4:6]
    out[6] *= sx
    out[7] *= sy
    A = np.eye(8)
    A[:2, :2] = J
    A[4:6, 4:6] = J
    return out, A @ cov @ A.T


class Track:
    _next_id = 1

    def __init__(self, det, kf, stamp, feat_alpha):
        self.kf = kf
        self.mean, self.cov = kf.initiate(xyxy_to_cxcywh(det['box']))
        self.id = Track._next_id
        Track._next_id += 1
        self.state = TRACKED
        self.confirmed = False
        self.hits = 1
        self.start_stamp = stamp
        self.last_stamp = stamp
        self.score = det['score']
        self.det = det
        self.feat = None
        self.feat_alpha = feat_alpha
        self.matched_this_frame = True
        self.xy = None            # last ground position (odom) and its time
        self.xy_t = None
        self._update_feat(det)
        self._update_xy(det, stamp)

    @property
    def box(self):
        return cxcywh_to_xyxy(self.mean[:4])

    def _update_xy(self, det, stamp):
        if det.get('xy') is not None:
            self.xy = np.asarray(det['xy'], float)
            self.xy_t = stamp
            self.xy_range = float(det.get('range', 0.0))

    def _update_feat(self, det):
        f = det.get('feat')
        if f is None or not det.get('clean', True):
            return
        if self.feat is None:
            self.feat = f.copy()
        else:
            self.feat = self.feat_alpha * self.feat + (1 - self.feat_alpha) * f
        n = np.linalg.norm(self.feat)
        if n > 0:
            self.feat /= n

    def predict(self):
        if self.state != TRACKED:
            # Lost tracks drift: stop their size changing (BoT-SORT idea).
            self.mean[6] = self.mean[7] = 0.0
        self.mean, self.cov = self.kf.predict(self.mean, self.cov)

    def update(self, det, stamp, min_hits):
        self.mean, self.cov = self.kf.update(self.mean, self.cov, xyxy_to_cxcywh(det['box']))
        self.state = TRACKED
        self.hits += 1
        if self.hits >= min_hits:
            self.confirmed = True
        self.last_stamp = stamp
        self.score = det['score']
        self.det = det
        self.matched_this_frame = True
        self._update_feat(det)
        self._update_xy(det, stamp)


class PersonTracker:
    def __init__(self, high_thresh=0.5, low_thresh=0.15, new_track_thresh=0.55,
                 match_thresh=0.8, second_match_thresh=0.5, unconfirmed_thresh=0.7,
                 proximity_thresh=0.5, appearance_thresh=0.25,
                 track_buffer_s=3.0, min_hits=3, feat_alpha=0.9,
                 gate_base_m=0.4, gate_speed_mps=1.5, gate_range_k=0.03):
        self.high_thresh = high_thresh
        self.low_thresh = low_thresh
        self.new_track_thresh = new_track_thresh
        self.match_thresh = match_thresh
        self.second_match_thresh = second_match_thresh
        self.unconfirmed_thresh = unconfirmed_thresh
        self.proximity_thresh = proximity_thresh
        self.appearance_thresh = appearance_thresh
        self.track_buffer_s = track_buffer_s
        self.min_hits = min_hits
        self.feat_alpha = feat_alpha
        self.gate_base = gate_base_m
        self.gate_speed = gate_speed_mps
        self.gate_range_k = gate_range_k
        self.kf = BoxKalman()
        self.tracks = []

    # cost
    def _depth_gate(self, cost, tracks, dets, stamp):
        """Forbid (cost 1e5) pairs whose ground positions are out of reach."""
        for i, t in enumerate(tracks):
            if t.xy is None:
                continue
            for j, d in enumerate(dets):
                if d.get('xy') is None:
                    continue
                r = max(float(d.get('range', 0.0)), t.xy_range)
                reach = (self.gate_base + self.gate_speed * max(stamp - t.xy_t, 0.0)
                         + self.gate_range_k * r * r)
                if np.linalg.norm(np.asarray(d['xy']) - t.xy) > reach:
                    cost[i, j] = 1e5
        return cost

    def _fused_cost(self, tracks, dets):
        iou_d = 1.0 - iou_matrix([t.box for t in tracks], [d['box'] for d in dets])
        # ByteTrack's score fusion: weak detections look a bit further away.
        scores = np.array([d['score'] for d in dets])
        iou_d = 1.0 - (1.0 - iou_d) * scores[None, :]
        cost = iou_d.copy()
        tf = [t.feat for t in tracks]
        df = [d.get('feat') for d in dets]
        if any(f is not None for f in tf) and any(f is not None for f in df):
            emb = np.ones_like(iou_d)
            for i, a in enumerate(tf):
                if a is None:
                    continue
                for j, b in enumerate(df):
                    if b is None:
                        continue
                    emb[i, j] = (1.0 - float(a @ b)) / 2.0
            emb[emb > self.appearance_thresh] = 1.0
            emb[iou_d > self.proximity_thresh] = 1.0
            cost = np.minimum(iou_d, emb)
        return cost

    @staticmethod
    def _match(cost, thresh):
        if cost.size == 0:
            return [], list(range(cost.shape[0])), list(range(cost.shape[1]))
        c = np.where(cost > thresh, 1e5, cost)
        rows, cols = linear_sum_assignment(c)
        matches = [(r, k) for r, k in zip(rows, cols) if cost[r, k] <= thresh]
        mr = {r for r, _ in matches}
        mc = {k for _, k in matches}
        return (matches, [r for r in range(cost.shape[0]) if r not in mr],
                [k for k in range(cost.shape[1]) if k not in mc])

    # main step
    def update(self, dets, stamp, H=None):
        for t in self.tracks:
            t.matched_this_frame = False
            t.predict()
            if H is not None:
                t.mean, t.cov = warp_state(t.mean, t.cov, H)

        high = [d for d in dets if d['score'] >= self.high_thresh]
        low = [d for d in dets if self.low_thresh <= d['score'] < self.high_thresh]

        confirmed = [t for t in self.tracks if t.confirmed]
        unconfirmed = [t for t in self.tracks if not t.confirmed]

        matches, um_t, um_d = self._match(
            self._depth_gate(self._fused_cost(confirmed, high), confirmed, high, stamp),
            self.match_thresh)
        for ti, di in matches:
            confirmed[ti].update(high[di], stamp, self.min_hits)
        rem_tracks = [confirmed[i] for i in um_t if confirmed[i].state == TRACKED]
        rem_high = [high[i] for i in um_d]

        if rem_tracks and low:
            iou_d = self._depth_gate(
                1.0 - iou_matrix([t.box for t in rem_tracks], [d['box'] for d in low]),
                rem_tracks, low, stamp)
            m2, um_t2, _ = self._match(iou_d, self.second_match_thresh)
            for ti, di in m2:
                rem_tracks[ti].update(low[di], stamp, self.min_hits)
            unmatched_tracked = [rem_tracks[i] for i in um_t2]
        else:
            unmatched_tracked = rem_tracks
        for t in unmatched_tracked:
            t.state = LOST

        m3, um_u, um_d3 = self._match(
            self._depth_gate(self._fused_cost(unconfirmed, rem_high), unconfirmed, rem_high, stamp),
            self.unconfirmed_thresh)
        for ti, di in m3:
            unconfirmed[ti].update(rem_high[di], stamp, self.min_hits)
        for i in um_u:
            unconfirmed[i].state = REMOVED
        rem_high = [rem_high[i] for i in um_d3]

        for d in rem_high:
            if d['score'] >= self.new_track_thresh:
                self.tracks.append(Track(d, self.kf, stamp, self.feat_alpha))

        for t in self.tracks:
            if t.state == LOST and stamp - t.last_stamp > self.track_buffer_s:
                t.state = REMOVED
            # A lost box that has wandered out of all sense is not worth keeping.
            if t.mean[2] <= 1 or t.mean[3] <= 1:
                t.state = REMOVED
        self.tracks = [t for t in self.tracks if t.state != REMOVED]
        return [t for t in self.tracks if t.state == TRACKED and t.confirmed]

    def get(self, track_id):
        for t in self.tracks:
            if t.id == track_id:
                return t
        return None


def rotation_homography(K, R_cur_prev):
    return K @ R_cur_prev @ np.linalg.inv(K)
