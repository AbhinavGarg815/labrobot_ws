"""From a 2D person detection + aligned depth to a 3D position with covariance."""

import numpy as np

# COCO keypoint indices
L_SHOULDER, R_SHOULDER, L_HIP, R_HIP, L_KNEE, R_KNEE = 5, 6, 11, 12, 13, 14
CORE_KPTS = (L_SHOULDER, R_SHOULDER, L_HIP, R_HIP, L_KNEE, R_KNEE)

DEPTH_MIN_M = 0.3
DEPTH_MAX_M = 8.0
# Depth hits the front of the body; the person's centre is behind it.
BODY_HALF_DEPTH_M = 0.02
# D435 RMS depth error grows with range squared: 7.4 mm at 1 m (measured, M2.T1).
DEPTH_RMS_AT_1M = 0.0074


def core_region(box, kpts, kpt_conf=0.5, img_w=640, img_h=480):
    x0, y0, x1, y1 = [float(v) for v in box]
    w, h = x1 - x0, y1 - y0
    pts = None
    if kpts is not None:
        k = np.asarray(kpts)
        sel = [i for i in CORE_KPTS if k[i, 2] >= kpt_conf]
        if len(sel) >= 2:
            pts = k[sel, :2]
    if pts is not None:
        rx0, ry0 = pts.min(axis=0)
        rx1, ry1 = pts.max(axis=0)
        cx = (rx0 + rx1) / 2
        half = max((rx1 - rx0) / 2, w / 6)
        rx0, rx1 = cx - half, cx + half
        if ry1 - ry0 < h / 6:
            cy = (ry0 + ry1) / 2
            ry0, ry1 = cy - h / 12, cy + h / 12
        u, v = cx, (ry0 + ry1) / 2
    else:
        # No joints: the central column of the box's upper-middle part.
        rx0, rx1 = x0 + 0.35 * w, x1 - 0.35 * w
        ry0, ry1 = y0 + 0.20 * h, y0 + 0.60 * h
        u, v = (rx0 + rx1) / 2, (ry0 + ry1) / 2
    rx0, rx1 = max(rx0, x0, 0), min(rx1, x1, img_w - 1)
    ry0, ry1 = max(ry0, y0, 0), min(ry1, y1, img_h - 1)
    return (int(rx0), int(ry0), int(np.ceil(rx1)), int(np.ceil(ry1))), (u, v)


def body_depth(depth, region, scale=1.0, bin_m=0.05, keep_m=0.15, min_pixels=30):
    """Dominant-mode depth (m) in a region of a depth image."""
    x0, y0, x1, y1 = region
    if x1 <= x0 or y1 <= y0:
        return None, 0
    patch = depth[y0:y1, x0:x1]
    # Subsample big regions: a 200x200 patch has more pixels than needed.
    step = max(1, int(np.sqrt(patch.size / 4000.0)))
    vals = patch[::step, ::step].ravel().astype(np.float32) * scale
    vals = vals[(vals > DEPTH_MIN_M) & (vals < DEPTH_MAX_M)]
    if vals.size < min_pixels:
        return None, int(vals.size)
    nb = int(np.ceil((DEPTH_MAX_M - DEPTH_MIN_M) / bin_m))
    hist, edges = np.histogram(vals, bins=nb, range=(DEPTH_MIN_M, DEPTH_MAX_M))
    smooth = np.convolve(hist, np.ones(3), mode='same')
    mode = (edges[np.argmax(smooth)] + edges[np.argmax(smooth) + 1]) / 2
    near = vals[np.abs(vals - mode) <= keep_m]
    if near.size < min_pixels:
        return None, int(near.size)
    return float(np.median(near)), int(near.size)


def backproject(u, v, z, K):
    """Pixel + depth -> point in the camera optical frame (x right, y down, z out)."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return np.array([(u - cx) / fx * z, (v - cy) / fy * z, z])


def measurement_cov_cam(p_cam, fx, pixel_sigma=6.0, extra_m=0.05):
    z = p_cam[2]
    s_along = np.hypot(DEPTH_RMS_AT_1M * z * z * 3.0, extra_m)
    s_across = np.hypot(z * pixel_sigma / fx, extra_m)
    return s_along, s_across


def ground_cov(bearing_xy, s_along, s_across):
    c, s = bearing_xy
    R = np.array([[c, -s], [s, c]])
    return R @ np.diag([s_along ** 2, s_across ** 2]) @ R.T


class TargetFilter:
    """2D constant-velocity Kalman filter for the target in `odom`."""

    def __init__(self, sigma_a=1.0, max_speed=2.0):
        self.sigma_a = sigma_a
        self.max_speed = max_speed
        self.x = None
        self.P = None
        self.t = None
        self.last_meas_t = None

    @property
    def initialised(self):
        return self.x is not None

    def reset(self):
        self.x = self.P = self.t = self.last_meas_t = None

    def init(self, z, R, t):
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.zeros((4, 4))
        self.P[:2, :2] = R
        self.P[2:, 2:] = np.eye(2) * 1.0 ** 2
        self.t = t
        self.last_meas_t = t

    def predict(self, t):
        if self.x is None or t <= self.t:
            return
        dt = t - self.t
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.sigma_a ** 2
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ G.T * q
        if self.last_meas_t is not None and t - self.last_meas_t > 1.0:
            # Coasting: decay velocity (time constant 1 s).
            self.x[2:] *= np.exp(-dt)
        self.t = t

    def mahalanobis2(self, z, R):
        S = self.P[:2, :2] + R
        d = np.asarray(z) - self.x[:2]
        return float(d @ np.linalg.solve(S, d))

    def update(self, z, R, t):
        if self.x is None:
            self.init(z, R, t)
            return
        self.predict(t)
        H = np.eye(2, 4)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(z) - H @ self.x)
        self.P = (np.eye(4) - K @ H) @ self.P
        sp = np.hypot(self.x[2], self.x[3])
        if sp > self.max_speed:
            self.x[2:] *= self.max_speed / sp
        self.last_meas_t = t

    def position_at(self, t, horizon_cap=2.0):
        if self.x is None:
            return None
        dt = min(t, (self.last_meas_t or self.t) + horizon_cap) - self.t
        return self.x[:2] + self.x[2:] * max(dt, 0.0)

    @property
    def speed(self):
        return 0.0 if self.x is None else float(np.hypot(self.x[2], self.x[3]))
