"""Offline tests for tracker, geometry and identity (no ROS, no GPU)."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from labrobot_follow import geometry as geo  # noqa: E402
from labrobot_follow.identity import TargetIdentity  # noqa: E402
from labrobot_follow.tracker import (PersonTracker, Track, iou_matrix,  # noqa: E402
                                     rotation_homography)

K = np.array([[604.2, 0, 318.4], [0, 604.3, 251.8], [0, 0, 1]])
RNG = np.random.default_rng(0)


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def appearance(seed, dim=512):
    return unit(np.random.default_rng(seed).normal(size=dim))


def noisy(f, s=0.25):
    return unit(f + RNG.normal(size=f.size) * s / math.sqrt(f.size))


def det(cx, w=80, h=300, top=60, score=0.9, feat=None, clean=True):
    return {'box': np.array([cx - w / 2, top, cx + w / 2, top + h]), 'score': score,
            'feat': feat, 'clean': clean}


@pytest.fixture(autouse=True)
def reset_ids():
    Track._next_id = 1


def test_iou():
    a = np.array([[0, 0, 10, 10]])
    b = np.array([[5, 0, 15, 10], [20, 20, 30, 30]])
    np.testing.assert_allclose(iou_matrix(a, b), [[50 / 150, 0.0]])


def test_single_person_keeps_id_through_occlusion():
    tr = PersonTracker(track_buffer_s=3.0)
    f = appearance(1)
    ids = set()
    t = 0.0
    for i in range(30):
        t = i / 15
        out = tr.update([det(200 + 2 * i, feat=noisy(f))], t)
        ids |= {o.id for o in out}
    for i in range(30, 45):            # 1 s fully occluded
        t = i / 15
        assert tr.update([], t) == []
    for i in range(45, 60):
        t = i / 15
        out = tr.update([det(200 + 2 * i, feat=noisy(f))], t)
        ids |= {o.id for o in out}
    assert ids == {1}


def _meet_and_turn_back(use_feat):
    tr = PersonTracker()
    fa, fb = appearance(1), appearance(2)
    seen = {}
    for i in range(70):
        k = i if i < 30 else 60 - i
        xa, xb = 200 + 5 * k, 470 - 5 * k
        overlap = abs(xa - xb) < 80
        da = det(xa, score=0.35 if overlap else 0.9,
                 feat=noisy(fa) if use_feat else None, clean=not overlap)
        db = det(xb, feat=noisy(fb) if use_feat else None, clean=not overlap)
        da['who'], db['who'] = 'A', 'B'
        for o in tr.update([da, db], i / 15):
            if o.matched_this_frame:
                seen.setdefault(o.id, set()).add(o.det['who'])
    return seen


def test_meet_and_turn_back_needs_appearance():
    without = _meet_and_turn_back(False)
    assert any(len(v) > 1 for v in without.values()), \
        f'negative control: motion alone should swap ids here, got {without}'
    Track._next_id = 1
    withf = _meet_and_turn_back(True)
    assert all(len(v) == 1 for v in withf.values()), withf
    assert len(withf) == 2


def _rotating_camera_ids(use_h, rate=0.6):
    """Static person, camera turning at `rate` rad/s, 15 fps, narrow box."""
    dyaw = rate / 15
    # optical frame: y down; turning left (about base +z) = about camera -y.
    c, s = math.cos(dyaw), math.sin(dyaw)
    H = rotation_homography(K, np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]]).T)
    tr = PersonTracker()
    ids = set()
    for i in range(25):
        cx = 60 + 604.2 * math.tan(dyaw * i)
        out = tr.update([det(cx, w=40)], i / 15, H=H if (use_h and i) else None)
        ids |= {o.id for o in out}
    return ids, H


def test_camera_rotation_compensation():
    ids, H = _rotating_camera_ids(True)
    # the homography moves the principal point by fx * tan(dyaw)
    p = H @ np.array([318.4, 251.8, 1.0])
    assert p[0] / p[2] - 318.4 == pytest.approx(604.2 * math.tan(0.6 / 15), rel=1e-3)
    assert ids == {1}
    Track._next_id = 1
    ids_without, _ = _rotating_camera_ids(False)
    assert ids_without != {1}, 'negative control: without compensation the track should break'


def test_core_region_uses_hips_and_knees():
    k = np.zeros((17, 3))
    for i, (x, y) in {geo.L_HIP: (300, 200), geo.R_HIP: (340, 200),
                      geo.L_KNEE: (305, 320), geo.R_KNEE: (335, 320)}.items():
        k[i] = (x, y, 0.9)
    region, (u, v) = geo.core_region([260, 0, 380, 470], k)
    assert region[0] <= 300 and region[2] >= 340
    assert region[1] == 200 and region[3] == 320
    assert u == pytest.approx(320) and v == pytest.approx(260)


def test_body_depth_rejects_background_and_occluder():
    d = np.full((480, 640), 0.0, np.float32)
    region = (300, 200, 340, 320)
    patch = np.full((120, 40), 2.0, np.float32) + RNG.normal(0, 0.01, (120, 40)).astype(np.float32)
    patch[:, 15:25] = 4.5                 # background between the legs
    patch[:20, :] = 1.2                   # an arm/occluder in front, smaller area
    d[200:320, 300:340] = patch
    z, n = geo.body_depth(d, region)
    assert z == pytest.approx(2.0, abs=0.03)
    assert n > 100


def test_body_depth_none_when_no_data():
    d = np.zeros((480, 640), np.float32)
    assert geo.body_depth(d, (300, 200, 340, 320))[0] is None


def test_backproject_centre():
    p = geo.backproject(318.4, 251.8, 2.0, K)
    np.testing.assert_allclose(p, [0, 0, 2.0], atol=1e-9)


def test_target_filter_constant_velocity():
    kf = geo.TargetFilter(sigma_a=0.5)
    R = np.eye(2) * 0.05 ** 2
    for i in range(50):
        t = i * 0.1
        kf.update(np.array([1.0 + 0.5 * t, 2.0]) + RNG.normal(0, 0.03, 2), R, t)
    assert kf.x[2] == pytest.approx(0.5, abs=0.1)
    assert abs(kf.x[3]) < 0.1
    # prediction is capped 2 s past the last measurement
    p_far = kf.position_at(4.9 + 10.0)
    p_cap = kf.position_at(4.9 + 2.0)
    np.testing.assert_allclose(p_far, p_cap)


def test_target_filter_gate():
    kf = geo.TargetFilter()
    R = np.eye(2) * 0.05 ** 2
    kf.update(np.array([1.0, 0.0]), R, 0.0)
    kf.predict(0.1)
    assert kf.mahalanobis2([1.02, 0.0], R) < 9.21
    assert kf.mahalanobis2([3.0, 0.0], R) > 9.21


class FakeTrack:
    def __init__(self, tid, feat, clean=True, confirmed=True):
        self.id = tid
        self.det = {'feat': feat, 'clean': clean}
        self.confirmed = confirmed


def test_identity_reacquires_only_the_target():
    fa, fb = appearance(10), appearance(11)
    idn = TargetIdentity(reid_votes=3)
    idn.lock(1)
    for i in range(30):                   # learn A, see B as a distractor
        ev, _ = idn.step([FakeTrack(1, noisy(fa)), FakeTrack(2, noisy(fb))], i * 0.1,
                         alive_ids={1, 2})
        assert ev == 'tracking'
    # A's track disappears; tracker drops it
    ev, _ = idn.step([FakeTrack(2, noisy(fb))], 3.1, alive_ids={2})
    assert ev == 'searching' and idn.target_id is None
    # B alone never re-acquired
    for i in range(20):
        ev, _ = idn.step([FakeTrack(2, noisy(fb))], 3.2 + i * 0.1, alive_ids={2})
        assert ev == 'searching'
    # A returns as a new track 7: needs 3 votes
    evs = [idn.step([FakeTrack(2, noisy(fb)), FakeTrack(7, noisy(fa))], 6 + i * 0.1,
                    alive_ids={2, 7})[0] for i in range(3)]
    assert evs == ['searching', 'searching', 'reacquired']
    assert idn.target_id == 7


def test_identity_plausibility_gate_blocks():
    fa = appearance(12)
    idn = TargetIdentity(reid_votes=1)
    idn.lock(1)
    for i in range(10):
        idn.step([FakeTrack(1, noisy(fa))], i * 0.1, alive_ids={1})
    idn.step([], 1.1, alive_ids=set())
    ev, _ = idn.step([FakeTrack(5, noisy(fa))], 1.2, plausible=lambda tr: False,
                     alive_ids={5})
    assert ev == 'searching'


def test_identity_detects_switch():
    fa, fb = appearance(20), appearance(21)
    idn = TargetIdentity(switch_frames=4)
    idn.lock(1)
    for i in range(20):
        idn.step([FakeTrack(1, noisy(fa)), FakeTrack(2, noisy(fb))], i * 0.1,
                 alive_ids={1, 2})
    # the tracker swaps: track 1 now carries B's appearance, track 2 A's
    evs = [idn.step([FakeTrack(1, noisy(fb)), FakeTrack(2, noisy(fa))], 2 + i * 0.1,
                    alive_ids={1, 2})[0] for i in range(4)]
    assert evs[-1] == 'switch'
    assert idn.target_id is None
    # and the gallery was not polluted by B
    assert idn.score(fb) < 0.3


def test_identity_unclean_views_not_learned():
    fa, fb = appearance(30), appearance(31)
    idn = TargetIdentity()
    idn.lock(1)
    idn.step([FakeTrack(1, noisy(fa))], 0.0, alive_ids={1})
    n0 = len(idn.short)
    for i in range(10):
        ev, _ = idn.step([FakeTrack(1, noisy(fb), clean=False)], 0.5 + i * 0.4,
                         alive_ids={1})
        assert ev == 'occluded'
    assert len(idn.short) == n0


def _pass_in_front(use_gate):
    """people1 (97 s): seated person B at 2.42 m, box cut by the image edge (never clean -> no appearance)."""
    tr = PersonTracker() if use_gate else PersonTracker(gate_base_m=1e9)
    seen = {}
    for i in range(60):
        t = i / 15
        db = {'box': np.array([485, 38, 640, 324.]), 'score': 0.88, 'feat': None,
              'clean': False, 'xy': np.array([2.42, -1.0]), 'range': 2.42, 'who': 'B'}
        dets = []
        if not (30 <= i < 36):                 # B occluded by A for 6 frames
            dets.append(db)
        if i >= 28:
            cx = 600 - 8 * (i - 28)
            dets.append({'box': np.array([cx - 75, 0, cx + 75, 440.]), 'score': 0.85,
                         'feat': None, 'clean': False, 'xy': np.array([1.57, -0.5]),
                         'range': 1.6, 'who': 'A'})
        for o in tr.update(dets, t):
            if o.matched_this_frame:
                seen.setdefault(o.id, set()).add(o.det['who'])
    return seen


def test_depth_gate_stops_track_takeover():
    without = _pass_in_front(False)
    assert any(len(v) > 1 for v in without.values()), \
        f'negative control: without the gate a track should jump, got {without}'
    Track._next_id = 1
    withg = _pass_in_front(True)
    assert all(len(v) == 1 for v in withg.values()), withg


def test_identity_low_score_alone_is_not_a_switch():
    fa, fb = appearance(40), appearance(41)
    idn = TargetIdentity(switch_frames=4)
    idn.lock(1)
    for i in range(20):
        idn.step([FakeTrack(1, noisy(fa))], i * 0.1, alive_ids={1})
    # the same track now looks unlike the memory (e.g. seen close, from the front)
    evs = [idn.step([FakeTrack(1, noisy(fb))], 2 + i * 0.1, alive_ids={1})[0] for i in range(10)]
    assert 'switch' not in evs and idn.target_id == 1
    assert idn.score(fb) < 0.3          # and it was not learned
