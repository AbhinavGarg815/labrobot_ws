#!/usr/bin/env python3
"""Run the person pipeline offline on a recorded bag (scripts/record_people.sh).

    python3 tools/eval_follow_bag.py ~/labrobot_bags/NAME [--lock-at 3.0] [--video]
"""

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from tf2_ros import Buffer, TransformException

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'labrobot_follow'))
from labrobot_follow.detector import PoseDetector, ReidEncoder  # noqa: E402
from labrobot_follow.pipeline import PersonPipeline  # noqa: E402
from labrobot_follow.tracker import Track  # noqa: E402

M = Path.home() / 'labrobot_models' / 'engines'
COLOUR = '/camera/camera/color/image_raw'
DEPTH = '/camera/camera/aligned_depth_to_color/image_raw'
INFO = '/camera/camera/color/camera_info'


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, t = reader.read_next()
        yield topic, deserialize_message(data, get_message(types[topic])), t


def _reid_cols(pipe, tr):
    f = tr.det.get('feat') if tr.matched_this_frame else None
    if f is None or not pipe.identity.has_model:
        return ''
    return f'{pipe.identity.score(f):.3f}'


def draw(rgb, tracks, pipe, st, t):
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for tr in tracks:
        b = (tr.det['box'] if tr.matched_this_frame else tr.box).astype(int)
        is_t = tr.id == pipe.identity.target_id
        col = (0, 0, 255) if is_t else (0, 200, 0)
        cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), col, 3 if is_t else 1)
        m = pipe.pos3d.get(tr.id)
        label = f'{"T " if is_t else ""}{tr.id}' + (f' {m[3]:.2f}m' if m else '') + \
            ('' if tr.det.get('clean') else ' occ')
        cv2.putText(img, label, (b[0] + 2, max(b[1] + 18, 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        if tr.matched_this_frame and tr.det.get('kpts') is not None:
            for x, y, c in tr.det['kpts']:
                if c > 0.5:
                    cv2.circle(img, (int(x), int(y)), 3, (255, 255, 0), -1)
    txt = f't={t:6.2f}  {st["state"]}  id={st.get("track_id")}  ' + \
        (f'reid={st["reid_score"]:.2f}' if 'reid_score' in st else '')
    cv2.putText(img, txt, (6, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--detector', default='yolo26s-pose')
    ap.add_argument('--reid', default='osnet_ain_x1_0_msmt17')
    ap.add_argument('--lock-at', type=float, default=None)
    ap.add_argument('--select-at', default=None, help='T:ID')
    ap.add_argument('--video', action='store_true')
    ap.add_argument('--reid-thresh', type=float, default=0.72)
    ap.add_argument('--reid-margin', type=float, default=0.05)
    a = ap.parse_args()

    bag = Path(a.bag).expanduser()
    out = bag / f'eval_{a.detector}_{a.reid}'
    out.mkdir(exist_ok=True)
    det = PoseDetector(str(M / f'{a.detector}_480x640_fp16.engine'), conf_thresh=0.15)
    reid = ReidEncoder(str(M / f'{a.reid}_fp16.engine'))
    pipe = PersonPipeline(det, reid, reid_thresh=a.reid_thresh, reid_margin=a.reid_margin)
    buf = Buffer(cache_time=rclpy.duration.Duration(seconds=3600))
    K = None
    pending = {}          # stamp ns -> {'c': msg, 'd': msg}
    t0 = None
    sel_t, sel_id = (None, None)
    if a.select_at:
        sel_t, sel_id = a.select_at.split(':')
        sel_t, sel_id = float(sel_t), int(sel_id)
    locked = False
    fcsv = open(out / 'frames.csv', 'w', newline='')
    tcsv = open(out / 'tracks.csv', 'w', newline='')
    fw = csv.writer(fcsv)
    tw = csv.writer(tcsv)
    fw.writerow(['t', 'state', 'event', 'track_id', 'range', 'bearing_deg', 'x', 'y', 'speed',
                 'reid_score', 'gallery', 'people', 'ms_detect', 'ms_reid', 'ms_track'])
    tw.writerow(['t', 'id', 'x0', 'y0', 'x1', 'y1', 'score', 'clean', 'range', 'bearing_deg', 'target',
                 'reid_vs_target'])
    events = open(out / 'events.txt', 'w')
    video = None
    n_frames = n_notf = 0
    proc_ms = []
    wall0 = time.monotonic()

    for topic, msg, _ in read_bag(bag):
        if topic in ('/tf', '/tf_static'):
            for tf in msg.transforms:
                if topic == '/tf_static':
                    buf.set_transform_static(tf, 'bag')
                else:
                    buf.set_transform(tf, 'bag')
            continue
        if topic == INFO and K is None:
            K = np.array(msg.k, float).reshape(3, 3)
            continue
        if topic not in (COLOUR, DEPTH):
            continue
        key = Time.from_msg(msg.header.stamp).nanoseconds
        slot = pending.setdefault(key, {})
        slot['c' if topic == COLOUR else 'd'] = msg
        if len(slot) < 2 or K is None:
            # drop pairs that will never complete
            if len(pending) > 30:
                for k in sorted(pending)[:-30]:
                    del pending[k]
            continue
        c, d = slot['c'], slot['d']
        del pending[key]
        ts = key * 1e-9
        if t0 is None:
            t0 = ts
        t = ts - t0
        try:
            tf = buf.lookup_transform('odom', c.header.frame_id, Time(nanoseconds=key))
        except TransformException:
            n_notf += 1
            continue
        R = quat_to_rot(tf.transform.rotation)
        tv = tf.transform.translation
        tcam = np.array([tv.x, tv.y, tv.z])
        rgb = np.frombuffer(c.data, np.uint8).reshape(c.height, c.step)[:, :c.width * 3].reshape(
            c.height, c.width, 3)
        depth = np.frombuffer(d.data, np.uint16).reshape(d.height, d.step // 2)[:, :d.width]
        if not locked and a.lock_at is not None and t >= a.lock_at:
            pipe.select_nearest()
            locked = True
        if not locked and sel_t is not None and t >= sel_t:
            pipe.select_id(sel_id)
            locked = True
        w0 = time.perf_counter()
        tracks, st, times = pipe.process(rgb, depth, K, R, tcam, ts)
        proc_ms.append((time.perf_counter() - w0) * 1000)
        n_frames += 1
        for e in pipe.events:
            events.write(f'{t:7.2f}  {e}\n')
            print(f'{t:7.2f}  {e}')
        pipe.events.clear()
        fw.writerow([f'{t:.3f}', st['state'], st['event'], st.get('track_id'), st.get('range'),
                     st.get('bearing_deg'), st.get('x'), st.get('y'), st.get('speed'),
                     st.get('reid_score'), st['gallery'], st['people'],
                     f'{times["detect"]:.1f}', f'{times["reid"]:.1f}', f'{times["track"]:.1f}'])
        for tr in tracks:
            m = pipe.pos3d.get(tr.id)
            b = tr.det['box'] if tr.matched_this_frame else tr.box
            tw.writerow([f'{t:.3f}', tr.id, *[f'{v:.0f}' for v in b], f'{tr.score:.2f}',
                         int(bool(tr.det.get('clean'))),
                         f'{m[3]:.3f}' if m else '', f'{math.degrees(m[4]):.1f}' if m else '',
                         int(tr.id == pipe.identity.target_id),
                         _reid_cols(pipe, tr)])
        if a.video:
            frame = draw(rgb, tracks, pipe, st, t)
            if video is None:
                video = cv2.VideoWriter(str(out / 'video.avi'), cv2.VideoWriter_fourcc(*'MJPG'),
                                        15, (frame.shape[1], frame.shape[0]))
            video.write(frame)

    fcsv.close()
    tcsv.close()
    events.close()
    if video is not None:
        video.release()
    wall = time.monotonic() - wall0
    pm = np.array(proc_ms) if proc_ms else np.zeros(1)
    summary = {'frames': n_frames, 'frames_without_tf': n_notf, 'bag_seconds': round(t if t0 else 0, 1),
               'pipeline_ms_mean': round(float(pm.mean()), 1),
               'pipeline_ms_p95': round(float(np.percentile(pm, 95)), 1),
               'wall_s': round(wall, 1), 'tracks_created': Track._next_id - 1}
    print(json.dumps(summary))
    (out / 'summary.json').write_text(json.dumps(summary, indent=1))
    print(f'outputs in {out}')
    det.trt.close()
    reid.trt.close()


if __name__ == '__main__':
    main()
