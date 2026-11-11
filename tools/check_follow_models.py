#!/usr/bin/env python3
"""Parity + latency check of the person-following TensorRT engines.

    python3 tools/check_follow_models.py            # all engines found
    python3 tools/check_follow_models.py --runs 300

Parity: each engine's output on the reference image is compared with the
original PyTorch model's (tools/follow_model_reference.py). Pass criteria:
same number of people, every box matched at IoU >= 0.9, score within 0.05,
visible keypoints within 3 px (median); re-ID cosine to the PyTorch
embedding >= 0.995 for every crop.
Latency: wall time of the full Python call (copy in, infer, copy out,
decode), which is what the node will pay - not just GPU time.
Robot does not move. Safe with the nav stack running (that is the point:
the numbers are measured under the real load).
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'labrobot_follow'))
from labrobot_follow.detector import PoseDetector, ReidEncoder  # noqa: E402
from labrobot_follow.tracker import iou_matrix  # noqa: E402

M = Path.home() / 'labrobot_models'


def timeit(fn, runs):
    for _ in range(10):
        fn()
    t = []
    for _ in range(runs):
        t0 = time.perf_counter()
        fn()
        t.append((time.perf_counter() - t0) * 1000)
    t = np.array(t)
    return t.mean(), np.percentile(t, 95)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=200)
    a = ap.parse_args()
    ref = np.load(M / 'reference.npz')
    img = ref['image']
    ok_all = True
    print(f'{"engine":34s} {"parity":28s} {"mean ms":>8s} {"p95 ms":>8s}')
    for name in ('yolo26n-pose', 'yolo26s-pose', 'yolo11n-pose', 'yolo11s-pose'):
        path = M / 'engines' / f'{name}_480x640_fp16.engine'
        if not path.exists():
            print(f'{name:34s} (no engine)')
            continue
        det = PoseDetector(str(path), conf_thresh=0.25, iou_thresh=0.7)
        d = det(img)
        rb, rs, rk = ref[f'{name}_boxes'], ref[f'{name}_scores'], ref[f'{name}_kpts']
        if len(d) != len(rb):
            verdict, ok = f'FAIL {len(d)} vs {len(rb)} people', False
        else:
            iou = iou_matrix(np.array([x['box'] for x in d]), rb)
            j = iou.argmax(axis=1)
            ious = iou.max(axis=1)
            ds = max(abs(x['score'] - rs[k]) for x, k in zip(d, j))
            kerr = []
            for x, k in zip(d, j):
                vis = rk[k][:, 2] > 0.5
                kerr.extend(np.hypot(*(x['kpts'][vis, :2] - rk[k][vis, :2]).T))
            km = float(np.median(kerr)) if kerr else 0.0
            ok = ious.min() >= 0.9 and ds <= 0.05 and km <= 3.0
            verdict = f'{"OK" if ok else "FAIL"} iou>={ious.min():.3f} ds<={ds:.3f} kpt {km:.1f}px'
        ok_all &= ok
        mean, p95 = timeit(lambda: det(img), a.runs)
        print(f'{name:34s} {verdict:28s} {mean:8.2f} {p95:8.2f}')
        det.trt.close()

    boxes = ref['reid_boxes']
    for wname in ('osnet_x0_25_msmt17', 'osnet_ain_x1_0_msmt17'):
        path = M / 'engines' / f'{wname}_fp16.engine'
        if not path.exists():
            print(f'{wname:34s} (no engine)')
            continue
        enc = ReidEncoder(str(path))
        e = enc(img, boxes)
        cos = np.sum(e * ref[f'{wname}_emb'], axis=1)
        ok = cos.min() >= 0.995
        ok_all &= ok
        verdict = f'{"OK" if ok else "FAIL"} cos>={cos.min():.4f}'
        for n in (1, 4, 8):
            bb = np.resize(boxes, (n, 4))
            mean, p95 = timeit(lambda: enc(img, bb), a.runs)
            print(f'{wname + f" x{n}":34s} {verdict if n == 1 else "":28s} {mean:8.2f} {p95:8.2f}')
        enc.trt.close()
    load = os.getloadavg()
    print(f'load average {load[0]:.2f} {load[1]:.2f} {load[2]:.2f}')
    print('PARITY OK' if ok_all else 'PARITY FAILED')
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
