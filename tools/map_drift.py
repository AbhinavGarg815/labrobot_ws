#!/usr/bin/env python3
"""Did a session bend the saved map? Compare the old nodes of two RTAB-Map databases.

    python3 tools/map_drift.py BEFORE.db AFTER.db          # report, exit 0 = OK, 2 = DISTORTED
    python3 tools/map_drift.py BEFORE.db AFTER.db --quiet  # one line

Lifelong mapping adds nodes and loop closures, and RTAB-Map then re-optimises
the WHOLE graph - so a session can move the nodes that were already mapped.
Measured (Problem 49 / 50): normal edits move old nodes ~0.5 deg / 4 cm median;
two sessions re-oriented the map by -4.9 and +3.9 deg (median 34 and 16 cm,
max 72 cm), which broke global localisation and shifted every saved waypoint.

Compares the optimised poses (Admin.opt_ids/opt_poses, written on every clean
close) of the nodes present in BOTH files. Read-only on both.
"""
import argparse
import math
import sqlite3
import sys
import zlib

import numpy as np

MAX_ROTATION_DEG = 1.5      # rigid rotation of the old map as a whole
MAX_MEDIAN_SHIFT_M = 0.08   # typical displacement of an old node


def opt_poses(path):
    db = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
    row = db.execute('SELECT opt_ids, opt_poses FROM Admin').fetchone()
    if not row or row[0] is None:
        raise ValueError(f'{path}: no optimised poses (never closed cleanly?)')
    unz = lambda b: zlib.decompress(bytes(b)[:-12])     # zlib + int32 rows/cols/type trailer
    ids = np.frombuffer(unz(row[0]), np.int32)
    P = np.frombuffer(unz(row[1]), np.float32).reshape(-1, 12)
    return dict(zip(ids.tolist(), P))


def compare(before, after):
    A, B = opt_poses(before), opt_poses(after)
    common = sorted(set(A) & set(B))
    if len(common) < 10:
        return {'ok': True, 'why': f'only {len(common)} common nodes - nothing to compare', 'common': len(common)}
    PA = np.array([[A[i][3], A[i][7]] for i in common])
    PB = np.array([[B[i][3], B[i][7]] for i in common])
    shift = np.linalg.norm(PA - PB, axis=1)
    ma, mb = PA.mean(0), PB.mean(0)
    U, _, Vt = np.linalg.svd((PA - ma).T @ (PB - mb))
    R = (U @ Vt).T
    rot = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    med = float(np.median(shift))
    ok = abs(rot) <= MAX_ROTATION_DEG and med <= MAX_MEDIAN_SHIFT_M
    why = (f'{len(common)} old nodes: rotation {rot:+.2f} deg, shift median {med*100:.1f} cm, '
           f'max {shift.max()*100:.1f} cm; {len(set(B) - set(A))} new nodes')
    return {'ok': ok, 'why': why, 'common': len(common), 'rotation_deg': rot, 'median_shift_m': med}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('before'); ap.add_argument('after')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()
    try:
        r = compare(a.before, a.after)
    except Exception as e:  # noqa: BLE001
        print(f'  map drift check failed: {e}'); return 1
    verdict = 'OK' if r['ok'] else 'DISTORTED'
    print(f'  map drift {verdict}: {r["why"]}' + ('' if a.quiet or r['ok'] else
          f'  (limits: rotation {MAX_ROTATION_DEG} deg, median shift {MAX_MEDIAN_SHIFT_M*100:.0f} cm)'))
    return 0 if r['ok'] else 2


if __name__ == '__main__':
    sys.exit(main())
