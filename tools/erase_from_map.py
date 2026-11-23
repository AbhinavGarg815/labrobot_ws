#!/usr/bin/env python3
"""Erase a transient obstacle (a person, a bag left on the floor) from an RTAB-Map map.

    python3 tools/erase_from_map.py --db ~/.ros/rtabmap.db --box X0 X1 Y0 Y1 --dry-run
    python3 tools/erase_from_map.py --db ~/.ros/rtabmap.db --box X0 X1 Y0 Y1
"""
import argparse
import os
import shutil
import sqlite3
import struct
import sys
import time
import zlib

import numpy as np

CV_32FC2 = 13


def unz(blob):
    """RTAB-Map compressData2: zlib payload + int32 rows, cols, cv type."""
    b = bytes(blob)
    rows, cols, typ = struct.unpack('<iii', b[-12:])
    return rows, cols, typ, zlib.decompress(b[:-12])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--db', default=os.path.expanduser('~/.ros/rtabmap.db'))
    ap.add_argument('--box', nargs=4, type=float, required=True, metavar=('X0', 'X1', 'Y0', 'Y1'),
                    help='map-frame box to clear, metres')
    ap.add_argument('--margin', type=float, default=0.10, help='added on every side (m)')
    ap.add_argument('--dry-run', action='store_true', help='report only, change nothing')
    a = ap.parse_args()

    x0, x1, y0, y1 = a.box
    x0, x1, y0, y1 = min(x0, x1) - a.margin, max(x0, x1) + a.margin, min(y0, y1) - a.margin, max(y0, y1) + a.margin
    if not os.path.exists(a.db):
        print(f'  no database at {a.db}'); return 1
    if os.system('pgrep -f "lib/rtabmap_slam/rtabma[p]" > /dev/null') == 0:
        print('  RTAB-Map is running - stop the stack first (scripts/stop_robot.sh)'); return 1

    if not a.dry_run:
        backup = f'{a.db}.before_erase_{time.strftime("%Y%m%d_%H%M%S")}'
        shutil.copy2(a.db, backup)
        print(f'  backup: {backup}')
    db = sqlite3.connect(a.db if not a.dry_run else f'file:{a.db}?mode=ro', uri=a.dry_run)
    row = db.execute('SELECT opt_ids, opt_poses FROM Admin').fetchone()
    if not row or row[0] is None:
        print('  database has no optimised poses (open and close it once in RTAB-Map first)'); return 1
    ids = np.frombuffer(unz(row[0])[3], np.int32)
    poses = np.frombuffer(unz(row[1])[3], np.float32).reshape(-1, 12)
    pose = dict(zip(ids.tolist(), poses))

    removed = touched = 0
    for nid, ob in db.execute('SELECT id, obstacle_cells FROM Data WHERE obstacle_cells IS NOT NULL').fetchall():
        if nid not in pose:
            continue                      # not in the optimised graph: never drawn
        _, _, typ, raw = unz(ob)
        if typ != CV_32FC2:
            print(f'  node {nid}: unexpected cell type {typ} (2D grids expected) - stopping'); return 1
        pts = np.frombuffer(raw, np.float32).reshape(-1, 2)
        m = pose[nid]
        wx = m[0]*pts[:, 0] + m[1]*pts[:, 1] + m[3]
        wy = m[4]*pts[:, 0] + m[5]*pts[:, 1] + m[7]
        inside = (wx >= x0) & (wx <= x1) & (wy >= y0) & (wy <= y1)
        if not inside.any():
            continue
        removed += int(inside.sum()); touched += 1
        if a.dry_run:
            continue
        keep = np.ascontiguousarray(pts[~inside], np.float32)
        blob = zlib.compress(keep.tobytes()) + struct.pack('<iii', 1, len(keep), CV_32FC2)
        db.execute('UPDATE Data SET obstacle_cells=? WHERE id=?', (sqlite3.Binary(blob), nid))

    verb = 'would remove' if a.dry_run else 'removed'
    print(f'  box x[{x0:+.2f},{x1:+.2f}] y[{y0:+.2f},{y1:+.2f}]: {verb} {removed} obstacle points '
          f'from {touched} nodes')
    if not a.dry_run:
        db.execute('UPDATE Admin SET opt_map=NULL')     # rebuild from the edited nodes on next load
        db.commit()
        print(f'  integrity: {db.execute("PRAGMA integrity_check").fetchone()[0]}; stored grid cleared - '
              'RTAB-Map rebuilds it on next start')
    return 0


if __name__ == '__main__':
    sys.exit(main())
