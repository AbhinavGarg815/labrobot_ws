#!/usr/bin/env python3
"""Score a narrow-gap run: was the ROUTE centred, and did the ROBOT follow it?

    python3 tools/gap_edges.py --save /tmp/edges.json      # before the run
    python3 tools/record_path.py /tmp/run.json 120 &       # during the run
    python3 tools/gap_run_report.py /tmp/edges.json /tmp/run.json
"""
import json, math, sys

HL, HW = 0.21, 0.203

def main():
    E = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2]))
    ox, oy, oth = E['robot_odom']; c, s = math.cos(oth), math.sin(oth)
    def loc(x, y): return (c*(x-ox) + s*(y-oy), -s*(x-ox) + c*(y-oy))
    track = [(*loc(p[0], p[1]), math.atan2(math.sin(p[2]-oth), math.cos(p[2]-oth)), p[3] if len(p) > 3 else None)
             for p in R['odom']]
    plans = [(pl['t'], [loc(*q) for q in pl['odom_pts']]) for pl in R.get('plans', []) if 'odom_pts' in pl]
    def plan_lat(path, x):
        best = min(path, key=lambda q: abs(q[0]-x)) if path else None
        return best[1] if best and abs(best[0]-x) < 0.06 else None
    print('  slice   gap   route off-centre   robot off-route   left cl.  right cl.')
    worst = (9, None); offs = []; trk = []
    for sl in E['slices']:
        x = sl['x']; L = loc(*sl['left']); Rt = loc(*sl['right'])
        le, re = L[1], Rt[1]; ctr = (le + re) / 2
        lc = rc = 9; lats = []; times = []
        for f, l, h, t in track:
            pts = []
            for k in range(21):
                for dx, dy in ((-HL + 2*HL*k/20, -HW), (-HL + 2*HL*k/20, HW), (-HL, -HW + 2*HW*k/20), (HL, -HW + 2*HW*k/20)):
                    cx = f + dx*math.cos(h) - dy*math.sin(h); cy = l + dx*math.sin(h) + dy*math.cos(h)
                    if x - 0.05 <= cx < x + 0.05: pts.append(cy)
            if pts:
                lc = min(lc, le - max(pts)); rc = min(rc, min(pts) - re)
            if abs(f - x) < 0.03: lats.append(l); times.append(t)
        if lc == 9: continue
        worst = min(worst, (min(lc, rc), x))
        rob = sum(lats)/len(lats) if lats else None
        pl = None
        if plans and times and times[0] is not None:
            cand = [p for p in plans if p[0] <= times[0]] or plans[:1]
            pl = plan_lat(cand[-1][1], x)
        ro = f'{(pl-ctr)*100:+6.1f} cm' if pl is not None else '     -   '
        tr = f'{(rob-pl)*100:+6.1f} cm' if (pl is not None and rob is not None) else '     -   '
        if pl is not None: offs.append(pl - ctr)
        if pl is not None and rob is not None: trk.append(rob - pl)
        print(f'  {x:5.2f}  {sl["width"]*100:4.1f}     {ro}          {tr}        {lc*100:5.1f} cm  {rc*100:5.1f} cm')
    print(f'\n  MINIMUM CLEARANCE: {worst[0]*100:.1f} cm at {worst[1]:.2f} m')
    if offs: print(f'  route off-centre : mean {sum(offs)/len(offs)*100:+.1f} cm, worst {max(offs, key=abs)*100:+.1f} cm   (planner)')
    if trk:  print(f'  robot off-route  : mean {sum(trk)/len(trk)*100:+.1f} cm, worst {max(trk, key=abs)*100:+.1f} cm   (controller)')
    print('  (+ = left, - = right)')

if __name__ == '__main__':
    main()
