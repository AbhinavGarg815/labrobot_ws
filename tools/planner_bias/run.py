#!/usr/bin/env python3
"""Does the global planner centre the robot in a gap? Test it with no robot.

    python3 tools/planner_bias/run.py

Draws synthetic maps (8 x 8 m, 5 cm cells) with one 0.4 m thick wall across
the map and a single opening of known width and centre, starts the real Nav2
planner on them (rig.launch.py: map_server + planner_server, the robot's own
footprint/inflation/planner settings), and measures how far the route runs
from the true centre of the opening.

Each opening is crossed in both directions, and walls run along both map axes.
That separates the two possible kinds of bias:
  * tied to the ROBOT  (e.g. always to its right) -> flips sign in map terms
    when the direction of travel reverses;
  * tied to the MAP GRID (e.g. a half-cell shift towards -x/-y) -> same sign in
    map terms whichever way the robot travels.
All three narrow-gap runs on hardware ran 4-7 cm to the robot's right while
heading roughly +x in the map, so either explanation fits; this decides.
"""
import os, subprocess, sys, time, signal, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.environ.get('PB_OUT', '/tmp/planner_bias')
RES, SIZE, ORIGIN = 0.05, 160, -4.0

def make_map(name, wall_axis, lo, hi):
    """wall_axis 'x': wall spans y (a band in x = -0.2..0.2), opening y in [lo,hi].
       wall_axis 'y': wall spans x (band in y), opening x in [lo,hi]."""
    os.makedirs(OUT, exist_ok=True)
    g = np.full((SIZE, SIZE), 254, np.uint8)            # [row=y index, col=x index]
    c = ORIGIN + (np.arange(SIZE) + 0.5) * RES             # cell centres
    X, Y = np.meshgrid(c, c)
    if wall_axis == 'x':
        wall = (np.abs(X) < 0.2) & ~((Y > lo) & (Y < hi))
    else:
        wall = (np.abs(Y) < 0.2) & ~((X > lo) & (X < hi))
    g[wall] = 0
    pgm = os.path.join(OUT, name + '.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{SIZE} {SIZE}\n255\n'.encode()); f.write(np.flipud(g).tobytes())
    yml = os.path.join(OUT, name + '.yaml')
    open(yml, 'w').write(f'image: {pgm}\nresolution: {RES}\norigin: [{ORIGIN}, {ORIGIN}, 0.0]\n'
                         'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
    return yml

def pose(n, x, y, yaw):
    p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = n.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    return p

def plan(n, ac, start, goal):
    g = ComputePathToPose.Goal(); g.start = pose(n, *start); g.goal = pose(n, *goal)
    g.use_start = True; g.planner_id = 'GridBased'
    f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    r = f.result().get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=15)
    return [(p.pose.position.x, p.pose.position.y) for p in r.result().result.path.poses]

PLANNERS = {
    'navfn': '''      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.25
      use_astar: true
      allow_unknown: false''',
    'smac2d': '''      plugin: nav2_smac_planner/SmacPlanner2D
      tolerance: 0.25
      downsample_costmap: false
      allow_unknown: false
      max_planning_time: 2.0
      cost_travel_multiplier: 2.0
      use_final_approach_orientation: false''',
    'thetastar': '''      plugin: nav2_theta_star_planner/ThetaStarPlanner
      how_many_corners: 8
      w_euc_cost: 1.0
      w_traversal_cost: 2.0
      allow_unknown: false''',
}

def params_for(planner):
    base = open(os.path.join(HERE, 'params.yaml')).read()
    a = base.index('      plugin: nav2_navfn_planner/NavfnPlanner'); b = base.index('global_costmap:')
    out = os.path.join(OUT, f'params_{planner}.yaml'); os.makedirs(OUT, exist_ok=True)
    open(out, 'w').write(base[:a] + PLANNERS[planner] + '\n' + base[b:])
    return out

def main():
    planner = sys.argv[1] if len(sys.argv) > 1 else 'navfn'
    params = params_for(planner)
    print(f'  planner: {planner}')
    cases = []   # (map name, wall axis, opening lo, hi)
    for w, (lo, hi) in {'0.60 (12 cells, centre on a cell edge)': (-0.30, 0.30),
                        '0.65 (13 cells, centre mid-cell)': (-0.30, 0.35)}.items():
        cases.append((w, 'x', lo, hi)); cases.append((w, 'y', lo, hi))
    print('  opening                                  wall  travel   route offset from true centre')
    for i, (w, axis, lo, hi) in enumerate(cases):
        yml = make_map(f'gap{i}', axis, lo, hi)
        proc = subprocess.Popen(['ros2', 'launch', os.path.join(HERE, 'rig.launch.py'), f'map:={yml}', f'params:={params}'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            rclpy.init(); n = Node('planner_bias'); ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
            if not ac.wait_for_server(timeout_sec=30): print('  planner did not start'); return 1
            time.sleep(3.0)                     # let the static map reach the costmap
            ctr = (lo + hi) / 2
            for sgn, label in ((+1, '+'), (-1, '-')):
                if axis == 'x':
                    path = plan(n, ac, (-2.5*sgn, ctr, 0 if sgn > 0 else math.pi), (2.5*sgn, ctr, 0 if sgn > 0 else math.pi))
                    inside = [y for x, y in path if abs(x) < 0.2]; lat_axis = 'y'
                else:
                    path = plan(n, ac, (ctr, -2.5*sgn, math.pi/2*sgn), (ctr, 2.5*sgn, math.pi/2*sgn))
                    inside = [x for x, y in path if abs(y) < 0.2]; lat_axis = 'x'
                if not inside: print(f'  {w:40s} {axis}    {label}{"x" if axis=="x" else "y"}     NO PATH'); continue
                off = np.mean(inside) - ctr
                # Robot's right: travelling +x it is -y, -x -> +y, +y -> +x, -y -> -x.
                robot_right = (-off * sgn) if axis == 'x' else (off * sgn)
                print(f'  {w:40s} {axis}    {label}{"x" if axis=="x" else "y"}     '
                      f'{off*100:+5.1f} cm in map {lat_axis}   ({robot_right*100:+5.1f} cm to the robot\'s right)')
            n.destroy_node(); rclpy.shutdown()
        finally:
            os.killpg(proc.pid, signal.SIGINT); proc.wait(timeout=20)
    return 0

if __name__ == '__main__':
    sys.exit(main())
