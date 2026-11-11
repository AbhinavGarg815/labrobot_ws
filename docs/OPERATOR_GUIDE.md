# labrobot — Operator Guide

How to run the robot day to day: connect, start, localise, drive, stop, and
look after the map. Every command here is run **on the Jetson, from
`~/ros2_ws`**, unless it says "on your PC".

For the engineering history (why things are the way they are, every problem
and fix) see `docs/PROJECT_REPORT.md`. This guide is kept up to date with it.

Last updated: 2026-11-11.

---

## 1. What is what

| Part | What it does |
|---|---|
| Jetson Orin Nano (`ubuntu.local`) | Runs everything: ROS 2 Humble, camera, mapping, navigation |
| Arduino Mega (`/dev/labrobot_mcu`) | Talks to the two RMCS-2303 motor drives; **brakes the robot by itself if commands stop arriving** (watchdog) |
| RealSense D435i | The only sensor: colour + depth. Used for the map, localisation and obstacle avoidance. **Faces forward only** — the robot cannot see beside or behind itself until it turns |
| RTAB-Map | Builds and stores the map (`~/.ros/rtabmap.db`) and tracks where the robot is on it |
| Nav2 | Plans routes, drives, avoids obstacles |
| auto_localise | At startup: turns the robot slowly in place to work out where it is on the map |
| Foxglove | Your view of the robot from the PC, over an SSH tunnel |

The robot is about 41 cm wide and needs about **0.35 m clear all round** to turn
in place. Top speed while navigating is slow on purpose (6 cm/s).

---

## 2. Connecting

**On your PC**, two terminals:

```bash
ssh zine@ubuntu.local                                   # terminal 1: a shell on the robot
ssh -N -L 8765:localhost:8765 zine@ubuntu.local          # terminal 2: Foxglove tunnel (leave it open)
```

Then in the Foxglove app: *Open connection* → *Foxglove WebSocket* →
`ws://localhost:8765`.

**Foxglove setup (once):**

- **3D panel** → settings (gear) → *Frame* → **Display frame = `map`**.
  This matters: goals you click are sent in the display frame, and a goal in
  `odom` fails after ~10 s (see section 8).
- Useful topics to show: `/map`, `/global_costmap/costmap`, `/plan`,
  `/camera/camera/color/image_raw/compressed`,
  `/labrobot/localisation_status`, `/labrobot/mission_status`.
- Two **Publish** panels for missions, both on topic `/labrobot/mission_cmd`,
  schema `std_msgs/msg/String`: one with `{"data": "go"}`, one with
  `{"data": "stop"}`.

Raw images and raw point clouds are deliberately not sent over Wi-Fi (they
starved the robot's CPU once); the compressed image is.

---

## 3. Daily use

### 3.1 Start

1. Put the robot on the floor with **~0.35 m clear all round**. Power it on and
   wait about a minute for the Jetson to boot.
2. On the robot:

   ```bash
   cd ~/ros2_ws
   scripts/bringup_nav.sh --localization      # map frozen (current recommendation)
   ```

   It returns after about 2 minutes with a line like
   `READY mode=frozen robot=docs/logs/nav_1635.log ...`. Progress at any time:
   `cat docs/logs/bringup_status`.
3. **The robot then turns slowly in place to localise itself.** Keep clear. It
   may also drive ~0.6 m towards open space and turn again (up to 4 spots).
4. Watch the result in Foxglove (`/labrobot/localisation_status`) or:

   ```bash
   tail -f $(ls -t docs/logs/nav_*_autoloc.log | head -1)
   ```

   - `[LOCALISED] ...` — done, go to 3.2.
   - `NOT localised after 4 viewpoints` or `only turned N deg` — see 3.5.

Without `--localization` the map keeps updating as the robot drives
("lifelong mapping"). That is **paused** for now: it has twice rotated the map
by 4-5° (see section 6.4).

### 3.2 Check the robot is where the map says

In Foxglove (display frame `map`), the robot model should sit where the real
robot is and face the same way. If it is clearly off, re-localise (3.5) before
driving.

### 3.3 Drive: three ways

**a) One goal, straight from Foxglove.** 3D panel → *Publish pose* → topic
`/goal_pose` → click the destination and drag for the final heading. Nav2
drives there immediately. Watch `/plan` for the route.

**b) Named waypoints and a mission** (the robot checks each waypoint against
the live map before driving to it):

```bash
scripts/pick_waypoints.sh a            # click waypoints in Foxglove; saved as a1, a2, ... (robot does NOT move)
                                       #   Publish pose -> topic /waypoint_pick (NOT /goal_pose). Ctrl-C when done
python3 tools/check_waypoints.py a1 a2 a3     # are they clear and reachable? (robot does not move)
ros2 run labrobot_bringup waypoints.py go a1 a2 a3 --wait-start   # waits for "go" in Foxglove
```

Then press **go** in Foxglove. During the mission:

- each waypoint reports `REACHED`, `APPROACHED` (an obstacle now covers it, so
  the robot stopped at the nearest reachable spot within 0.5 m), `BLOCKED`
  (nothing reachable within 0.5 m — skipped), or `FAILED` (tried twice);
- **stop** in Foxglove, or Ctrl-C in the terminal, cancels the robot's goal.

Other mission options: `--return` (come back to the start), `--loop 3`
(repeat the route), `--timeout 600` (give up after 600 s). Without
`--wait-start` the mission starts immediately.

Before it moves, a mission checks that the robot is localised and that the
camera view matches the map. If not, it prints
`NOT starting the mission` and the reason; see 3.5.

**c) Relative moves** (handy for small tests):

```bash
ros2 run labrobot_bringup nav_goal.py --forward 1.0              # 1 m ahead
ros2 run labrobot_bringup nav_goal.py --forward 1.0 --left 0.5 --turn 90
```

Managing waypoints:

```bash
ros2 run labrobot_bringup waypoints.py list          # names, positions, distance from the robot
ros2 run labrobot_bringup waypoints.py save door      # the robot's CURRENT pose as "door"
ros2 run labrobot_bringup waypoints.py add desk 1.2 -0.5 90   # x y (m) and heading (deg) on the map
ros2 run labrobot_bringup waypoints.py delete door
```

Waypoints live in `docs/maps/waypoints.yaml`. **They belong to the map they
were picked on**: after building a new map, pick them again.

### 3.4 Stop and power off

```bash
scripts/stop_robot.sh          # stop everything; RTAB-Map saves the map (can take up to ~90 s)
scripts/shutdown_robot.sh      # stop everything, flush the disk, power the Jetson off
```

**Always use one of these before switching off at the battery.** Pulling the
battery while the stack runs has cost two maps. `shutdown_robot.sh` ends with
`Powering off - disconnect the battery when the Jetson's LEDs go off`: wait
for the LEDs, then disconnect.

`stop_robot.sh` also checks the map was not distorted during the session
(6.4) and keeps a known-good copy, `~/.ros/rtabmap.db.lastclean`.

### 3.5 When the robot is not localised (or was moved by hand)

```bash
scripts/localise.sh                     # ROBOT TURNS: the same search as at startup
```

If it still fails but **you can see in Foxglove that the pose shown is right**:

```bash
scripts/localise.sh --confirm           # accept it if visual matches agree + the view check passes
scripts/localise.sh --confirm-geometry  # accept on your check + the view check (no visual matches needed)
```

If the pose shown is **wrong**, do not confirm it. Move the robot into open
floor with walls and furniture in view and run `scripts/localise.sh` again.
`scripts/localise.sh --dry-run` shows what it would run without doing it.

### 3.6 Emergency stop

- Fastest: **stop** in Foxglove (missions) or Ctrl-C in the mission terminal.
- Everything: `scripts/stop_robot.sh` — all nodes stop; the Arduino watchdog
  brakes the wheels as soon as commands stop.
- Physical: lift the drive wheels / cut the battery. (Last resort — see 3.4.)

---

## 4. Building a new map

Do this when the lab has changed a lot, or when localisation keeps failing.

1. **Back up the current map** — starting a new map **deletes**
   `~/.ros/rtabmap.db`:

   ```bash
   scripts/stop_robot.sh
   cp ~/.ros/rtabmap.db docs/maps/backup/lab_before_new_map_$(date +%Y%m%d).db
   ```
2. Start mapping (terminal 1), and **keep the robot still for the first 30 s**
   (the gyro calibrates itself):

   ```bash
   ros2 launch labrobot_bringup robot.launch.py mapping:=true
   ```
3. Drive by keyboard (terminal 2):

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
       -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
   ```

   `i` forward, `,` back, `j`/`l` turn, `k` stop. Press `x` repeatedly first
   to bring the linear speed down to about 0.1.
4. How to drive for a good map:
   - slowly, smooth turns; no fast spins (the wheels slip);
   - **always move, not just rotate** — the camera cannot see the floor right
     next to the robot, so a spot is only mapped once you have driven past it;
   - cover every area you will want to navigate, looking at walls and furniture;
   - come back past places you have already mapped (that is what lets
     RTAB-Map correct drift), and finish near where you started;
   - keep people out of view if you can — anything seen standing still ends up
     in the map (it can be removed later, 6.3).
   - Live quality while you drive: `ros2 run labrobot_bringup monitor_slam.py`.
5. Finish with `scripts/stop_robot.sh` (Ctrl-C the teleop first). Then keep a
   copy:

   ```bash
   cp ~/.ros/rtabmap.db docs/maps/backup/lab_v7_$(date +%Y%m%d).db
   ```
6. Pick new waypoints (3.3) — the old ones belong to the old map.

---

## 5. Health checks (robot does not move)

| Command | Tells you |
|---|---|
| `ros2 run labrobot_bringup check_system.py` | every topic's rate, CPU, RAM, temperatures |
| `ros2 run labrobot_bringup localisation_check.py` | does the camera view match the map at the current pose (`OK` / `WRONG`) |
| `ros2 run labrobot_bringup check_costmap.py --span 1.0` | the obstacle map around the robot, and whether its footprint is clear |
| `python3 tools/depth_ahead.py` | what the camera actually sees ahead, top-down |
| `python3 tools/check_waypoints.py NAME ...` | are waypoints clear and reachable |
| `ros2 run labrobot_bringup check_map.py` | map quality: loop closures, drift |
| `scripts/check_camera_usb.sh` | is the camera on USB 3 (needed) |
| `ros2 control list_hardware_components` | motor driver state — must be `active` |

---

## 6. The map

### 6.1 Files

| File | What it is |
|---|---|
| `~/.ros/rtabmap.db` | **the active map** |
| `~/.ros/rtabmap.db.lastclean` | known-good copy, refreshed by `stop_robot.sh` after every clean close |
| `docs/maps/backup/` | every saved map and backup (not in git — too large) |
| `docs/maps/waypoints.yaml` | named waypoints (in git) |

Active since 2026-11-11: `lab_v6_clean_20261027.db` (the original 27 Oct map).

### 6.2 Switching to another saved map

```bash
scripts/stop_robot.sh
cp docs/maps/backup/<MAP>.db ~/.ros/rtabmap.db
cp docs/maps/backup/<MAP>.db ~/.ros/rtabmap.db.lastclean   # keep the known-good copy the SAME map
scripts/bringup_nav.sh --localization
```

Waypoints picked on a different map will be in the wrong place.

### 6.3 Removing something wrongly mapped (a person, a bag)

With the stack stopped, find the box in map coordinates (metres) in Foxglove,
then:

```bash
python3 tools/erase_from_map.py --box X0 X1 Y0 Y1 --dry-run   # how much it would remove
python3 tools/erase_from_map.py --box X0 X1 Y0 Y1             # do it (backs up the map first)
```

### 6.4 The drift guard

After each session, `stop_robot.sh` compares the old part of the map with the
known-good copy (`tools/map_drift.py`). If the session rotated the map more
than 1.5° or moved its nodes more than 8 cm on average, it prints
`WARNING: this session DISTORTED the saved map`, moves that map to
`docs/maps/backup/distorted_<time>.db` and restores the known-good one — so
your waypoints stay valid. A brand-new map is recognised and simply becomes
the new known-good copy. Check two maps by hand:

```bash
python3 tools/map_drift.py docs/maps/backup/A.db ~/.ros/rtabmap.db
```

---

## 7. Logs

Everything is in `docs/logs/`, named by start time:

| File | Contents |
|---|---|
| `bringup_status` | one line: how far the last bringup got, the map mode, the log names |
| `nav_HHMM.log` | camera, motors, RTAB-Map |
| `nav_HHMM_nav2.log` | Nav2 (planning, driving, recoveries) |
| `nav_HHMM_autoloc.log` | startup localisation |
| `localise_*.log` | `scripts/localise.sh` runs |
| `autoloc/` | every localisation attempt: scan, verdict and a picture |

Quick searches: `grep -E "Goal (succeeded|failed)" docs/logs/nav_*_nav2.log`,
`grep "serial write" docs/logs/nav_*.log` (motor link trouble).

---

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Foxglove goal fails after ~10 s; Nav2 log: `Extrapolation Error ... into the past ... from frame [odom]` | The 3D panel's display frame is `odom`. Set it to **`map`** and send again. |
| Mission: `NOT starting the mission ... current view does not match the map` | The pose is off. `scripts/localise.sh`; if Foxglove shows the right pose, `--confirm`. |
| Localisation: `NOT localised after 4 viewpoints` | Drive or carry the robot to open floor with structure in view; `scripts/localise.sh`. If the pose shown is right: `--confirm` / `--confirm-geometry`. |
| `only turned N deg (obstacle within ~0.3 m?)` or `Collision Ahead - Exiting Spin` with nothing there | Stale obstacles in the local costmap (often after a hand move). Clear it: `ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"` — only if the space really is clear. |
| Robot will not move at all; `list_hardware_components` not `active` | The motor driver stopped. `scripts/stop_robot.sh`, then start again. Look for `serial write failed (...)` in the robot log. |
| Bringup hangs, or Nav2 never becomes active after a partial restart | Stale DDS files: `scripts/stop_robot.sh`, `rm -f /dev/shm/fastrtps_*`, start again. |
| Camera missing, or `check_camera_usb.sh` says USB 2 | Re-plug the camera cable firmly (USB 3 port); run the check again. |
| `rtabmap` dies at startup with `descriptors empty` / `doesn't match loaded words` | The map was not closed cleanly (battery pulled). `cp ~/.ros/rtabmap.db.lastclean ~/.ros/rtabmap.db`. |
| Stop / kill commands seem to kill your own terminal | Never `pkill -f` a pattern that is also in your command line; use `scripts/stop_robot.sh`. |

---

## 9. Known limitations (2026-11-11)

- **Lifelong mapping is paused** (start with `--localization`): two sessions
  each rotated the map 4-5°. The drift guard now catches that, but the cause
  is not yet fixed.
- In the middle of the lab, RTAB-Map's position on the 27 Oct map can be
  ~25 cm off (its old images no longer match well). Missions there may refuse
  to start (view check). A fresh map is the planned fix.
- Foxglove goals need display frame `map` (a goal relay that removes this is
  planned).
- The robot cannot see beside or behind itself; after a hand move, obstacles it
  remembered are in the wrong place until cleared or seen again.

---

## 10. Reference: scripts and tools

**`scripts/`** (operator scripts)

| Script | Purpose |
|---|---|
| `bringup_nav.sh [--localization] [--no-autoloc]` | start everything + startup localisation |
| `localise.sh [--confirm \| --confirm-geometry] [--dry-run]` | localise again while running |
| `pick_waypoints.sh PREFIX` | save waypoints clicked in Foxglove |
| `stop_robot.sh` | stop everything, save the map, drift guard |
| `shutdown_robot.sh` | stop + power off safely |
| `check_camera_usb.sh` | camera on USB 3? |
| `soak_test_camera.sh MIN` | long camera link test |
| `00_setup_jetson.sh`, `01_install_realsense_udev.sh`, `02_build_librealsense_rsusb.sh` | one-time Jetson setup |

**`ros2 run labrobot_bringup ...`**

| Node / script | Purpose |
|---|---|
| `waypoints.py` | `save`, `pick`, `add`, `list`, `delete`, `go` (missions) |
| `nav_goal.py` | a goal relative to the robot |
| `auto_localise.py` | startup localisation (normally via the scripts above) |
| `localisation_check.py` | camera view vs map at the current pose |
| `check_costmap.py`, `check_system.py`, `check_map.py`, `monitor_slam.py` | health checks |

**`tools/`** (diagnostics)

| Tool | Purpose |
|---|---|
| `check_waypoints.py` | waypoints vs the live costmap |
| `map_drift.py` | did a session bend the map |
| `erase_from_map.py` | remove a wrongly mapped object |
| `depth_ahead.py` | what the camera sees, top-down |
| `show_pose_candidates.py` | show candidate poses in Foxglove so you can say which is real |
| `record_path.py` | record planned and driven paths of a run |
