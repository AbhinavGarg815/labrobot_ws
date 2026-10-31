# labrobot — Project Report

Living record of the labrobot build: what each file is for, what broke, and how
it was fixed. Updated as work proceeds, not at the end.

**Last updated:** 2026-08-03 · **Current milestone:** M3 (SLAM) · M0–M2 complete, M3.T1 in progress

---

## 1. What this robot is

A two-wheeled differential-drive indoor robot on four fixed casters. Its defining
constraint is that **perception is camera-only** — a single Intel RealSense D435i,
no lidar. Goal: SLAM, point-to-point navigation, and obstacle avoidance.

| Item | Detail |
|---|---|
| Compute | Jetson Orin Nano 8GB devkit (p3767-0003), JetPack 6.x / L4T R36.5 |
| OS / ROS | Ubuntu 22.04, ROS 2 Humble |
| MCU | Arduino Mega 2560, USB → Jetson, pinned to `/dev/labrobot_mcu` |
| Motor drives | 2 × RMCS-2303, MODBUS ASCII 9600 baud, slave 7, one per Mega UART |
| Motors | 2 × RMCS-5030, 13-line encoder, 1:241 gearbox, 60 wheel-rpm max |
| Camera | Intel RealSense D435i on a mast at z ≈ 0.844 m, pitched 15° down |
| Power | 3S LiPo 8000 mAh 30C → drives directly; Jetson via 13 V converter |

### Geometry

| Property | Value | Source |
|---|---|---|
| Base footprint | 0.4203 × 0.4068 m | measured from `base_link.stl` |
| Total height | 0.857 m | mesh |
| Mast section | 21.4 × 115.0 mm, z 0.15 → 0.83 | mesh cross-section |
| Wheel radius / width | 0.050 m / 0.026 m | measured |
| Wheel separation | **0.274 m** geometric, **0.2669 m** calibrated | spin test vs tape |
| Casters | 4 **swivel** (castoring), one per corner | user-confirmed |
| Counts per wheel rev | **12532** (52 × 241) | verified on hardware to 0.2% |

Wheels (±0.15 m) sit **inside** the base half-width (±0.203 m), so the Nav2
footprint is just the base rectangle.

### Kinematic ceiling

60 wheel-rpm is a hard mechanical limit → **v_max 0.314 m/s**, ω_max 2.29 rad/s.
Configured limits are 0.26 m/s and 1.5 rad/s. This is a slow robot, which
actually suits camera-only SLAM (little motion blur).

---

## 2. Repository layout

### `src/labrobot_description/` — robot model (ROS package)

| File | Purpose | When you touch it |
|---|---|---|
| `urdf/labrobot_properties.xacro` | **Single source of truth for every calibratable number** | Any time a physical constant changes. Nowhere else. |
| `urdf/labrobot.urdf.xacro` | Top-level model: links, joints, collisions, camera frames | Structural changes only |
| `urdf/labrobot.ros2_control.xacro` | One macro, two plugins (Gazebo vs. our RMCS-2303 bridge) | Adding hardware params |
| `urdf/labrobot.gazebo.xacro` | Sim-only sensors and plugins | Simulation work |
| `urdf/materials.xacro` | Colours | Rarely |
| `config/controllers.yaml` | **Hardware** controller config | Tuning the real robot |
| `config/controllers_gazebo.yaml` | **Sim** controller config | Keep in sync with the above |
| `launch/display.launch.py` | RViz / TF only, no hardware | Checking the model |
| `launch/gazebo.launch.py` | Gazebo Classic bringup | Simulation |
| `meshes/*.stl` | Visual meshes (collision uses primitives) | On CAD re-export |

Build target selection is one argument:

```bash
xacro labrobot.urdf.xacro                 # real robot (default)
xacro labrobot.urdf.xacro use_sim:=true   # Gazebo
```

### `src/labrobot_hardware/` — ros2_control plugin (ROS package)

| File | Purpose | When you touch it |
|---|---|---|
| `src/rmcs2303_system.cpp` | The `SystemInterface`: lifecycle, `read()`/`write()`, unit conversion, link health | Changing what ros2_control sees |
| `src/serial_port.cpp` | Non-blocking POSIX serial. **Never blocks** — a stalled write would stall every controller | Rarely |
| `labrobot_hardware.xml` | pluginlib declaration. The class name must match the `<plugin>` in the URDF exactly | Renaming the plugin |
| `CMakeLists.txt` | Compiles the firmware's `labrobot_protocol.cpp` **into this library**, so both ends of the link share one wire format | Adding sources |

### `src/labrobot_bringup/` — launch files (ROS package)

| File | Purpose |
|---|---|
| `launch/drivetrain.launch.py` | Real-hardware bringup: `robot_state_publisher` + `ros2_control_node` + both controllers |

### `firmware/` — Arduino Mega firmware (not a ROS package)

Deliberately outside `src/` so `colcon` ignores it.

| File | Purpose |
|---|---|
| `labrobot_firmware/rmcs2303.{h,cpp}` | Non-blocking MODBUS ASCII master. One transaction at a time, never blocks, never allocates. |
| `labrobot_firmware/wheel_drive.{h,cpp}` | Per-wheel sequencing on top of the driver: what to send next, encoder cache, direction inversion, health. |
| `labrobot_firmware/labrobot_protocol.{h,cpp}` | Jetson wire format: framing, CRC-16, encode/decode. **Arduino- and ROS-free**, so firmware, tests and the future ros2_control plugin share one definition. |
| `labrobot_firmware/host_link.{h,cpp}` | Binds the protocol to the wheels: command dispatch, unit conversion, status at 50 Hz, comms watchdog. |
| `labrobot_firmware/labrobot_firmware.ino` | Both modes: production protocol (default) and bench console (`--bench`). |
| `host_client.py` | Host-side protocol client: status, ping, drive, watchdog test. Always sends a stop on exit, including Ctrl-C. |
| `test/run_tests.sh` | **Run this before flashing.** Builds and runs all host tests. |
| `test/test_rmcs2303.cpp` | Framing / LRC / parser tests |
| `test/test_wheel_drive.cpp` | State-machine, inversion, fault-recovery tests |
| `test/test_protocol.cpp` | Framing, CRC, exhaustive single-bit corruption |
| `test/test_host_link.cpp` | Command dispatch, conversions, watchdog |
| `test/Arduino.h` | Minimal Arduino shim for host compilation |
| `test/fake_drive.h` | Simulated RMCS-2303 with an **independent** MODBUS implementation |
| `flash.sh` | Test → compile → upload → monitor |
| `serial_console.py` | Non-interactive serial console (works over SSH, no pyserial) |

### `scripts/`

Host setup and hardware diagnostics. Not ROS packages — run directly.

| File | Purpose |
|---|---|
| `00_setup_jetson.sh` | One-time provisioning: `dialout` group, udev rule, ROS + AVR packages |
| `01_install_realsense_udev.sh` | udev rule giving write access to the raw USB node, which libusb must claim |
| `02_build_librealsense_rsusb.sh` | Builds librealsense with `FORCE_RSUSB_BACKEND=ON`. **Required for the IMU** — see Problem 20 |
| `check_camera_usb.sh` | One second. Reports the negotiated USB link speed. **Run after any cable or port change** |
| `stop_robot.sh` | Stops every node **without killing the calling shell** — see the pkill trap below |
| `soak_test_camera.sh [min]` | The rigorous camera link check. Phase 1 measures the link itself, phase 2 the ROS pipeline — see Problem 23c |
| `measure_camera_cost.sh` | CPU cost per camera stream configuration, used to choose the profiles in `camera.launch.py` |
| `udev/` | The rule files installed by the scripts above |

**`labrobot_bringup` scripts worth knowing**

| File | Purpose |
|---|---|
| `check_system.py` | Every topic rate + per-node CPU/RSS + RAM + thermals in one table. **Run after anything that adds a consumer.** |
| `compare_odom.py` | Passive wheel-odometry vs EKF recorder. Run alongside any motion |
| `drive_test.py` | Commanded straight / rotate / UMBmark square. Closes its loop on the **EKF**, not the wheels |
| `imu_bias_compensator.py` | Removes the gyro bias the rest of the stack cannot |
| `check_ground_plane.py`, `analyse_floor.py` | Camera mount pose and depth quality from the floor |
| `check_imu.py` | IMU characterisation |

**`labrobot_bringup` launch files**

| File | Purpose |
|---|---|
| `camera.launch.py` | D435i. `fps:=15` by default - see the resource budget |
| `drivetrain.launch.py` | ros2_control + diff_drive_controller |
| `localization.launch.py` | Madgwick + bias compensator + EKF |
| `visual_odometry.launch.py` | RTAB-Map RGB-D odometry, **measurement only** (no TF, no motion prior) |
| `mapping.launch.py` | RTAB-Map SLAM. Takes odometry from the **EKF**, not its own VO — see M3.T1. Owns `map -> odom` only |
| `foxglove.launch.py` | WebSocket bridge for Foxglove on your PC: `ws://<jetson-ip>:8765` |
| `robot.launch.py` | **Everything, correctly ordered and delayed.** `mapping:=true` / `localization:=true` |

### `tools/`

| File | Purpose |
|---|---|
| `rs_link_probe.cpp` | Frame-accurate D435i link probe: talks to librealsense directly with no ROS in the path, counting dropped frames from the frame number. Built on demand by `soak_test_camera.sh` into `build/rs_link_probe` |

### `docs/`

This report.

---

## 3. Common commands

```bash
# Build (ALWAYS use --symlink-install; see Problem 3)
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# Bring up the real drivetrain (hardware)
ros2 launch labrobot_bringup drivetrain.launch.py
ros2 control list_controllers
ros2 topic echo /diff_drive_controller/odom --once

# Drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

# Check the model / TF tree, headless over SSH
ros2 launch labrobot_description display.launch.py gui:=false rviz:=false
ros2 run tf2_ros tf2_echo base_footprint camera_color_optical_frame

# Camera: bring up, and verify the USB link after ANY cable or port change
ros2 launch labrobot_bringup camera.launch.py
scripts/check_camera_usb.sh          # 1 second: is it USB 3?
scripts/soak_test_camera.sh 8        # 8 min: is the link actually clean?

# Firmware: test only, board untouched
firmware/flash.sh --check

# Firmware: production build (binary protocol) + upload
firmware/flash.sh

# Firmware: bench console for commissioning the drives
firmware/flash.sh --bench

# Talk to production firmware (binary; a text monitor shows garbage)
firmware/host_client.py --status
firmware/host_client.py --drive 10 10 --seconds 4     # wheel-rpm per side
firmware/host_client.py --watchdog-test               # prove it self-stops

# Scripted (non-interactive) console; 'cmd:secs' sets the hold time
firmware/serial_console.py --send 'h:1' --send 'f:4' --send 's:1' --seconds 2
```

### Bench console keys

| Key | Action |
|---|---|
| `f` / `b` | forward / reverse |
| `l` / `r` | spin left / right |
| `s` | brake |
| `n` | neutral — release motors so wheels turn by hand |
| `e` | E-stop (cuts motor power) |
| `h` | zero encoders |
| `+` / `-` | test speed ±5 wheel-rpm |
| `?` | status once |
| `d` | raw bytes of the last Modbus exchange |
| `p` | probe how the drive interprets the read count field |

---

## 4. Design decisions

**ros2_control hardware interface, not a Python bridge.** Reuses the existing
`diff_drive_controller` config, gives standard odometry/TF and a real-time loop,
and lets one controller config drive both sim and hardware.

**RTAB-Map RGB-D + robot_localization EKF for SLAM.** Apt-installable, CPU-
friendly, produces both a 3D map and the 2D occupancy grid Nav2 needs. Suits a
high, tilted, camera-only sensor better than faking a laser scan from one depth
row. Isaac ROS cuVSLAM was rejected for Docker complexity on a tethered link.

**Own MODBUS driver, not the vendor `RMCS2303drive` library.** Its `port` is a
file-scope global, so two drives on two UARTs is *impossible*; its LRC handler
calls `port->end()` on a single glitch, permanently killing the motor link; it
blocks `delay(50)` per write; and it uses Arduino `String` on an 8 KB machine.

**One drive per UART, both at slave 7.** Legal because the links are
point-to-point, and deliberate: separate UARTs let the two Modbus transactions
overlap. A shared bus would halve the encoder rate. **Do not "consolidate" this.**

**Velocity is differentiated from position, not read from the drive.** See
Problem 6.

**One shared protocol header, no ROS or Arduino dependencies.** Two hand-kept
copies of a wire format drift, and the drift appears as a robot acting on stale
commands. The Python client is cross-checked against the C++ encoder byte for
byte, so the two cannot silently diverge.

**Velocity on the wire is milli-radians/second at the wheel.** Full scale is
6283 (60 wheel-rpm), well inside int16, and 1 mrad/s is far finer than the
encoder resolves in a control period, so the unit never limits precision.

**Zero speed brakes rather than disables.** A disabled drive freewheels, and this
robot has no parking brake.

---

## 5. Problem log

Each entry: symptom → root cause → fix → how it was proven.

### Problem 1 — `wheel_separation` was 10.8% wrong
**Symptom:** none yet; found by inspection.
**Cause:** config used `0.2445`, the distance between wheel *joint origins*. The
contact patch is 16 mm further out per side.
**Fix:** `0.274` = 0.300 m outer-face spacing − 0.026 m tyre width.
**Impact:** would have under-rotated every commanded turn by ~11%.

### Problem 2 — velocity limits 3× beyond physical capability
**Cause:** config claimed 1.0 m/s and 2.0 rad/s; the motors cap at 0.314 m/s.
**Fix:** 0.26 m/s, 1.5 rad/s. The datasheet warns that commanding an unreachable
speed gives an "uncertain result".

### Problem 3 — edits silently ignored (stale install space)
**Symptom:** changed a value in `labrobot_properties.xacro`, rebuilt, and the
generated URDF still had the old number.
**Cause:** `$(find labrobot_description)` inside xacro resolves to the **install
space**. A normal `colcon build` copies files, so the included xacro was stale.
**Fix:** always `colcon build --symlink-install`.

### Problem 4 — truncated mesh, then renamed meshes
**Symptom (a):** `base_link.stl` header declared 295,162 triangles but held
230,685 — cut off at exactly 11 MiB.
**Consequence:** measurements from the partial mesh were *wrong but plausible* —
the mast read 97 mm wide with an 8.9 mm offset, because the whole −57…−40 mm side
was missing. The complete file gives 115.0 mm, symmetric.
**Fix:** re-copied; now verifies as complete.
**Symptom (b):** re-exported meshes arrived as `left_wheel_link_1.stl` etc., so
three of four URDF references pointed at non-existent files.
**Fix:** renamed the files back to the clean names.
**Lesson:** always verify `84 + 50×triangles == filesize` before trusting a mesh,
and check that every mesh reference resolves after any CAD re-export.

### Problem 5 — every speed read rejected as malformed
**Symptom:** `crc` counter climbing ~14/s on both wheels, yet `to=0` (the drives
*were* answering) and position stuck at 0.
**Cause:** our parser required ≥15 chars — the length of a *write echo*. A
1-register read reply is **13 chars**. Position reads (2 registers, 17 chars)
passed; speed reads (13 chars) were all rejected.
**Fix:** minimum length 13.
**Proof:** raw dump showed `:0703020000F4` with a valid LRC (`07+03+02+00+00 =
0x0C`, two's complement `0xF4`).
**Also fixed:** the one "bad frame" counter was replaced with specific reasons
(`short`, `badlrc`, `wrongslave`, `timeout`, …) — a single counter cannot tell a
wiring fault from line noise from an unpowered drive.

### Problem 6 — the drive's speed register is too coarse for odometry
**Symptom:** reported rpm only ever took values 0, 450, 915, 1380, 1845.
**Cause:** exact steps of ~465 base-rpm. The drive samples 13 encoder lines over
a ~10 ms window → 1.93 wheel-rpm ≈ **0.010 m/s** resolution.
**Fix:** stopped reading register 24 entirely; velocity is differentiated from
the 32-bit position instead. Gives ~0.13 wheel-rpm (**14× finer**) *and* frees
every Modbus slot for position, doubling the sample rate to ~27 Hz.

### Problem 7 — motor took 30 s to reach speed
**Symptom:** commanded 10 wheel-rpm, only reached 8.0 before the test ended.
Easy to misdiagnose as a torque or PID problem.
**Cause:** acceleration register set to **500**. The datasheet reset value is
**0x4E20 = 20000** — we were 40× too low. Position data showed a perfectly linear
ramp of 2.02 wheel-rpm/s → 29.8 s to reach 60 rpm.
**Fix:** 20000. Setpoint now reached in under a second.

### Problem 8 — a diagnostic that lied
**Symptom:** the dump showed a *position* request beside a *speed* reply, which
made a healthy link look like it was answering the wrong question. It led to a
wrong conclusion, stated to the user, that the drive ignored 2-register reads.
**Cause:** `lastRequest` was latched at **send** time, so it had already advanced
to the next request while `lastFrame` still held the previous reply.
**Fix:** latch the request at **parse** time, so the pair always corresponds.
**Truth:** the `p` probe showed count=1 → byte count 02, count=2 → **byte count
04**, count=4 → clamped to 04. Two-register reads work; the 32-bit position *is*
read atomically.

### Problem 9 — "wheels are not moving" (they were)
**Symptom:** encoders logged 8372 counts tracking the setpoint to +0.4%, but no
wheel motion was visible.
**Cause:** not a fault. 10 wheel-rpm is **one revolution per 6 seconds**. Over a
4 s run the wheel turned 240°, with no mark on the tyre — while the base motor
span at 2410 rpm and looked obviously alive. "Base motor moving, output shaft
not" is what a healthy 1:241 gearbox looks like.
**Fix:** none needed. Re-tested at 40 wheel-rpm (1 rev / 1.5 s) and it was
unmistakable.
**Lesson:** use ≥40 wheel-rpm for anything a human is meant to eyeball.

### Problem 10 — robot spun instead of driving, then drove backwards
**Symptom:** "moved left then right", then after a fix "both moved backwards".
**Cause:** the two motors are mounted **mirrored**, so commanding both the same
rotational sense spins the robot.
**Fix:** `kInvertLeft = true, kInvertRight = false`. Inversion flips the
commanded direction bit **and** the reported encoder sign together — inverting
only the command would leave the robot driving correctly while odometry ran
backwards, which looks fine on a bench and corrupts the map later. Covered by
dedicated tests.

### Problem 11 — a "systematic" wheel asymmetry that wasn't (corrected)
**Symptom:** left ran 0.86% short of right at 10 rpm and 0.87% at 40 rpm. The
consistency across speeds was read as proof of a mechanical cause, and it was
reported as such — **wrongly**.
**Cause:** converting to time instead of percent: −73 counts = 35.0 ms, −291
counts = 34.8 ms. A constant **~35 ms** — one Modbus cycle. The wheels are polled
on independent ~37 ms cycles, so their zero and final samples land a cycle apart.
The offset scales with speed, which is *precisely why* the percentage looked
convincingly constant. A later run landed within 2 counts.
**Lesson:** bench encoder counts cannot measure wheel asymmetry. Only a ground
UMBmark run can. When a ratio looks suspiciously constant, re-express it in other
units before believing it.

### Problem 12 — no way to stop a runaway
**Cause:** the robot has **no kill switch** other than removing the battery, and
bench mode had no watchdog. A crashed console or dropped USB would leave the
wheels driving.
**Fix:** 4 s auto-brake in bench mode; any command re-arms it. Verified firing.
**Still open:** see Risks.

### Problem 13 — watchdog fired on the very command that should clear it
**Symptom:** with the Jetson streaming velocity at 20 Hz, the `WATCHDOG` flag
was set on nearly every status frame and the robot barely moved. The flag
flickered clear for one frame now and then.
**How it was found:** hypothesising was going nowhere, so host-link counters
(`host_frames` / `host_errors`) were added to the Status message. They showed
`rx=16/0err` — commands *were* arriving, at the right rate, with zero rejects.
That eliminated the link and the parser in one measurement and pointed straight
at the watchdog arithmetic.
**Cause:** `poll()` sampled `now = millis()` at the top, but `handleFrame()`
called `millis()` again for `last_command_ms_`. If even 1 ms had elapsed,
`last_command_ms_ > now`, and the **unsigned** `now - last_command_ms_` wrapped
to ~4.29e9 — instantly ≥ 500 ms.
**Fix:** one timestamp sampled per `poll()` and used everywhere, plus a signed
`int32_t` difference so the comparison also survives the 49-day millis()
rollover.
**Why the tests missed it:** the fake clock only advanced *between* iterations,
so `millis()` never changed inside a `poll()`. Fixed by giving the stub an
auto-advance mode where every `millis()` call ticks — reproducing how real time
behaves. The new regression test was verified to fail on the old code and pass
on the new.
**Lesson:** never compare two timestamps sampled at different points in a call
chain, and never use unsigned subtraction for elapsed-time comparisons. When a
theory will not converge, add a counter instead of another hypothesis.

### Problem 14 — controller_manager segfaulted on the first warning
**Symptom:** the plugin loaded, configured, opened the port and activated
cleanly, then `ros2_control_node` died with SIGSEGV in the RT thread. Controllers
never came up, and the only visible symptom was spawners timing out on
`/controller_manager/list_controllers` — which points nowhere near the cause.
**Cause:** `RCLCPP_WARN_THROTTLE(logger(), *rclcpp::Clock::make_shared(), ...)`.
The macro captures the clock by reference into a lambda; the temporary
`shared_ptr` is destroyed at the end of the full expression, so its deleter runs
and the lambda then calls `now()` on freed memory. The stack trace named the
deleter (`{lambda(long*)#1}`) directly above `rclcpp::Clock::now()`.
**Fix:** an owned `rclcpp::Clock throttle_clock_` member.
**Lesson:** the throttle macros need a clock that outlives the statement. Never
pass them a temporary. It only fires on the first *warning*, so a happy-path
test never reaches it.

### Problem 15 — brace-init built the wrong vector
**Cause:** `std::vector<double> hw_positions_{kNumJoints, 0.0}` does not make two
zeros. Brace-init prefers the `initializer_list` constructor, so it makes the two
VALUES `{2.0, 0.0}` — the left wheel would start life reporting 2 radians.
**Fix:** `= std::vector<double>(kNumJoints, 0.0)`, parenthesised.
**Found by:** reading the code while chasing Problem 14, not by a test — the
values were later overwritten in `on_activate`, so it would have stayed hidden
until something read joint state before activation.

### Problem 16 — 4.5% of status frames appeared to be dropping
**Symptom:** the first successful run reported 51 dropped frames out of 1136.
**Investigation:** the cumulative percentage could not distinguish a burst from a
steady leak, so the health log was changed to report drops **per 10 s window**.
**Result:** all drops occur in one burst while the controller spawners are
loading; every window after that is **0.00%** at a steady 49.9 Hz. Not a link
fault. It is the kernel tty buffer overflowing while the RT loop is held up
during controller configuration, and it costs nothing — encoder position is
absolute, so a gap costs latency, not distance.
**Lesson:** a cumulative average hides everything that matters. Report rates per
window, or a link that starts failing mid-mission will be masked by a clean
history.

### Problem 17 — plugin never received a single status frame
**Symptom:** the drivetrain launched cleanly, both controllers went active, but
`/joint_states` and odometry stayed frozen at zero and the robot would not move.
The hardware log stopped at "activated; waiting for first status frame" — and
stayed there for 14 minutes.
**Isolation:** killing ROS and running `firmware/host_client.py --status` against
the same board gave 199 frames at 50 Hz with zero errors. Firmware perfect, so
the fault was on the ROS side. Earlier runs of the *same* plugin had worked,
which made it a race rather than a logic error.
**Cause:** opening a tty asserts DTR, which **resets the Arduino into its
bootloader**. The plugin opened the port and immediately wrote a STOP frame, so
that frame landed in the bootloader — which may swallow it or take it for an
upload attempt — and the plugin then started reading before the sketch was
running. `host_client.py` had always slept 2.5 s for exactly this reason; the
plugin did not.
**Fix:** wait `bootloader_wait_ms` (default 2500) after opening, then flush,
before writing anything. Safe in `on_configure` — a lifecycle transition, not the
RT loop.
**Bonus:** this also removed the startup frame-drop burst from Problem 16. The
post-wait flush discards the backlog that used to overflow the kernel tty buffer,
so every window now reports **0.00%**, including the first.
**Lesson:** any host code that opens an Arduino's serial port must wait out the
bootloader. The failure is intermittent, which is what made it look like a
plugin bug rather than a startup race.

### Problem 18 — a calibration measurement that could not be true
**Symptom:** the straight-line test reported 1.0208 m of odometry, and the tape
reading came back as "1.20-1.21 m, similar, wheel radius is fine". Those two
statements contradict each other: 1.20 vs 1.02 is an 18% error, not "similar".
**Resolution without re-measuring:** the straight and spin tests over-constrain
the geometry, because both describe the same robot. Working the numbers through:

| assumed real straight | implied radius | implied separation | outer-face span |
|---|---|---|---|
| 1.205 m | 59.0 mm | 315.1 mm | **341 mm — impossible** |
| 1.025 m | 50.2 mm | 268.0 mm | 294 mm — consistent |

The wheels are 300 mm outer-face to outer-face by tape, so they cannot be 341 mm
apart. Only the ~1.02 m reading describes a robot that can physically exist. The
user then confirmed 1.025 m.
**Lesson:** when two independent measurements constrain the same geometry, check
them against each other before accepting either. It caught a bad number without
a second trip to the robot, and an 18% radius error would have corrupted every
map built afterwards.

### Problem 19 — two bugs in the calibration tool itself
Both surfaced from one run whose start pose read `yaw=+13.73 deg`.

**(a) "Lateral drift" was measured in the wrong frame.** A 1 m straight run
reported 242 mm of lateral drift, which looks like a badly misaligned robot. It
was not: the run started at a heading of 13.73 deg (left over from the previous
spin), and `1.0227 * sin(13.73 deg) = 0.2427 m`. The robot had driven dead
straight — heading changed only 0.10 deg — but the tool reported raw odom-frame
`y` as if the robot always started pointing along +x.
**Fix:** resolve displacement into the START frame and report forward/lateral
separately.

**(b) The tool overshot every move, and it would have ruined UMBmark.** That
13.73 deg start yaw *was* the leftover from `--rotate 360`: the loop stops when
odometry reaches the target and then the robot keeps going. Pure deceleration at
the configured 1.5 rad/s^2 only explains 3.1 deg; the rest is ~470 ms of command
latency, because a stop must cross the USB link and then two ~37 ms Modbus writes
(control, then speed) queued behind whatever transaction is already in flight.
Over the four turns of a UMBmark square that is **~55 deg of accumulated error** —
the measurement would have been worthless, and worse, it would have looked like a
calibration result.
**Fix:** ease down to a creep speed on approach instead of running full speed into
the target, and report the coast explicitly so it is never mistaken for a
calibration error. Measured: **13.73 deg -> 0.74 deg per turn, 18x better.**
**Lesson:** calibrate the instrument before trusting it. Both bugs produced
plausible-looking numbers that pointed at the robot rather than the tool.

### Problem 20 — the D435i IMU did not exist, for two stacked reasons
**Symptom:** `rs-enumerate-devices` reported `Imu Type: BMI085` but offered zero
accel/gyro streams. Colour and depth were perfect.

**Blocker 1 - permissions.** The IMU sits behind the camera's HID interface,
which libusb must claim, and that needs WRITE access to the raw USB node.
`/dev/bus/usb/002/003` shipped as `crw-rw-r-- root:root`. Colour and depth were
unaffected because they are `/dev/video*` nodes owned by group `video`, which the
user is in - so the failure looked selective and confusing. Fixed by the official
RealSense udev rules, which `dpkg -L` confirms the ROS `librealsense2` package
does not ship. Vendored to `scripts/udev/`, installed by
`scripts/01_install_realsense_udev.sh`.

**Blocker 2 - the wrong backend.** Fixing permissions was necessary but not
sufficient, and changed nothing visible. The packaged librealsense is compiled
with the **V4L2 backend** (126 v4l2 symbols vs 20 libusb), which reads the IMU
from the kernel IIO subsystem. IIO entries for a USB HID sensor come from
`hid-sensor-accel-3d` / `hid-sensor-gyro-3d`, and **those modules are not built
for this JetPack kernel** - so that build has no code path to the IMU on this
machine at all. Fixed by building the *same version* (2.58.2) with
`FORCE_RSUSB_BACKEND=ON` to a user-writable prefix
(`scripts/02_build_librealsense_rsusb.sh`). Same soname, so selecting it is just
`LD_LIBRARY_PATH`; `realsense2_camera` needs no rebuild and the packaged library
stays as a fallback. Fully reversible.

**Result:** `Accel @ 400/200/100 Hz`, `Gyro @ 400/200 Hz`, and the node verifiably
loads the RSUSB library.
**Lesson:** two independent blockers stacked behind one symptom. Fixing the first
changed nothing visible, which is exactly when it is tempting to assume the fix
was wrong and revert it.

### Problem 21 — two measurement tools that lied
**(a) The camera cost script.** Consecutive runs contended for the device - the
previous node had not released it - producing a plausible **4.8 fps / 45.7% CPU**
result that was really 448,319 `Device or resource busy` retries. The script now
kills and *verifies* the device is free before starting, and prints
`RESULT INVALID` if it ever sees `resource busy` again.

**(b) `ros2 topic hz` reports "no data"** for the IMU topics even with
`--qos-reliability best_effort`, while `ros2 topic echo` works fine. A direct
rclpy subscriber shows they publish at 199.5 / 199.5 / 100.5 Hz. Do not trust
`topic hz` on these; count messages directly.

**Also:** `pkill -f realsense2_camera_node` kills its own shell, because the
pattern appears in the invoking command line. Use `realsense2[_]camera_node`.

### Problem 22 — the camera mount was 4.4 deg off the CAD figure
**Symptom:** none visible. The URDF said 15 deg down, taken from CAD, and nothing
had ever checked it.
**Method:** the floor is a better instrument than a tape up a mast. base_footprint
is *defined* on the ground plane, so if the URDF is right, depth points on the
floor must come back flat and at z = 0. Any mount error tilts or shifts that
plane by exactly the amount the mount is wrong
(`scripts/../labrobot_bringup/scripts/check_ground_plane.py`).
**Result:** the fitted floor sloped down with distance by 4.4 deg, plus 1.06 deg
of roll that CAD did not have at all. At 2 m that displaces points by 154 mm -
the floor would have mapped as a ramp.
**Ruling out the alternative:** a sloping floor produces an identical signature.
Repeating the measurement with the robot rotated 180 deg settled it - a mount
error is fixed in the ROBOT frame and keeps its sign, a floor slope is fixed in
the WORLD frame and flips. It did not flip. The same decomposition (mean =
robot-fixed, half-difference = world-fixed, exactly as UMBmark does it) put the
floor level to 0.25 deg.
**Fix:** `camera_pitch` 15.00 -> 10.60 deg, `camera_roll` 0 -> -1.06 deg.
**VERIFIED**: after applying the correction the plane tilt fell from 4.83 deg to
**0.29 deg**, and the displacement at 2 m from 168 mm to 10 mm. The residual
0.29 deg matches the floor's own levelness (0.25 deg, measured independently by
the 180 deg flip), so this is now floor-limited - tuning further would be fitting
the floor rather than the robot.
**Also measured, for free:** depth RMS about the fitted plane is 10.8-11.9 mm,
which is normal at these ranges. The factory depth calibration is fine and does
NOT need redoing.

### Problem 23b — the replacement cable was USB 2 only

A cable swap intended to fix the instability made things worse in a different
way. The camera enumerated on **Bus 001 (USB 2 root) at 480 Mbps** instead of
Bus 002 at 5000 Mbps.

| | old cable | replacement |
|---|---|---|
| link | USB 3.2, 5000 Mbps | **USB 2.10, 480 Mbps** |
| depth rate | 30 fps | **11.5 fps** |
| colour rate | 30 fps | **22.6 fps** |
| re-enumerations | ~7 per session | 0 |
| `control_transfer` errors | 8-13 per startup | 112 in one log |

The camera sits in the same physical hub - a Realtek hub presents as two devices,
a USB 2 one on Bus 001 and a USB 3 one on Bus 002 - so this is the **cable**
lacking working SuperSpeed pairs, not a different port. Most USB-C cables carry
only the USB 2 conductors; they charge and pass data perfectly while being
physically incapable of SuperSpeed, and the link falls back **silently** rather
than failing.

This is a nasty failure mode precisely because nothing errors. Downstream it
would present as a slow SLAM pipeline, and RTAB-Map at 11.5 fps depth would be
blamed long before the cable was.

**Tooling added so this is never diagnosed by hand again:**
* `scripts/check_camera_usb.sh` - one second, reports the negotiated link speed
  and what to do about it. Run after any cable or port change.
* `scripts/soak_test_camera.sh [total_minutes]` - the rigorous check. See
  Problem 23c for what it measures and why it was rebuilt.

**Resolved** by a third cable that is both USB 3 and electrically sound.

### Problem 23 — D435i USB link is unstable (RESOLVED — it was the cable)
**Symptom:** `control_transfer returned error ... Resource temporarily
unavailable` in bursts, and the camera **re-enumerated from Bus 002 Device 003
to Device 010** over one session - it disconnected and reconnected roughly seven
times. Twice the node stayed alive but stopped publishing TF, so a dependent
tool failed with "camera_depth_optical_frame does not exist" while the node
looked healthy in `pgrep`.
**Cause:** the cable. Three were tried: the original (USB 3 pairs present but
marginal signal integrity - see the symptoms above), a replacement that had no
SuperSpeed pairs at all (Problem 23b), and finally one that is both.
**Proof of fix**, `scripts/soak_test_camera.sh 8`, 2026-08-03:

| | old cable | USB-2 cable | current cable |
|---|---|---|---|
| link | 5000 Mbps, unstable | 480 Mbps | **5000 Mbps (USB 3.2)** |
| depth frames dropped | not measured | n/a | **0 of 7135 (0.00%)** |
| colour frames dropped | not measured | n/a | **0 of 7135 (0.00%)** |
| re-enumerations | ~7 per session | 0 | **0** |
| kernel USB resets | present | 0 | **0** |
| `control_transfer` errors | 8-13 per startup | 112 in one log | **5 in 8 min** |
| TF losses | 2 observed | - | **0 of 23 checks** |

Four minutes of continuous 848x480x30 depth + 640x480x30 colour with **not one
dropped frame**. M3 is unblocked.

### Problem 23c — the soak test was measuring the wrong thing, three times over

Worth recording in full, because the failure mode is subtle and general: a test
that is wrong in the *conservative* direction still costs real time, and it burns
it on hardware that was never broken.

**1. Two `set -e` bugs meant the USB-2 guard never ran.** The link speed was read
with `SPEED=$(for d in ...; do ...; done | head -1)`. Under `set -eo pipefail`
that takes the exit status of the *last* loop iteration - never the RealSense -
so the assignment "failed" and killed the script one line before the guard could
fire. The abort message never printed, and the resulting exit 1 looked exactly
like the guard working. A second instance of the same class sat in
`DEV0=$(lsusb | grep ... | head -1)`, where `head` closing the pipe early gives
the upstream `grep` a SIGPIPE that `pipefail` promotes to a script abort.
*Lesson: under `set -e`, verify a guard fires by forcing both branches, not by
observing an exit code that several unrelated faults also produce.*

**2. The harness measured its own scheduling jitter.** The soak loop called
`lsusb` - a subprocess, tens of milliseconds - on the same thread that drained
the topics, and single-stepped `rclpy.spin_once`. It reported IMU "stalls" with a
worst gap of 27.6 ms against a 25 ms threshold: pure self-inflicted latency.
Fixed by reading the device number from sysfs (one cheap file read) and spinning
the executor on a dedicated thread.

**3. The real error: judging a cable by a ROS topic rate.** Even after the fixes
the test failed on `depth 25.9 of 30 Hz`. That number measured **25.9, 29.2 and
29.5 fps across three consecutive runs on identical hardware** - it is a property
of the ROS node, DDS and Jetson scheduling, which sit three layers above the
link. Asked directly through librealsense, the same cable was delivering
**29.97 fps with zero dropped frames**.

**The restructure.** `scripts/soak_test_camera.sh` now splits its time:
* **Phase 1 - link health**, via `tools/rs_link_probe.cpp` (built on demand to
  `build/rs_link_probe`). Talks to librealsense directly, no ROS, and counts
  drops from the **frame number** - a frame the link lost leaves a hole in the
  sequence that no host CPU load can fake or hide. **This phase decides
  PASS/FAIL.**
* **Phase 2 - pipeline health**, the ROS topics the robot actually consumes. A
  rate shortfall here **warns** rather than fails, and says explicitly that it is
  a host problem, not a cable one.

Re-enumerations (sysfs `devnum` polling) and kernel USB resets (`dmesg`) are
counted across both phases; those *are* link faults and do fail the run.

**Byproduct worth keeping:** `align_depth` costs **80.6% of one core versus
29.8% without** - 13.4% of the machine's six cores. Affordable, and now measured
rather than assumed. Note `depth_align` (99% of 30 Hz) consistently outruns the
unaligned `depth_raw` (98%) and `colour` (92%), and RTAB-Map consumes the aligned
topic - so the stream that matters most is the healthiest one.

### Floor capture — full sensor characterisation

`labrobot_bringup/scripts/analyse_floor.py`, 10 frames averaged.

**Mount pose is now essentially exact** (residuals, so zero is correct):

| | |
|---|---|
| floor height | −2.4 ± 1.2 mm |
| pitch error | −0.005 ± 0.056° |
| roll error | −0.045 ± 0.051° |

**Camera height, measured independently of the URDF.** The plane's perpendicular
distance from the camera origin *is* the height: **846.7 mm measured vs 844.4 mm
in the URDF — 2.3 mm apart.** `camera_z` came from CAD and had never been checked
against anything until now.

**Depth noise vs range** — the number that decides how far to trust depth:

| range | RMS |
|---|---|
| 0.4–1.0 m | 7.4 mm |
| 1.0–1.5 m | 10.2 mm |
| 1.5–2.0 m | 14.3 mm |
| 2.0–3.0 m | 23.8 mm |
| 3.0–4.0 m | 28.2 mm |
| 4.0–6.0 m | 34.6 mm |

Fitted exponent **0.84**, against the textbook stereo value of 2.0 — this unit
does better than worst case at short range. Practical reading: trust depth under
~2 m, treat beyond 3 m as a hint. Feeds directly into the RTAB-Map and costmap
range limits.

### BLIND ZONE — the number Nav2 must be told

**The floor is invisible closer than 1.00 m from `base_footprint`.** The robot's
front edge is at 0.21 m, so there is **0.79 m of ground ahead of the bumper the
camera cannot see** — 3.0 s of travel at full speed.

This is geometry, not a data problem: height 847 mm with a 10.78° tilt and a ~58°
depth VFOV puts the bottom of view 39.8° below horizontal, predicting first floor
visibility at 1.02 m. Measured 1.00 m.

Tilt trade-off, if it is ever revisited:

| tilt | blind ahead of bumper | reaction time | top of view |
|---|---|---|---|
| **10.78° (as built)** | **0.79 m** | **3.1 s** | −18.2° |
| 15° (design) | 0.67 m | 2.6 s | −14.0° |
| 20° | 0.53 m | 2.0 s | −9.0° |
| 30° | 0.30 m | 1.1 s | +1.0° |

Negative "top of view" means the camera still sees above the horizon, which is
where walls, door frames and ceiling edges give visual odometry its stable
features. Floor-only views are texture-poor and RTAB-Map struggles on them.

**Recommendation: leave the tilt as built and handle the blind zone in Nav2.** A
camera-only robot needs high features far more than it needs to see its own
bumper, and obstacle persistence in the costmap is exactly the mechanism for
remembering what was seen a second ago. Concretely, in M4.T2: do not clear the
costmap aggressively, and keep the footprint inflation larger than 0.79 m of
travel at planning speed.

### M3.T1 — visual odometry validated, and what each sensor is actually for

`ros2 launch labrobot_bringup visual_odometry.launch.py`, deliberately with
`publish_tf: false` and **no motion prior from the EKF** - seeding RTAB-Map from
the EKF and then comparing the two would be circular.

**Health while running:** quality 202-231 inliers, std dev 7-12 mm / 0.020-0.024
rad, update time 82-92 ms, zero tracking losses.

**Over a 360 deg in-place spin:**

| | rotation | translation (should be zero) |
|---|---|---|
| EKF (wheels + gyro) | 360.52 deg | **0.2 mm** |
| visual odometry | 360.04 deg | **70.8 mm** (max excursion 105 mm) |

So the two disagree about **rotation by 0.13%** and about **translation by 70 mm**.
That is not a fault in either - it is each sensor's known character, and it
settles the architecture:

| quantity | best source | why |
|---|---|---|
| translation | **wheels** | 0.2 mm over a full spin; agreed with the EKF to 0.1 mm over 1 m |
| rotation | **gyro** | wheels lose 4-12% to castor slip (Problem 25); VO confirms the gyro to 0.13% |
| loop closure / absolute correction | **camera** | the only sensor that can recognise a place it has been before |

Visual odometry adds nothing to *odometry* here: the EKF already beats it on
translation by two orders of magnitude and matches it on rotation. Its value in
M3.T2 is loop closure and map correction, which is what RTAB-Map is actually for.
Raw VO translation drifting ~70 mm per revolution is normal - during pure
rotation the baseline is tiny, so small per-frame errors integrate freely.

### M3.T2 — mapping and loop closure (step 1 complete)

`ros2 launch labrobot_bringup mapping.launch.py`, then
`ros2 run labrobot_bringup check_map.py`.

**First map, from a single 360 deg in-place spin at 0.2 rad/s** - chosen because
it needs no floor space at all, and it is the cleanest possible loop-closure test:
the end of the rotation must recognise the start of it.

| | |
|---|---|
| grid | 152 x 180 @ 5 cm = 7.6 x 9.0 m |
| mapped free space | 11.62 m2 |
| loop closure | node **1**, hypothesis **0.956** (accept threshold 0.11) |
| graph | 38 nodes, 29 in working memory |
| **map->odom correction** | **32.3 mm, -1.37 deg** |

That last row is the one that matters: loop closure had to move the map only
32 mm to make it self-consistent, i.e. the EKF odometry was already nearly right
over a full revolution. Node creation is capped by `Rtabmap/DetectionRate: 1`
(1 Hz), so a 31 s spin yields ~31 nodes, one every 11.5 deg.

**Step 2, a 1.2 x 0.8 m rectangular loop** (sweeps 1.78 x 1.38 m, fits a
2 x 1.5 m lab space), then a re-observation spin:

| | after spin | after loop | after re-observation |
|---|---|---|---|
| mapped free space | 11.62 m2 | 14.14 m2 | **15.27 m2** |
| graph nodes | 38 | 130 | **164** |
| map->odom correction | 32.3 mm | 104.5 mm | 142.1 mm, -3.49 deg |
| loop closures | 1 | - | **11 in 45 s** (+13 proximity) |

Loop closures land on nodes 643-678 with best hypothesis 0.890 - the robot is
reliably recognising places it has already mapped. **M3.T2 is functionally
complete**: occupancy grid for Nav2, loop closure, and a correction that stays
small enough to be believable.

**Three traps that made a working system look broken:**

* **Loop closures must be COUNTED, not sampled.** `loop_closure_id` on `/info` is
  non-zero only on the single iteration a closure is accepted. Reading it once
  returns 0 almost every time, which reads as "loop closure never works" on a
  system firing 11 closures in 45 s. `check_map.py` counts over a window.

* **The occupancy grid is only built while something SUBSCRIBES to /map.** A tool
  that connects, reads one message and disconnects receives the TRANSIENT_LOCAL
  latched grid from *before* it arrived. A map that had grown from 1832 to 4646
  free cells during the spin read as byte-identical before and after. The
  giveaway was `pub=0.0000s` in RTAB-Map's own log line. `check_map.py` holds the
  subscription open.
* **Loop closure is not written to the node's log.** Grepping the log for "loop
  closure" returns nothing and looks like total failure, while `/info` was
  reporting `loopClosureId=1` at hypothesis 0.956 the entire time. Ask the topic,
  not the log.

**Open mechanical concern.** On the loop's fourth leg the robot drifted **-8.92
deg and 95 mm over 0.8 m**, against 0.27 / -1.52 / -1.27 deg on the first three.
Same castor catch-and-release as Problem 25, but the worst instance yet, and it
happened on the leg following the most direction changes. This is worth a
physical look before Nav2 - a 9 deg veer per metre is more than path tracking
should have to fight.

### M4.T1/T2 — Nav2 on a lidar-free robot

```bash
ros2 launch labrobot_bringup robot.launch.py localization:=true pointcloud:=true
ros2 launch labrobot_bringup navigation.launch.py
```

Config in `config/nav2_params.yaml`, launch in `launch/navigation.launch.py`.
Verified active, with all costmaps populated from camera data alone:

| | size | lethal | inflated | free |
|---|---|---|---|---|
| `/map` (RTAB-Map) | 172x190 | 7296 | - | 6107 |
| `/global_costmap` | 172x190 | 14897 | 1643 | 1458 |
| `/local_costmap` | 60x60 (3x3 m) | 837 | 416 | 2347 |

**Obstacles come from the depth point cloud, not a laser scan.**
`depthimage_to_laserscan` was rejected on geometry: the camera is pitched
**10.78 deg down**, so a "horizontal" scan band is not horizontal in the world.
Its optical axis meets the floor at 0.844/tan(10.78 deg) = **4.43 m** and rows
below centre hit nearer, so the floor would be reported as an obstacle ring.
Instead the cloud is transformed through TF into `base_footprint` and filtered by
height (`min_obstacle_height: 0.06`), which is correct precisely because the
mount pose was calibrated in M2.T1.

**AMCL is not used.** It needs a LaserScan this robot does not have, and RTAB-Map
already publishes `map -> odom` from visual place recognition.

**THE BLIND-ZONE RULE (M4.T2).** Both costmap layers set
**`raytrace_min_range: 0.80`**. The robot cannot see anything closer than 0.79 m
(measured, M2.T1), so inside that radius an absence of returns is *not* evidence
of free space - it is evidence of being too close to look. At the default 0.0 the
robot would confidently erase any obstacle it had just driven up to. This single
parameter is the difference between a camera-only robot that stops for a bench
leg and one that drives into it.

**Point cloud cost.** `decimation:=3` in the driver gives 43k points at 16 Hz and
10.5 MB/s - affordable. Crucially the decimation applies only to the cloud: the
published depth images stay 848x480 / 640x480, so mapping quality is untouched.

**Jetson-specific trap.** The point cloud parameter is **`pointcloud__neon_.enable`**,
not `pointcloud.enable`. realsense2_camera derives the name from the librealsense
FILTER name, and on ARM that filter is "Pointcloud (NEON)". The node accepts the
x86 name silently ("Parameter not set") and simply never publishes a cloud, which
looks exactly like the driver ignoring you. `camera.launch.py` sets both names.

### M4.T3 — autonomous point-to-point navigation WORKS

`ros2 run labrobot_bringup nav_goal.py --forward 1.0` sends a goal RELATIVE to
the robot's current pose (computed in the map frame, but expressed as "1 m
forward" so it can be checked against the room before driving).

| | goal 1: straight | goal 2: turn-around + drive back |
|---|---|---|
| result | **SUCCEEDED** | **SUCCEEDED** |
| moved | 0.987 / 1.000 m | 0.901 / 0.950 m |
| position error | 80 mm | 67 mm |
| heading error | +9.4 deg | -9.4 deg |
| time | 11.6 s | 46.1 s |

**The +/-9.4 deg heading error is the TOLERANCE, not a control failure.**
`yaw_goal_tolerance` is 0.17 rad = 9.74 deg and the controller stops the instant
it is inside. Landing at 9.4 deg twice with opposite signs is what "stop when
good enough" looks like. Tighten the tolerance if it matters - but that means
more time fighting the castors for an angle nothing downstream needs.

**Speeds were reduced after the first run** from 0.26 m/s / 1.0 rad/s to
**0.15 m/s / 0.6 rad/s**, and the acceleration limits with them. The reasoning is
in `nav2_params.yaml`: the camera cannot see within 0.79 m so speed buys no extra
warning, the castors slip worse the harder they are asked to change direction,
and this is a 3S pack. `velocity_smoother` enforces the same ceiling
independently of the controller, so a mis-tuned controller cannot exceed it.

**Open tuning item (M4.T4).** Goal 2 took 46 s for ~11 s of actual motion. The
`distance remaining` trace plateaued at 0.21 m for 14 s and 0.17 m for 16 s - the
final approach crawling, most likely `RotateToGoal.slowing_factor: 5.0` plus
castor stiction at very low speed.

### M4.T5b — narrow gap PASSED: 72.8 cm gap, 16 cm/side (2026-10-07)

Two objects 72 cm apart (user's tape; live depth 72.8 cm). Gap measured with
`tools/gap_edges.py` (10 stacked depth frames, 2nd/98th-percentile edges),
route checked first with `tools/plan_probe.py` (planner only, no motion), then
`nav_goal.py --forward 2.4 --left -0.10` with RPP, point cloud on.

Result: **SUCCEEDED**, 2.31 m in 44 s, 98 mm from goal. From the recorded path
(`docs/logs/gap72_path.json`) against the measured edges: **minimum clearance
8.9 cm** (right), 18-30 cm left; robot 4-7 cm right of the gap centre.

Two things to improve before the 61 cm (10 cm/side) gap:
1. **Off-centre by ~6 cm, by design.** With `inflation_radius 0.26` a 72.8 cm
   gap has a ~21 cm cost-free strip down its middle; NavFn is indifferent
   anywhere inside it, so it does not centre. In a 61 cm gap that 6 cm offset
   would leave ~4 cm on one side. Fix: a cost gradient that reaches the centre
   of the gap (larger inflation radius, steeper scaling) so the centre is the
   unique minimum - the standard Nav2 aisle-centring setup.
2. **False start.** 3.3 s in (~15 cm, far from the gap) RPP reported
   "collision ahead", patience expired, the recovery cleared the local costmap,
   and the rest of the run was clean. A stale mark near the start; not yet
   identified.

### M5 — waypoint navigation (2026-10-14, in progress)

- `waypoint_follower` added to navigation.launch.py (FollowWaypoints action,
  `stop_on_failure: false`, 2 s wait at each waypoint).
- `scripts/waypoints.py`: `save` (current pose) / `pick` (pose clicked in
  Foxglove on /waypoint_pick) / `add NAME X Y YAW` / `list` / `delete` /
  `go A B C [--return] [--loop N]`. Stored in `docs/maps/waypoints.yaml`.
- `scripts/nav_gate.py`: the shared pre-motion gate (auto_localise verdict +
  current-view check) used by nav_goal.py and waypoints.py.
- auto_localise: exits on SIGINT/SIGTERM (a failed run had lingered, ignoring
  SIGINT, and its stale latched "localised: false" competed with the new
  run's "true"); bringup_nav.sh kills any leftover instance first.

**Problem 42 — switching to mapping mode started a NEW map.** First mission
(wp1 -> wp2 -> wp3 -> start): "SUCCEEDED, 0/4 reached", every leg failed.
After `set_mode_mapping`, map->odom reset from (5.65, 0.70, 90 deg) to
(-0.13, -0.14, -6 deg): RTAB-Map began a disconnected session with its own
origin, so the waypoints (old-map frame) were in the wrong place. It joins the
old map only on a later strong visual closure (got 16/20, 13/20). That session
was discarded and the clean map restored.
Fix (built, NOT yet tested): `join_saved_map()` - after switching, add a user
loop closure (`/rtabmap/add_link`, type kUserClosure) from the new session's
first node to the nearest saved node using the known pose, then verify the
robot's map pose did not move (<= 15 cm / 5 deg); on failure return to
localisation mode (frozen but correct map).

Session ended on low battery. The last bringup's rtabmap died with "could not
create service: rcl node's context is invalid" - confirmed harmless: it died
at 16:46:41, the moment the stop was sent while it was still loading.

### Problem 43 — git repository corrupted by the power cut (2026-10-15)

A commit made at 16:47, a minute before the battery died, left 22 zero-length
object files (the commit, its trees and blobs) and `main` pointing at the empty
commit: every git command failed with "loose object ... is corrupt". Files had
been written but not flushed to disk when power went.
Recovery: backed up `.git` (`~/git_backup_20261015_1631`), confirmed the
previous commit 9834eb6 intact, pointed `main` back at it, deleted the empty
objects, cleaned the truncated reflog line, rebuilt the index from HEAD and
re-staged from the intact working tree; `git fsck --full` clean. Nothing lost.
Prevention: `git config core.fsyncObjectFiles true` (git 2.34) so each commit
reaches the disk before git reports success; `sync` after commits. On a
battery robot, also stop the stack and run `sync` before switching off.

### Narrow gaps — accepted by the user (2026-10-14)

72 cm (16 cm/side) passed with 8.9 cm minimum clearance (M4.T5b); the 61 cm
(10 cm/side) gap was passed (M4.T5d/e) and the user has accepted this
milestone - "leave it for now". Known residuals, parked: ~1 cm corner margins
at 10 cm/side, 3-7 cm right-of-route tracking (planner bias fixed by Smac 2D;
controller corner-cutting vs replanning not yet separated - instrumentation
in place: gap_edges --save, record_path plans history, gap_run_report).

### M5 — waypoint navigation WORKS (2026-10-29)

Full chain on map v6: cold start -> auto_localise (spot 1: geometric distinct
72%, 2/2 visual closures agree, single-view 100%) -> joined the saved map
(link 1341 -> 528, pose held within 9 cm) -> lifelong mapping -> user-defined
waypoints -> mission.

- Waypoints were clicked by the user in Foxglove (`waypoints.py pick`,
  topic /waypoint_pick). Foxglove published them in `base_link` (its display
  frame); the first version rejected such clicks, losing the user's first
  waypoint. Fixed: any frame is converted to map through TF.
- `p2` was 16 cm from an obstacle (robot half-width 0.20 m): flagged before
  driving, user chose the nearest spot with 0.35 m clearance (76 cm away).
- **Mission p1 -> p2: SUCCEEDED, 2/2 waypoints reached in 228 s, final pose 7 cm
  from p2** (goal tolerance 10 cm). Compare the first mission (2026-10-14): map
  frame reset on switching to mapping, every leg failed.

The track recording for this run was lost: a cleanup command's process filter
matched its own command line again and interrupted the recorder before it
wrote its file. Lesson repeated: stop processes by PID, never by a pattern the
calling command also contains.

### Problem 45 — power cut corrupted the map database; RTAB-Map's SQLite defaults (2026-10-29)

After a lifelong-mapping session on 10-27 lost power, `~/.ros/rtabmap.db`
(grown 400 -> 598 MB) read "database disk image is malformed". Restored
`lab_v6_clean_20261027.db` (only minutes of updates lost; corrupt copy kept).
Cause: RTAB-Map's defaults `DbSqlite3/JournalMode 3` (MEMORY) and
`DbSqlite3/Synchronous 0` (OFF) - journal in RAM, nothing forced to disk. SQLite
documents that this combination will very likely corrupt the database on power
loss mid-transaction. Fix in mapping.launch.py: JournalMode 1 (TRUNCATE,
journal on disk, interrupted transactions roll back on next open) and
Synchronous 2 (FULL, each commit on disk before it counts). Verified applied
at startup ("Setting RTAB-Map parameter DbSqlite3/JournalMode=1 ...").
Related: waypoints.py could not send its own cancel on Ctrl-C (the default rclpy
signal handler shut the context first) - now handled like auto_localise and
confirms the cancel or warns.

### Map v6 — fresh map (2026-10-27)

Fresh teleop map after v4 went stale (19 days, room changed; Problem 44 test).
`Kp/MaxFeatures` raised 500 -> 1000 (fixed at node creation; more words per
place for relocalisation). An earlier attempt that day (v5, 1110 nodes) lost
power before a clean save; integrity ok, kept as
`docs/maps/backup/lab_v5_partial_20261027_1431.db`.
v6: 1 session, 34.2 m, 1271 poses, 2112 links, 710 nodes after optimisation,
188 loop closures + 329 proximity, final drift correction 1.69 m, largest
single jump 441 mm, no suspect jumps. Clean save, integrity ok, 406 MB, backed
up to `docs/maps/backup/lab_v6_20261027.db`.
**Cleanup (user request): remove a person mapped as an obstacle.**
1. Trimmed the last 20 s: `rtabmap-reprocess -stop 1251` -> 1251 nodes, 344 loop
   closures, integrity ok. But the person was STILL in the map - they had
   been seen earlier too.
2. Located the blob (isolated, 96% free around it): (0.15, -0.12), 0.45 x 0.40 m,
   in the bottom-middle corridor - `docs/images/v6_person_blob.png`, confirmed
   by the user's description.
3. RTAB-Map assembles the 2D map from each node's own obstacle list, so the
   person was removed AT THE SOURCE: 613 points in 18 nodes deleted from
   Data.obstacle_cells inside the box + 10 cm, stored grid cleared and rebuilt.
   Result: 70 occupied cells in the box -> 0, **0 cells changed outside it**.
   Editing only the assembled grid would have been undone by the next rebuild.
Active map: `docs/maps/backup/lab_v6_clean_20261027.db` (= ~/.ros/rtabmap.db).
Originals kept: `lab_v6_20261027.db`, `lab_v6_trimmed.db`.
Tool for next time: `tools/erase_from_map.py --box X0 X1 Y0 Y1 [--dry-run]`
(stack stopped; backs up the DB first). Dry-run on the pre-edit map finds the
same 613 points / 18 nodes; on the cleaned map, 0.
Old v4 waypoints archived to `docs/maps/old_v4/` (different frame); re-save
waypoints on v6.

### Problem 44 — auto_localise accepted a WRONG pose (2026-10-15)

Cold start: spot 1 geometric (2.92, -0.25) ambiguous 82%, moved 0.6 m, spot 2
(2.27, -0.30, 182 deg) distinct 76%, ring score 0.53; no RTAB-Map visual
closure; single-view check WRONG (9% near obstacles) - but it no longer gated
(Problem/M3.T6 change after a false alarm). Declared LOCALISED, joined the saved
map (user link) and switched to mapping. **The user saw the pose was wrong.**
Stopped at once; clean map restored, wrongly-linked DB kept as backup.
The two geometric estimates were mutually consistent (the odometry move
predicts spot 2 within 5 cm), so this was not noise: the matcher consistently
found a different, same-shaped part of the lab.

Fix - acceptance requires TWO INDEPENDENT METHODS to agree: a distinct,
well-fitting geometric match AND at least one RTAB-Map global (appearance)
closure implying the same map->odom correction. Otherwise move to another
viewpoint and look again (now up to 4), then report NOT localised. Corrections
are compared rather than robot poses (the old gate compared a closure's robot
pose from an earlier viewpoint with the current one). Every attempt's ring and
verdict is now saved to docs/logs/autoloc/ for post-mortems - this failure
could not be analysed because only the final picture was kept.
Evidence base: both user-verified localisations had agreeing visual closures;
the wrong one had none.
**Hardware test 2026-10-26 (same area as the wrong pose):** 4 viewpoints,
geometric best (2.02,-0.35) / (5.92,-4.15) / (3.92,-0.15) / (1.52,-0.45),
runner-up 95-100% every time, ring score 0.37-0.46, ZERO visual closures ->
**NOT localised** - the correct refusal where the old rule had given a
confident wrong answer. Neither shape nor appearance recognises this area: the
map is 19 days old (v4, 10-07) and lifelong updating never ran (Problems 42,
44). Remedy: fresh map, then lifelong SLAM via the now-verified map join.
**Lesson:** a gate removed after one false alarm let through the next real
error. Weigh a check's false alarms against what it catches before removing it.

### M3.T6 — auto_localise uses geometry: kidnap tests (2026-10-14)

`auto_localise.py` now: 360 deg turn -> geometric match (global_localise) ->
if ambiguous, turn to the most open direction (longest robot-width corridor
in the ring) and drive 0.6 m forward (Nav2 DriveOnHeading, collision-checked),
up to 3 viewpoints -> whole-room gate -> /initialpose to RTAB-Map (must adopt
within 0.3 m / 10 deg) -> mapping mode. Whole-room gate: ring score >= 0.35 and
any RTAB-Map visual closures made during the turn must AGREE (0.3 m / 10 deg),
a disagreement refuses. The single-view check is logged only: in kidnap 1 it
called WRONG (1% near obstacles) a pose that the geometric match and three
RTAB-Map closures agreed on within 12 cm - its view held a chair and a person,
the fixed structure beyond its 2.5 m range.

- **Kidnap 1:** geometric (3.52, -0.15) distinct 79%; RTAB-Map closures to 1051,
  1059, 1803 at (3.50, -0.03). Refused by the then single-view gate (fixed).
- **Kidnap 2:** spot 1 geometric (5.47, 0.30) ambiguous 93%, RTAB-Map closures
  1748-1750 at (5.46, 0.32); drove 0.6 m to the most open direction; spot 2
  turn timed out at 263 deg (robot turned at 0.045 rad/s vs 0.15 commanded).
  Allowance raised to 240 s, partial rings >= 240 deg accepted. Re-run: distinct
  (79%), (5.07, 0.50, 98 deg), ring score 0.40, single-view check OK (80%),
  **LOCALISED**, RTAB-Map switched to mapping. Picture:
  `docs/images/autoloc_kidnap2_match.png`.

**User confirmed (2026-10-14): the robot's position matches the map.**
Open: the ambiguity margin is thin (79% vs the 80% limit, twice); turn rate
varies strongly by spot (wheel slip).

### M3.T5 — geometric global localisation, no markers (2026-10-14)

Requirement (user): no fiducial markers; the robot may be kidnapped and must
find itself by rotating/moving, in a lab whose furniture moves.
`global_localise.py`: one slow 360 deg turn stitches depth (0.10-0.90 m band,
0.4-4 m) into a ring in the odom frame, reduced to a virtual 2D laser scan
(nearest return per 0.5 deg). Correlative scan matching (Olson 2009) over the
WHOLE map by FFT: likelihood field of obstacle SURFACES (occupied cells next
to free) minus a free-space penalty (ray samples at 30/55/80% landing on
mapped obstacles), robot restricted to mapped free space >= 15 cm from
obstacles. Confidence = distinctness: best vs best genuinely different place
(> 1 m or > 20 deg) must be < 80%.

Synthetic (rectangular room, 20% points missing, 25% clutter): 6/6 poses
correct (3.5 cm = half a cell, <= 1 deg), 4/6 distinct; the ambiguous ones
are real 90/180 deg room-symmetry aliases - the move-and-retry case.

Hardware, first clean run: 720 beams, search 7.1 s, **DISTINCT (runner-up 64%)**,
pose x=+1.87 y=-0.50 yaw -82 deg vs RTAB-Map's independent x=+1.89 y=-0.42
yaw -84 deg - **8 cm / 2 deg agreement** between geometry and appearance.
Picture: `docs/images/global_localise_first_match.png`.

Two versions failed first and were fixed: keeping every depth point smeared
walls into radial bands and scoring against solid furniture blobs made whole
regions match (score 0.197, ambiguous); the 60 s Spin allowance ran out at
273 deg (actual turn rate is below the commanded 0.15 rad/s). An earlier run
was also invalid because the robot was moved by hand mid-turn.

### M3.T4 — automatic startup localisation (2026-10-13/14, user requirement)

"Robot should spin on startup and localise itself automatically up to a
threshold, considering the dynamic environment." `auto_localise.py`, started by
`scripts/bringup_nav.sh` after Nav2:
- RTAB-Map starts in LOCALISATION mode (only the saved map competes - in SLAM
  mode the current session's own fresh views always won, hypothesis 0.93-0.98,
  so the saved map was never tested and the session stayed disconnected;
  that DB was discarded and the 19:35 backup restored).
- One slow continuous Nav2 Spin (0.15 rad/s, accel 0.15 rad/s^2), not 45 deg
  steps: the drive wheels slipped at each step start (user observation).
- Localised = >= 2 global closures to nodes that existed before this run,
  agreeing within 0.15 m / 5 deg, and the gross-error check not WRONG.
  `Vis/MinInliers` 15 while localising (the true place kept scoring 19/12/12
  vs wrong places 0-9), restored to 20 before anything maps; Ctrl-C-safe.
- Then `/rtabmap/set_mode_mapping` - lifelong SLAM, joined to the saved map.
- Publishes latched `/labrobot/localised` + `/localisation_status`;
  `nav_goal.py` refuses goals until true.

**Result on hardware so far: NOT localised.** Mechanics verified (720 deg
continuous turn, threshold restored to 20). From today's spot: 22 candidates,
20 at 0/15 inliers, best 10/15; two tentative "localization was good", never
confirmed. The week-old single map no longer matches the room well enough.

### Problem 41 — Spin refused "Collision Ahead" in open space: Iron parameter names on Humble (2026-10-14)

With Problem 40 fixed, every Spin aborted with "Collision Ahead" although the
nearest obstacle was 0.9 m away (a lone lethal cell 30 cm out was a red
herring: clearing it changed nothing). The behavior_server section used the
Iron+ names `local_frame: odom` / `global_frame: map`. Humble's
behavior_server declares only costmap_topic, footprint_topic, global_frame,
robot_base_frame, transform_tolerance, cycle_frequency (checked in
libbehavior_server_core.so) - so `local_frame` was ignored and the robot's
MAP-frame pose was checked against the ODOM-frame local costmap. Fix:
`global_frame: odom` plus explicit costmap/footprint topics. Verified: a 45 deg
Spin goal now SUCCEEDS.
**Lesson:** check parameter names against the installed distro, not examples
from newer docs - Nav2 ignores unknown parameters silently.

### Problem 40 — Nav2 recoveries never moved the robot (2026-10-13)

The behaviour server (Spin, Wait, DriveOnHeading) published to `/cmd_vel`,
which had **0 subscribers**: navigation.launch.py remapped the controller's
`cmd_vel` to `cmd_vel_nav` (into the velocity smoother) but not the behaviour
server's. Every recovery Spin in every run so far "failed" on its time
allowance with the robot motionless - earlier attributed to cramped space.
Fix: same remap on behavior_server. Verified: `/cmd_vel` no longer exists,
`/cmd_vel_nav` has both publishers, Spin steps now complete on hardware.

### Problem 39 — the pose-search check gave false WRONGs; replaced (2026-10-13)

The user could see in Foxglove that the pose was right; the search-based check
kept calling it 0.4-1.8 m wrong. Independent alignment at that pose: 83.8% of
9,817 live obstacle points on mapped obstacles, 100% within 15 cm, 0.0% in
mapped free space. The search found "better" poses because one view barely
constrains error along some directions; its self-test only ever proved that
wrong poses fail, never that the right one passes.

Replacement (`localisation_check.py`), current pose only, ~5 s instead of ~30 s:
WRONG if > 25% of the FARTHER half of the points land in mapped free space, or
< 60% of all points are within 10 cm of a mapped obstacle. Self-test now
includes the correct pose. Measured on the confirmed-correct view:

| offset | free_far | near_obs | verdict |
|---|---|---|---|
| none (correct) | 6.7 / 14 / 20% over 3 frames | 88-96% | OK |
| 15 cm left | 37% | 59% | WRONG |
| 15 cm right | 2% | 99% | **missed** |
| 0.15-0.3 m forward | 0-5% | 98% | **missed** |
| 5 / 12 deg | 39 / 33% | 61 / 54% | WRONG |
| 0.6 m back + 0.4 m left | 40% | 30% | WRONG |
| 1.8 m forward | 0% | 40% | WRONG |

It is a gross-error guard (hand moves, stale startup pose: 0.5-1.8 m), not a
precise check; small errors in directions the view does not constrain pass.
The correct pose's margin is ~5 points, so false refusals are possible - they
fail safe. `relocalise.py`'s /initialpose hint was removed with the search.

### Lifelong SLAM for navigation (2026-10-13, user decision)

"Update map in every navigation": `scripts/bringup_nav.sh` now starts
`robot.launch.py slam:=true` by default (`--localization` for a frozen map).
Moved furniture is re-mapped instead of lingering as phantom obstacles, and the
stored appearance keeps up with the room - the cause of Problem 38. Costs: DB
grows ~10 MB per metre; a false loop closure could bend the map (monitor_slam,
and back up `~/.ros/rtabmap.db` before long sessions).

### Problem 38 — relocalisation fails when the room has changed (2026-10-13)

The robot powered off mid-spin (pack discharged - nothing in the stack
monitors battery voltage; see Open risks). After restart, from an open spot:
two full 45-deg-step turns, RTAB-Map accepted **no** global closures.

- Spin 1 (default): 30 candidates, 23 at exactly 0/20 inliers, best 12/20.
- Spin 2 with `RGBD/LoopClosureReextractFeatures: true` (fresh features from
  the stored images instead of pairing by vocabulary word ID): 15
  candidates, 11 at 0/20, best **18/20** twice, one "localization was good".
  Kept on permanently in `mapping.launch.py`.

The user confirms objects have moved since mapping, and lighting differs
(evening vs afternoon). Mapping accepted 141 closures because it matched
against views minutes old. **Conclusion:** a single-session appearance map of a
changing lab does not relocalise reliably. Standard remedy: multi-session
mapping - extend the map in the current layout/light so RTAB-Map holds several
appearances of each place.

Also: `localisation_check` is not reliable at every heading (OK, then "1.1 m
wrong" after a pure 45 deg turn), and feeding its estimate to /initialpose
pushed the pose around. `relocalise.py --hint` is now opt-in, off by default.

### Problem 37 — relocalising after a hand move (2026-10-12)

**What goes wrong.** After the robot is carried (or the Jetson restarts with
the robot moved), localisation mode:
1. starts from the pose saved at the end of the last session ("Update map
   correction based on last localization saved in database") - a guess, here
   ~0.5 m out, which RTAB-Map reports as localised;
2. then REJECTS the true visual matches as a "teleport": its odometry cache
   (`RGBD/MaxOdomCacheSize`) and proximity links built at the stale pose
   contradict them (error ratio 5.28 / 3.76 vs `RGBD/OptimizeMaxError` 3.0).
And from a cluttered spot most candidate matches are on things that MOVE.
Stored images showed the most frequent candidate shared a castor pole stand,
chair and desk clutter with the live view; geometry correctly gave 0/20
inliers (35 rejections in 75 s). The far wall/shelf structure was barely in
view.

**Pre-goal check, reworked** (`localisation_check.py`). Moved furniture that
is IN the map fooled the all-points version into a false WRONG (near points
46% -> 93% at a pose 0.54 m away, far structure 73% vs 76%). The verdict now
comes from a pose search over the FARTHER half of the points only - moved
objects disagree up close, a wrong pose disagrees everywhere. Self-test 4/4
(1.8 m, 0.6/0.4 m, 12 deg, 0.3 m). A first attempt (far points scored only at
the all-points winner) went 2/4 and was discarded.

**`relocalise.py`** (new): check -> if wrong, publish the check's far-structure
estimate on `/initialpose` (replaces RTAB-Map's stale pose and clears its
odometry cache) -> optional `--spin` in 45 deg steps -> relocalised only when
RTAB-Map has accepted a GLOBAL loop closure AND the check agrees -> then clear
both costmaps (a hand move leaves stale marks in odom coordinates). Counting
proximity detections as confirmation was a bug: they are searched around the
current guess, so they cannot confirm it.

**Status:** the hint is taken (check goes WRONG -> OK after it), but from the
current cluttered spot RTAB-Map has not produced an accepted global closure in
two full turns, so `relocalise.py` correctly reports NOT relocalised.
Relocalise facing stable structure (walls, shelving), away from movable
furniture and people.

### M4.T5e — 61 cm gap with Smac 2D: route centred, robot still 3-7 cm right (2026-10-09)

Camera yaw re-measured: **-0.77 deg** (first run +0.57 deg). Opposite signs ->
no measurable mounting error (noise ~+-0.6 deg); URDF left unchanged.

Smac 2D's route, probed before the run, was **0.7-2.2 cm** from the measured
gap centre (NavFn: 4-7 cm right). Run: SUCCEEDED, 2.01 m, 38 s (vs 76.6 s),
one brief "collision ahead" stop. But the ROBOT ran 3-5 cm right of the gap
centre, 7 cm right of the probed route at the narrowest slice, heading -12 deg
on a -5 deg route; minimum corner clearance **1.1 cm** (NavFn run: 0.6 cm).

The free space opens to the right past the exit (right edge -0.47 -> -0.76 m).
Either the 1 Hz replans bent the route right towards it, or RPP (lookahead
clamped at its 0.30 m minimum by cost-regulated slowing) cut the corner. The
recorder only kept the last plan, so it could not tell. Instrumented:
`record_path.py` now keeps every replan (map + odom frame, timestamped),
`gap_edges.py --save` stores edges in odom, and `tools/gap_run_report.py`
splits each slice into route-off-centre (planner) and robot-off-route
(controller). Report tool verified on synthetic data.

### Problem 36 — the global planner had a built-in half-cell bias (2026-10-09)

All narrow-gap runs ran 4-7 cm right of centre. The camera yaw was measured
first (`tools/camera_yaw_check.py`: drive straight at a gap, compare its edges
from two distances in odom) at +0.57 deg (spread 1.07 deg over 8 slices) -
about 2 cm at 2 m, not enough alone, and to be re-measured before applying.

`tools/planner_bias/` runs the real planner_server alone on synthetic maps
(robot footprint, inflation 0.45/5.0), one opening per map, crossed both ways,
walls along both axes:

| planner | centre on a cell edge | centre mid-cell |
|---|---|---|
| NavFn | -2.4 cm | **-2.5 cm** |
| Smac 2D | -2.5 cm | **0.0 cm** |
| Theta* | -2.5 cm | **0.0 cm** |

NavFn's error has the same sign in MAP coordinates whichever way the robot
travels: a fixed half-cell shift towards -x/-y, even when a cell sits exactly
on the gap centre. Every hardware run headed ~+x, so it was always to the
robot's right. Smac 2D / Theta* are exact when they can be; the rest is 5 cm
grid rounding (0-2.5 cm). **Switched to `nav2_smac_planner/SmacPlanner2D`**
with `cost_travel_multiplier: 2.0`. Not yet verified on hardware.

Also: `labrobot_bringup/package.xml` now declares every package the launch
files start (Nav2, RTAB-Map, RealSense, foxglove, compressed transport); before,
a fresh `rosdep install` would have missed all of them.

### Version control

The workspace is a git repository (first commit 2026-10-09). `.gitignore`
excludes build/install/log, map databases and point-cloud exports (GB-scale;
backed up under `docs/maps/backup/` on the Jetson) and `docs/logs/`. Verified
changes are committed together with the report entry that documents them.

### M4.T5d — 61 cm gap (9.6 cm/side) re-run: through, but ~1 cm at the corners (2026-10-09)

Pre-goal localisation check passed (it had first refused - correctly failing
safe - on a bug: it ran before the camera's /tf_static frames had arrived; it
now waits up to 5 s for TF). Gap 59.9-77 cm over 1.5-2.9 m. Goal 2.5 m forward.

Result: **SUCCEEDED**, 2.46 m, 94 mm from goal, 76.6 s. No map-frame jumps
(odometry and map recordings agree). Clearance from the ODOMETRY trajectory
(`record_path.py` now records odom too) using the body's corners and heading:
**minimum 0.9-1.1 cm right at the narrowest slice, 0.6-0.8 cm left near the
exit**. The robot yawed up to ~10 deg inside the gap; at 0.42 m length that
swings the corners ~4 cm - most of the margin. Straight, the same path would
leave ~5 cm a side.

Also in this run: the planner failed for the first 2.3 s ("failed to create
plan with tolerance 0.25") and Nav2 ran a Spin at the start position (+36 deg,
1.4 m from the gap); two brief "collision ahead" stops inside the gap; average
0.042 m/s (cost-regulated slowing).

**Status vs requirement:** 15 cm/side passes reliably (8.9 cm min). 10 cm/side
gets through with ~1 cm corner margins - not yet trustworthy. Next: keep the
robot straight inside a gap (RPP tuning), and find why the first plan failed.

### Problem 35 — remote Foxglove starved the obstacle sensor (2026-10-09)

**Symptom.** 100 "observation buffer has not been updated" warnings; the point
cloud fell to 4.4 Hz, then 2.2 Hz, with gaps of 1.3-1.8 s. With
`expected_update_rate: 1.0` (Problem 34) every gap made the costmaps stale and
Nav2 refused to drive - the fail-safe doing its job, but navigation unusable.

**Cause.** `foxglove_bridge` at 79-98% CPU (load 8.7 on 6 cores) serving a
remote Foxglove session over Tailscale that had opened
`aligned_depth_to_color/image_raw` (raw 16-bit depth, ~9 MB/s, deflated per
message). Its outbox to the client stayed full and the CPU stayed high even
after that panel was closed. The camera driver lost the CPU race.
**A/B proof:** bridge stopped -> cloud 7.3 Hz, no gap > 1 s, CPU 57% idle;
bridge restarted with no client -> 4-7% CPU, cloud 10.5 Hz, worst gap 269 ms.
`/map` was ruled out (232x217 cells, 3 kB deflated, ~0.07 Hz).

**Fix.** `foxglove.launch.py` hides raw images and the raw point cloud by
default (`topic_whitelist` = `^(?!.*(/image_raw|/image_rect_raw|/points)$).*$`,
verified on the running bridge: raw streams hidden, compressed colour, map,
costmaps, TF offered). `heavy:=true` restores everything for wired,
non-driving use. Tool: `tools/cloud_rate.py` (rate + worst gap vs the 1 s limit).

**Lesson.** On a robot whose only obstacle sensor shares a 6-core CPU with the
visualisation bridge, what a remote viewer opens is a safety question.

### Pre-goal localisation check (2026-10-09)

`localisation_check.py` (also run by `nav_goal.py` before every goal - it
refuses on a wrong pose). Scores live obstacle returns that land within one cell
of a mapped obstacle AND whose ray from the robot crosses no mapped obstacle,
then searches +-2 m / +-15 deg for a clearly better pose. The first version
lacked the ray test and the search always found a "100%" pose by pushing the
cloud into dense furniture - its 4/4 self-test passed while meaning nothing.
With the ray test: wrong poses score 0-39%, the search recovers 57-60%, 4/4
classified correctly (`--selftest`). Margin is modest (true pose 45% vs best
alternative 57% at 0.67 m / -15 deg) - watch for false refusals.

### M4.T5c — 61 cm gap (10 cm/side): reached goal, but on a STALE pose (2026-10-07)

Gap measured 59.8-61.4 cm at 1.3-1.6 m (9.6-10.4 cm/side). Centring change
applied first: `inflation_radius` 0.26 -> 0.45, `cost_scaling_factor` 3.0 -> 5.0
(and RPP `inflation_cost_scaling_factor` 3.0 -> 5.0, which must match). The
costmap's gap edges matched the camera to within 2 cm; the planner still ran
1-5 cm off centre, which is the 5 cm grid (a 61 cm gap is ~12 cells across).

Result: SUCCEEDED after 80.7 s with repeated "collision ahead" stalls, a refused
Spin (correct - no room) and a Wait. **The recorded map-frame path jumps 1.8 m
backwards 0.83 m into the run**, matching RTAB-Map's "Localization was good"
at 19:38:42, 13 s after the 19:38:29 start. Cause: the robot was moved by hand
to set up the test, and neither wheels nor gyro saw it, so the goal ("2.3 m
ahead") was issued from a pose 1.8 m wrong. After the correction the goal was
~4.1 m away; the stale pose's phantom obstacles explain the stalls. Clearance
in the gap cannot be computed from this recording (frame jump), so this run is
**not** counted as a pass.

**Rule:** after ANY hand move, relocalise and run `tools/check_alignment.py`
before issuing a goal. Next: build that check into `nav_goal.py` so a goal is
refused when the live view does not match the map.

### Problem 34 — Nav2 ran BLIND for a whole session, and nothing said so (2026-10-07)

**Symptom.** The planner routed through a measured 72 cm gap ~11 cm off-centre,
leaving 2.5 cm clearance on one side. Comparing layers across the gap: the saved
map had neither box (placed after mapping), the global costmap had the left
side only, the local costmap had nothing.

**Cause.** Both costmaps take live obstacles from
`/camera/camera/depth/color/points`, which exists only with `pointcloud:=true`.
`robot.launch.py` defaulted it to **false** - while its own description said
"REQUIRED for Nav2". 0 cloud messages in 6 s. The left "obstacle" in the global
costmap was the inflation of an unrelated mapped wall. A goal through the gap
would have driven into the right-hand box: the collision check had no data.

**Why it was silent.** A costmap observation source with no
`expected_update_rate` never goes stale. No data looks exactly like "no
obstacles".

**Fix.**
1. `robot.launch.py` `pointcloud` now defaults to **true**; off is an explicit
   choice for teleop-only mapping.
2. `expected_update_rate: 1.0` on `depth_cloud` in both costmaps - fail closed.
   **Verified on hardware:** with the cloud off, a 0.3 m goal into open space
   was refused, robot moved **0.0 cm**, and the log said exactly why:
   `The /camera/camera/depth/color/points observation buffer has not been
   updated for 9.80 seconds`.

**Lesson.** A safety sensor's absence must stop the robot, never just empty the
picture. Any new observation source gets an `expected_update_rate`.

### Map v4 — v3 extended in a second session (2026-10-07)

`robot.launch.py slam:=true` (load the saved map AND keep adding to it). RTAB-Map
matched the old map 10 times while still stationary, so the new session joined
the existing graph instead of starting a disconnected one. Result: 2 sessions,
33.6 m total odometry (+25 m this session), 1973 poses, 2172 links; this session
added 451 nodes with 141 loop closures + 254 proximity detections, 825-914
features/frame, largest single jump 95 mm, no jumps over 0.5 m. Final map->odom
correction 567 mm (~2% of distance driven, absorbed by loop closures).
Clean shutdown, integrity ok, 712 MB, backed up to
`docs/maps/backup/lab_v4_extended_20261007.db`.

**Resolved, not a fault:** a 26 deg heading change "while stationary" in
localisation mode that morning was the robot being moved by hand. Live-depth
alignment against the map was 95.7% on occupied cells, 0.0% in free space.

### Problem 33 — my `z_voxels: 20` "fix" was rejected by Nav2 (2026-10-07)

Nav2's voxel grid supports at most 16 levels; at startup it printed
`Error, this implementation can only support up to 16 z values (20)`, so the
earlier enlargement of the grid to contain the camera never took effect. Correct
fix: `z_voxels: 16`, `z_resolution: 0.06` -> 0.96 m, containing the camera
(0.844 m) and max_obstacle_height (0.90). Verified on relaunch: 0 z-limit errors,
0 "sensor origin out of map bounds".
**Lesson:** read the startup log for the fix itself, not just for the symptom.

### Recoveries: BackUp removed (2026-10-07)

`config/behavior_trees/navigate_to_pose_no_backup.xml` = Nav2's stock
replanning+recovery tree minus `BackUp`. Reversing is blind (no rear sensor) and
rotates this chassis ~95 deg per 0.2 m. Remaining recoveries are clear-costmap,
Spin (accurate in place) and Wait. Wired in via `navigation.launch.py`.

`scripts/stop_robot.sh --nav` stops only Nav2, keeping rtabmap and the robot
running, so a params change does not cost a relocalisation.

### Map v3 — fresh teleop map over WiFi (2026-10-07)

Re-mapped from scratch after Problem 29. Teleop via the Foxglove Teleop panel at
10 Hz (`cmd_vel_timeout` is 0.5 s, so Foxglove's default 1 Hz makes the robot
stop-start). Results: 8.4 m driven, 1278 poses, 201 nodes, **37 loop closures +
85 proximity detections**, 860-900 features/frame, final map->odom correction
319 mm, largest single jump 96 mm, no suspect (>0.5 m) jumps. Clean shutdown
under the new stop script; `PRAGMA integrity_check` = ok. 463 MB, backed up to
`docs/maps/backup/lab_v3_20261007.db`.

### Problem 32 — left driver silent: RX/TX reversed (2026-10-07)

`drive link unhealthy (modbus faults L=... R=1)`, L rising ~8.5/s. The firmware
times out each request after 120 ms, and 1/0.120 = 8.3/s, so EVERY request was
timing out with no reply at all - not garbled replies, which would point at
noise. Cause: the left driver's RX and TX wires to Mega Serial1 (pins 18/19)
were reversed. Swapping them cleared it instantly. Verified with a 1.2 s
forward test: left +6.1 cm, right +5.9 cm, same direction.
**Diagnostic rule:** fault rate == 1/kTimeoutMs means "nothing is answering" -
check power, RX/TX crossing and GND before suspecting the driver.

### Problem 29 — every restart was corrupting the map database (my shutdown script)

**Symptom.** Nav2 rejected every goal with "no map -> base_footprint
transform". RTAB-Map had died two minutes into the run:
`Memory.cpp:3852::addLink() Condition (fromS->getWeight() >= 0 ...) not met!`,
preceded at startup by `Regenerated the dictionary with 5724 missing words`.

**Cause.** `scripts/stop_robot.sh` sent SIGTERM, waited 6 s, then SIGKILL to
everything. RTAB-Map flushes working memory to SQLite on SIGTERM; for a 1.9 GB
map that takes far longer than 6 s, so every restart cut the write short.

**Fix.** The script records whether rtabmap was running, then polls until it
has actually exited (`RTABMAP_GRACE`, default 90 s) before escalating, and warns
if it had to kill it. Tested both branches: no rtabmap -> no database message
(an earlier draft claimed "closed cleanly" when nothing had run); a decoy that
ignores SIGTERM -> waited the grace, warned, killed. Damaged DB kept in
`docs/maps/backup/`; the lab was re-mapped from scratch.

### Problem 30 — the display costmap hides lethal cells

`/local_costmap/costmap` is an OccupancyGrid rescaled to 0..100 (254 -> 100,
253 -> 99). Thresholding it at 253/254 found nothing, so a footprint sitting in
cost 253 read as "0% lethal, footprint clear". `check_costmap.py` now reads
`/local_costmap/costmap_raw` (nav2_msgs/Costmap, true 0..255) and draws the
costmap around the robot as ASCII with the footprint on it. Its output was
cross-checked against the live point cloud and matched to the cell.

### Problem 31 — DWB cannot look far enough ahead at 0.06 m/s

DWB's `sim_time` is a time: 3 s x 0.06 m/s = 18 cm of lookahead for a 42 cm
robot, so obstacles 40-80 cm ahead were invisible to it and it stalled. Switched
`FollowPath` to Regulated Pure Pursuit, whose lookahead is a distance
(0.3-0.7 m) and so does not shrink with the speed limit. Also: `inflation_radius`
0.30 -> 0.26 and `footprint_padding: 0.01` made explicit, giving a 10.6 cm
cost-free ribbon through a gap with 10 cm clearance per side (it was 2.6 cm).

**Open finding: reversing rotates the robot.** A commanded straight 20 cm
reverse at 0.05 m/s turned the robot ~95 deg (odometry, displacement direction
and gyro agree). Forward motion is 0.6 deg. Likely the castors flipping through
180 deg. Nav2's `backup` recovery reverses, so it should be replaced or limited.

### Problem 28 — RETRACTED: there is no drive PID droop, and no retune is needed

**This entry previously claimed an ~8 point drive PID droop under load and
recommended retuning the RMCS-2303 gains. That was WRONG on two counts and the
recommendation is withdrawn.**

The original comparison was straight-line motion at 0.06 m/s against in-place
rotation at 0.3 rad/s. Those are NOT the same operating point:

    straight 0.06 m/s  -> wheels 1.200 rad/s
    rotation 0.3 rad/s -> wheels 0.801 rad/s

Different wheel speeds. The gap could have been speed-dependent or
load-dependent, and the test could not distinguish them. On top of that, the
summary code applied a bad lag-shift and reported `scale 2.795` with
`cmd rms 0.213` for a run whose raw CSV plainly showed `L_cmd -0.797,
L_meas -0.799`. The aggregate was broken while the raw data was correct.

**Controlled re-test at MATCHED wheel speed** (`docs/logs/eq_straight.csv`,
`docs/logs/eq_rot.csv`), same drives, same session, same battery state:

| condition | wheel rad/s | L scale | R scale | err rms |
|---|---|---|---|---|
| A straight (low load) | 0.794 | 0.972 | 0.970 | 0.14 |
| B rotation (high load) | 0.796 | **0.983** | **0.979** | 0.10 |

Rotation tracks about **one point BETTER** than straight. There is no
load-dependent droop; the drives hold speed under the load of scrubbing castors.
**The PID tuning is fine and should be left alone.**

**What this leaves.** The wheels deliver ~97-98% of command in both conditions,
while a separate measurement put the robot's actual rotation at ~86% of
commanded. If both hold, the difference is genuine slip during rotation - which
is where this investigation started (Problem 25). Straight-line motion remains
provably clean.

**The lesson is about method, not the robot.** Two operating points that differ
in more than one variable cannot attribute a difference to either of them. The
confound was visible in a two-line calculation and was not done until after the
conclusion had been written up and a hardware change recommended.

### Problem 27 — the "drivetrain lag" was MY acceleration limits, not the drivetrain

The user pushed back on the castor explanation: *"it's not about castors, it was
moving fine straight before, currently it's not."* That was correct, and the
decisive test was to remove Nav2 entirely.

**Drivetrain only, no camera, no EKF, no Nav2, omega commanded = 0:**

| | historical known-good | this test |
|---|---|---|
| odometry | 1.0037 m | **1.0035 m** |
| lateral drift | -5.8 mm | **-4.4 mm** |
| heading | -0.59 deg | **-0.34 deg** |
| drivetrain lag | - | **+0 ms** |
| measured L/R ratio | - | **1.000** |
| asymmetry | - | **-0.0%** |

**Zero lag, zero asymmetry, perfectly straight.** The drivetrain and firmware are
flawless. The 120-220 ms of "drivetrain lag" appeared ONLY under Nav2.

**Cause: the acceleration limits, which had been tightened for "gentleness".**

```
acc_lim_x     0.15 m/s^2  -> 0 to 0.06 m/s  = 400 ms = 2 control periods
acc_lim_theta 0.4 rad/s^2 -> 0 to 0.3 rad/s = 750 ms = 3.75 control periods
controller period at 5 Hz = 200 ms
```

The robot could never reach a setpoint before the next one replaced it, so it
permanently trailed the command. Cross-correlation reads that trailing as
"drivetrain lag", and the same effect produced the 0.794 rotation gain that was
attributed to castor slip. **Neither was real.**

**Fix:** `acc_lim_x` 0.3 (full speed in 200 ms), `acc_lim_theta` 1.0 (300 ms),
with the velocity_smoother raised to match so it cannot become the tighter limit.

**Result:**

| | before | after |
|---|---|---|
| heading error at goal | +/-9.4 to 9.7 deg (at the tolerance limit) | **+2.5 deg** |
| spin-only commands | 16.0% | **0.0%** |
| both wheels same direction | 82.5% | **100.0%** |
| only one wheel turning | 1.4% | **0.0%** |
| wheel tracking scale | 0.994 / 0.995 | **0.998 / 0.997** |

**Lesson.** "Gentle" acceleration is not free: an acceleration limit is a rate
limit on the whole control loop, and if it cannot reach a setpoint within about
one control period the robot trails every command it is given. Three separate
symptoms - apparent drivetrain lag, low rotation gain, and jerky stop-and-turn
motion - all traced to one parameter pair I had tightened for comfort.

**What survives about the castors:** straight-line motion is now provably
perfect, so the castors do NOT affect it. The ~20% rotation gain shortfall does
persist and cannot be geometry (it would need an effective wheel separation of
0.334 m against a 0.300 m physical span), so slip during ROTATION is still real.
Straight = clean, rotation = slips. Both were true; conflating them was the error.

**A tool bug found in the same session:** the lag-attribution verdict fell through
to "dominated by MEASUREMENT" when BOTH lags measured zero, confidently
diagnosing a problem the data said did not exist. Now guarded, along with a
minimum-sample check.

### Problem 26 — 220 ms of drivetrain lag is what makes the motion jerky

User report: "its movement is NOT smooth, both motors should work together".
Measured with `verify_wheels.py` (commanded vs measured wheel speeds on ONE
interpolated timebase, using the controller's own live kinematics) and
`analyse_motion.py` (is the command an arc or a segment?).

**The motors are fine.** After removing the lag:

| wheel | cmd rms | act rms | err rms | err mean | scale |
|---|---|---|---|---|---|
| left | 0.609 | 0.610 | 0.075 | -0.008 | **0.994** |
| right | 0.717 | 0.716 | 0.069 | -0.006 | **0.995** |

Each wheel delivers 99.5% of what it is asked for with near-zero bias. On
straight segments the COMMANDED L/R ratio was 0.945 against a MEASURED 0.938 - so
of the apparent 6.4% asymmetry only **0.7% is the drivetrain**, the rest is the
controller genuinely steering. And `analyse_motion.py` shows the controller
commands coordinated arcs 53-74% of the time with **0% of samples having one
wheel idle** at normal speed - the motors do work together.

**The fault is timing:**

```
drivetrain lag (cross-correlation): +220 ms
```

`controller_frequency` is 20 Hz (50 ms), so the plant does not respond for **more
than four control cycles**. The controller corrects, sees nothing, corrects
harder, and then the first correction lands. That is lag-induced oscillation, and
it accounts for the jerky motion, the wandering heading (+/-9.7 deg at every
goal) and the long plateaus in the final approach.

It is consistent with the hardware: MODBUS ASCII at 9600 baud is ~37 ms per
transaction (Problem 5 / the RMCS-2303 protocol notes) and the firmware cycles
control / speed / position per wheel.

**Root cause, from the firmware's own timing.** The firmware issues ONE MODBUS
transaction per wheel per cycle (~37 ms at 9600 baud), prioritised
control > speed > position. Wheel velocity is differentiated from consecutive
POSITION reads, and reads only get the SPARE slots:

| controller rate | cycle | transaction slots | free for position |
|---|---|---|---|
| 20 Hz | 50 ms | 1.4 | **0.4** |
| 10 Hz | 100 ms | 2.7 | 1.7 |
| 5 Hz | 200 ms | 5.4 | 4.4 |

At 20 Hz a new speed is dirty almost every cycle, so speed writes and position
reads alternate and the feedback is ~74 ms stale before transport. The measured
budget: 37 (write speed) + 37 + 37 (velocity needs TWO position reads) + 20
(status frame) = **131 ms**, before any drive ramp.

**Fixes applied (both NOT YET VERIFIED ON HARDWARE):**
1. **`controller_frequency` 20 -> 5 Hz.** Fixes it twice over: the controller
   stops out-running the plant, AND the freed slots make the feedback fresh.
2. **Firmware starvation guard.** `kMaxWritesBetweenPos = 2` in `wheel_drive`:
   after two writes the next slot is forced to be a position read, bounding
   feedback staleness no matter how fast the host commands. Covered by a new host
   test, verified to FAIL when the guard is disabled - a test that passes either
   way would be worthless.

**RESULT after both fixes (hardware-verified 2026-08-06):**

| | before | after |
|---|---|---|
| total lag | 220 ms | **120 ms** |
| L/R speed ratio (median) | 0.635 | **0.979** |
| straight-segment asymmetry | -6.4% | **-2.2%** |
| samples with one wheel idle | 27.3% | **1.4%** |
| wheel tracking scale | 0.994 / 0.995 | 0.994 / 0.995 |

**And the lag breakdown, which changes the conclusion:**

```
command -> GYRO     :  +40 ms   actuation lag
command -> ENCODERS : +120 ms   actuation + measurement
difference          :  +80 ms   stale encoder feedback
```

**The drivetrain responds in 40 ms.** It was never slow - two thirds of the
remaining lag is the FEEDBACK PATH. Slowing the controller was right for
stability, but the deeper fix is fresher odometry, not a slower controller. Since
actuation is only 40 ms, `controller_frequency` could go back up somewhat once
the feedback is fresher; 5 Hz is currently matched to the 120 ms feedback, not to
the 40 ms plant.

**Separately: `gyro/command gain = 0.794`.** The wheels deliver 99.4% of what
they are asked, but the robot rotates at only **79%** of the commanded rate. That
missing 20% is castor slip - the same mechanism as Problem 25's 4-12% per
revolution, measured a different way and confirming it independently.

**A BUG IN THE FIRST ATTEMPT AT THIS MEASUREMENT, worth recording.** The initial
gyro comparison used `angular_velocity.z` directly and reported
`gyro/command gain = -0.196` with a nonsensical -1660 ms lag. `/imu/unbiased` is
in **camera_imu_optical_frame**, not the robot frame, and on this robot

```
robot yaw rate = +0.018*gx  -0.982*gy  -0.187*gz
```

so the dominant term is **-gy**. The observed gain of -0.196 is simply the gz
coefficient (-0.187): it was measuring 19% of the yaw rate, inverted. The EKF is
unaffected because it transforms through TF. **Any tool consuming raw IMU axes
must do the same** - the conclusion drawn from the broken version ("dominated by
measurement") happened to survive the fix, but that was luck, not method.

**Superseded: separate ACTUATION lag from MEASUREMENT lag.** The 220 ms figure
conflates them, and they have opposite fixes:
* actuation lag - the wheels really are late; slow the controller;
* measurement lag - the wheels move on time but the ENCODER FEEDBACK is late, so
  odometry and the EKF report a stale pose and Nav2 overshoots.

`verify_wheels.py` now separates them using the gyro, which is an independent
200 Hz witness with ~5 ms latency: command->gyro is actuation lag alone,
command->encoders is actuation + measurement. Run it next session.

**Lesson: measure lag separately from error.** A drivetrain that tracks perfectly
but 220 ms late is indistinguishable from one that tracks badly, until they are
separated - and the fixes have nothing in common.

### THE FINDING THAT UNBLOCKED NAVIGATION: a blind robot cannot map its own floor

The first navigation attempt was **ABORTED with the robot never moving**, and the
planner logged `failed to create plan with tolerance 0.25`. Sampling the costmap
along the intended path showed why:

```
  dist   costmap   raw map
   0.0       100       100     <- the robot was standing ON a cell
   0.1       100       100        the map called OCCUPIED
   0.2       100       100
   0.3        99         0
```

**A robot with a 0.79 m blind zone cannot map the ground it is standing on.**
Every map built up to that point came from SPINNING IN PLACE, so the floor within
~1 m of the spin centre was never observed from a distance. It was never cleared
as free; instead noisy grazing-angle returns filled it as occupied. The robot had
mapped everything except the one place it needed to be, and a planner cannot plan
out of a lethal cell.

**Fix: map by DRIVING.** Out 1.5 m, turn 180, back 1.5 m, turn 180 - so the start
area is observed from 1.5 m away. Result:

| | spin-only map | driven map |
|---|---|---|
| free cells | 6107 | **6588** |
| occupied cells | 7296 | **4663** |
| cells under the robot | **occupied** | **free** |

This is a permanent property of this robot, not a one-off. **Any mapping run must
include translation, not just rotation**, or the robot's own surroundings are
unmappable.

### One-command bringup, and the `pkill` trap that cost four incidents

```bash
ros2 launch labrobot_bringup robot.launch.py                    # drive + see
ros2 launch labrobot_bringup robot.launch.py mapping:=true      # build a map
ros2 launch labrobot_bringup robot.launch.py localization:=true # use a map
scripts/stop_robot.sh                                           # stop it all
```

**Keep the robot still for the first ~30 s** - the gyro bias is estimated from
10 s of stationary data and everything downstream inherits it.

The delays in `robot.launch.py` are load-bearing, not cosmetic: localization
starts at t+22 s because `imu_bias_compensator` averages the first 10 s of IMU it
receives, so starting it before the camera streams averages a partial stream;
mapping starts at t+45 s because RTAB-Map drops frames if the EKF is not yet
publishing `odom -> base_footprint`. Started by hand in the wrong order, all of
these present as "the map is bad" rather than "a node started 3 s too early".

**`pkill -f <pattern>` kills its own shell**, because the shell's command line
contains the pattern - the kill is *in* that command line. It exits 144 with no
error, having killed some targets and not others. This happened **four times** in
this project.

The usual workaround, the bracket form `realsense2[_]camera_node`, only helps
when that is the ONLY occurrence. A script that also runs
`ros2 launch labrobot_bringup ...` further down still has the plain string in its
command line and still dies. `scripts/stop_robot.sh` fixes it properly by keeping
the patterns **in a file**, so the killer's own command line can never contain
them, plus excluding self, parent and any shell.

Two smaller bugs found while writing it, both worth knowing:
* `pgrep -fc` prints `0` **and** exits non-zero when nothing matches, so
  `n=$(pgrep -fc x || echo 0)` yields the two-line string `"0\n0"` and `$(( ))`
  then fails with "syntax error in expression".
* Filtering self only in the *kill* step but not the *count* step made the script
  report "6 still matching" with nothing running - it was counting itself under
  every pattern. One shared filter function, used by both.

### M3.T3 — map save/load and localization-only mode

```bash
# build a map
ros2 launch labrobot_bringup mapping.launch.py

# stop it with SIGINT (NOT SIGKILL - the database is written on clean shutdown)

# come back later and localise in the saved map, without extending it
ros2 launch labrobot_bringup mapping.launch.py localization:=true
```

Localization mode sets `Mem/IncrementalMemory: false` (stop creating nodes) and
`Mem/InitWMWithAllNodes: true`. The second matters more than it looks: without it
the robot can only recognise places near where it *already thinks* it is, so a
robot switched on somewhere unexpected never localises and simply sits there
reporting no correction - which looks exactly like a broken map.

**Verified**, after restarting into a saved 1011-node database:

| | |
|---|---|
| loop closures | **15 in 40 s** against stored nodes 784-791 |
| proximity detections | 18 |
| best hypothesis | **0.973** |
| working memory | 165 nodes, **stable** (mapping mode grows continuously) |
| per-iteration cost | 0.16 s, down from 0.26 s in mapping mode |
| **map->odom correction** | **206.8 mm, -11.10 deg** |

That last row is the point of the exercise. RTAB-Map restarted knowing nothing
about where the robot was, recognised the place from the camera alone, and
recovered a pose that odometry had drifted **11 degrees** away from.

**Database size.** 385 MB for 1011 nodes = **390 kB per node**, of which depth
images are 215 MB (56%) and colour 71 MB (18%). Not a problem here - the Jetson
has 842 GB free, so even a 10 000-node map is 4 GB. If it ever does matter,
`Mem/ImagePostDecimation` is the knob, at the cost of loop-closure quality on
reload. Deliberately not set: degrading map quality to solve a non-problem.

### Remote visualisation

Remote access is over **SSH** (Tailscale removed from the project, 2026-10-31).
The bridge (started by robot.launch.py) listens on **127.0.0.1 only**; from your PC:

    ssh -N -L 8765:localhost:8765 zine@<jetson-ip>

then in Foxglove: **Open connection -> Foxglove WebSocket -> `ws://localhost:8765`**.
SSH provides authentication and encryption; a bridge on 0.0.0.0 has neither and
lets anyone on the network drive the robot (it accepts client publishing).
`foxglove.launch.py address:=0.0.0.0` reopens it on a trusted bench network.

**Network (2026-10-31):** an Intel AC8265 Wi-Fi/Bluetooth card is fitted in the
Jetson's M.2 E-key slot (iwlwifi, firmware 8265-36, interface `wlP1p1s0`,
Bluetooth hci0). It joins the "Galaxy" hotspot automatically (autoconnect yes;
10.58.238.11 at first connection, -42 dBm, 130 Mbit/s on 2.4 GHz). Reach it by
name - `ssh zine@ubuntu.local` (avahi/mDNS) - since hotspot addresses change.
The USB-tethered phone is no longer needed for the network.
Tailscale removed from the Jetson on 2026-10-31 (`systemctl disable --now
tailscaled`, `apt-get remove tailscale`) after SSH over the hotspot Wi-Fi was
verified (session from 10.58.238.66). Log: `~/tailscale_removal.log`.
Verified 2026-10-31: Foxglove connects through the tunnel (bridge sees the client
at 127.0.0.1). The bridge only runs with the robot stack; without it,
`ros2 launch labrobot_bringup foxglove.launch.py` starts it alone (a tunnel to a
port nobody listens on is what "the ssh -L command is not working" turned out
to be). This bridge speaks the `foxglove.sdk.v1` subprotocol (101 Switching
Protocols; `foxglove.websocket.v1` gets 400).
**Caution:** Foxglove's "Pose estimate" tool publishes /initialpose, which
RTAB-Map obeys - a stray click relocates the robot. Use /waypoint_pick for
waypoints.

Chosen over the alternatives because RViz needs either X forwarding (unusable
over SSH) or a matching `ROS_DOMAIN_ID`, and the latter means DDS discovery plus
raw sample traffic across WiFi/VPN - multicast discovery is unreliable over both
and a 640x480 cloud at 15 Hz is ~110 MB/s of UDP that will not fit. The bridge is
one TCP WebSocket and subscribes to a topic only while a panel is open on it.

**Cost of what you open**, worst first:

| topic | rate | note |
|---|---|---|
| `/camera/camera/depth/color/points` | ~110 MB/s | needs `camera.launch.py pointcloud:=true`; open briefly |
| `/camera/camera/color/image_raw` | 101.6 Mbit/s measured | **never open over WiFi** |
| `/camera/camera/color/image_raw/compressed` | **3.15 Mbit/s** measured (q60) | **live video over WiFi - use this** |
| `/rtabmap/cloud_map` | bursty, grows | the assembled map cloud |
| `/map` + `/tf` | small | what you actually want for "is the map right?" |

**Low-latency live video (2026-10-07).** Remote work is over WiFi, so the
colour stream is JPEG-compressed on the robot and Foxglove's Image panel renders
it directly.

* No H.264: the Orin Nano has **no NVENC** (no `/dev/nvhost-msenc`; NVIDIA
  dropped the encoder on this SKU). Software x264 would compete with VO for CPU.
* Installed `ros-humble-compressed-image-transport`. realsense2_camera already
  publishes through `image_transport`, so `/compressed` topics appear with no
  extra node and no extra image copy.
* `camera.launch.py jpeg_quality:=60` (default). Measured, 640x480 @ 15 fps:
  q95 82 kB/frame 9.3 Mbit/s, **q60 25.7 kB 3.15 Mbit/s**, q30 17.6 kB 2.15 Mbit/s.
* **Quality only applies at launch.** `ros2 param set` reads back the new value
  and changes nothing - the plugin caches it when the publisher is built
  (sweep q85..q20 on a live node: 82.8..80.7 kB, i.e. no effect).
* foxglove_bridge `send_buffer_limit` 10 MB -> 1 MB and `max_qos_depth` 10 -> 2.
  The bridge *drops* when the buffer fills; 10 MB was ~19 s of queued video, so
  on a weak link you would teleop while watching the past.
* Measure, don't trust the parameter: `python3 tools/image_bandwidth.py`.

### Resource budget — Jetson Orin Nano 8 GB, 6 cores, 15 W

Measured with `ros2 run labrobot_bringup check_system.py`, which reports every
topic rate, per-node CPU/RSS/threads, machine load, RAM, swap and thermals in one
place. Run it after any change that adds a consumer.

**Rate plan.** Every rate below is deliberate, and `check_system.py` fails if one
drifts, so a regression shows up as a red line rather than as vague slowness.

| stream | Hz | why this number |
|---|---|---|
| depth + colour | **15** | see below - 30 actively made things worse |
| visual odometry | **15** | 1:1 with the camera; every frame is processed |
| IMU (gyro/accel united) | 200 | hardware rate; the D435i offers 200 or 400 |
| wheel odom / joint states | 50 | `diff_drive_controller` publish rate |
| EKF | 50 | matches the odometry it fuses |
| RTAB-Map mapping | 1 (planned) | RTAB-Map's own default; map updates are not a control loop |
| Nav2 controller / costmaps | 20 / 5 (planned) | |

**The counter-intuitive result: halving the camera rate DOUBLED visual odometry.**

| config | realsense | rgbd_odometry | VO rate | machine busy |
|---|---|---|---|---|
| 30 fps, no VO | 75.6% | - | - | 33% |
| 30 fps + VO | 71.3% | **103%** | **5.9 Hz** | 47% |
| **15 fps + VO** | **42.7%** | **66.9%** | **14.7 Hz** | **35.7%** |

At 30 fps `rgbd_odometry` could only process ~6-7 frames/s, so three of every
four frames were captured, transported and deserialised at full cost purely to be
discarded - while the queueing starved the consumer that needed them. At 15 fps
it keeps up with every frame. At 0.26 m/s the robot travels 17 mm between frames,
which is ample overlap for RGB-D odometry.

`Odom/ImageDecimation` was tried first and made things **worse** (5.9 Hz at 103%
of a core versus 7.5 Hz at 90%). Input rate, not per-frame cost, was the problem.

**Current totals with camera + drivetrain + localization + visual odometry:**
CPU 26.8% of the machine, **64% headroom**; RAM 2.42 of 7.43 GiB (33%), no swap;
hottest zone 48 C, no throttling. Comfortable room for RTAB-Map mapping and Nav2.

**Two traps found while measuring this:**
* The camera needs **~8 s** after the node is killed before it can be reopened.
  At 5 s the driver reports `Open profile: Color 640x480 FPS:15` and then
  delivers **zero colour frames** while depth runs perfectly - it looks exactly
  like an unsupported profile. `tools/rs_link_probe 20 1 15` proved librealsense
  handles 15 fps with 0 dropped frames, which is what isolated it to the race.
* `Odom/ImageDecimation` and every other RTAB-Map config parameter is declared as
  a **string**. A launch argument default of `'2'` is inferred as an integer and
  the node aborts with `InvalidParameterTypeException` and exit code -6. Wrap
  them in `ParameterValue(x, value_type=str)`.

### Problem 25 — wheel odometry loses 4-12% of every rotation (M2.T3b)

The headline result of the moving-EKF validation, and it changes the architecture.

**Symptom:** commanded a 360 deg in-place spin. Wheel odometry reported 360.70 deg.
The floor marks said the robot had physically turned **~315 deg**. The gyro said
316.42 deg.

| spin | wheel says | gyro/EKF says | ground truth | wheel error |
|---|---|---|---|---|
| 360 @ 0.4 rad/s | 360.70 | 316.42 | ~315 | **+14.5%** |
| 360 @ 0.2 rad/s | 360.70 | 346.42 | - | |
| 360 @ 0.4 rad/s | 360.66 | 337.02 | - | |
| 360 @ 0.1 rad/s | 360.78 | 340.83 | - | |
| 1800 @ 0.4 rad/s | 1800.76 | 1711.96 | - | **+4.9%** |
| cumulative, 4 spins | - | 1340.7 | ~1320 | gyro **+1.4%** |

**Cause: the drive wheels slip.** Four swivel castors that catch and release
rather than swivelling freely. Proof it cannot be geometry: explaining
360.70 deg reported against 315 deg real needs an effective wheel separation of
**0.306 m**, wider than the 0.300 m outer-face span of the tyres themselves. The
contact patches physically cannot be that far apart, so the wheels must have
turned without the robot turning. That is ~106 mm of slip per wheel per
revolution.

**It is not correctable by calibration and not avoidable by driving slower.**
The error is random in size (14-44 deg per revolution) and random in where it
occurs - the divergence profile shows it arriving in discrete bursts, e.g. 11.2
of 14.3 deg between 74 and 152 deg of one revolution. Slowing to 0.1 rad/s (63 s
for one revolution) still lost 20 deg. `wheel_separation_multiplier` fitted to
this is fitting noise.

**Two hypotheses tested and rejected along the way:**
* *Drive-command lag.* The two RMCS-2303 sit on separate UARTs precisely so the
  Modbus transactions overlap; even a full 37 ms skew is 0.85 deg at spin speed,
  two orders of magnitude short. And encoder position is absolute, so a skewed
  poll cannot lose counts.
* *The test tool's own creep phase.* `drive_test.py` eases to 0.08 rad/s over
  the last 20 deg, which does produce visible stick-slip. But the profile shows
  the final 20% of the run contributing only ~1 deg. Not the cause.

**Fix:** `ekf.yaml` no longer fuses wheel `vyaw` at all. Yaw comes solely from
the gyro. It was already outvoted 3750:1 by covariance, but leaving it listed
misrepresented the design to anyone later tempted to rebalance the covariances.
Translation is unaffected - forward distance agreed to **0.1 mm** over 1 m.

**Consequence for M4:** angular velocity limits will not fix path-tracking error
from this, and the castors are worth a mechanical look (dirt, stiff bearings, a
flat spot). Slip this bad also wastes torque.

**The suspected gyro scale error was NOT REAL — resolved in M3.T1.** Floor marks
suggested the gyro over-reported by ~1.4% (5.2 deg per revolution), but the
measurement uncertainty was +/-0.7%, half the value. Rather than apply a
correction, it was deferred to a better instrument. RTAB-Map visual odometry then
measured, over a full 360 deg spin:

| | rotation |
|---|---|
| EKF (gyro) | 360.52 deg |
| visual odometry | 360.04 deg |
| difference | **0.48 deg (0.13%)** |

Two independent physics - MEMS rate integration and image feature geometry -
agreeing to half a degree per revolution. **No correction is needed, and applying
the 1.4% one would have injected the very error it was meant to remove.** This is
the whole argument for not tuning against a measurement whose uncertainty is
comparable to the effect.

### Problem 24 — fusing the gyro made the robot WORSE, twice

**Symptom:** with the EKF fusing wheel odometry plus the D435i gyro, the fused
heading drifted **3.52 deg/minute while the robot stood perfectly still**. Wheel
odometry alone drifted 0.000 deg. Adding a sensor had made localisation worse.

**Cause:** the measured gyro yaw bias is +0.11 deg/s = 6.7 deg/minute, and
nothing in the stack was removing it.

**First fix, which did not work.** I pointed the EKF at
`imu_filter_madgwick`'s `/imu/data` on the reasoning that it has
`do_bias_estimation: true`. Measured result: raw +0.1072 deg/s in, filtered
+0.1072 deg/s out — **0% removed**. Madgwick applies its bias estimate only to
the ORIENTATION it computes and republishes `angular_velocity` untouched. The
reasoning was sound but the premise about what that node publishes was wrong.

Fusing Madgwick's yaw *orientation* instead would not have helped either: with no
magnetometer, yaw has no absolute reference and drifts at the bias rate anyway.

And `robot_localization` cannot do it: its 15-state model is
`x y z / roll pitch yaw / velocities / accelerations` — there are **no gyro-bias
states**, so it integrates whatever it is handed.

**Working fix:** a dedicated node,
`labrobot_bringup/scripts/imu_bias_compensator.py`. It averages the gyro for 10 s
at startup with the robot stationary, then subtracts that bias from every sample.
It also fills in `angular_velocity_covariance`, which the driver leaves at zero —
and a zero covariance reads to an EKF as "infinitely trustworthy", so the filter
would have ignored the wheels entirely.

| | raw | compensated |
|---|---|---|
| yaw bias | +0.1122 deg/s | +0.0009 deg/s |
| drift | 6.73 deg/min | **0.05 deg/min** (99.2% removed) |
| **EKF stationary drift** | **−3.52 deg/min** | **−0.04 deg/min** |

**Deliberate limitation:** bias is estimated at startup only, not continuously.
Re-estimating whenever the robot looks stationary would also track thermal drift,
but a robot that is "stationary" because it is jammed against a wall with wheels
slipping would poison the estimate with real rotation. Startup-only is
predictable, and the node logs the value so drift is visible by re-running it.

**Also fixed:** the node died instantly on first launch with exit code 2 and only
a bare "process has died" in the log. `argparse` was rejecting ROS's own
`--ros-args`. Any rclpy script with a CLI needs
`parse_known_args(remove_ros_args(sys.argv)[1:])`.

---

## 6. Verified on hardware (2026-07-30)

| Check | Result |
|---|---|
| Commanded 10 wheel-rpm | 10.04 measured (+0.4%) |
| Commanded 40 wheel-rpm | 40.28 forward / 40.37 reverse |
| Counts per wheel rev | 12532 confirmed to 0.2% |
| Reverse | counts decrease, sign correct |
| Link errors under load | `crc=0`, `to=0` |
| Brake | holds position, counts frozen |
| Auto-brake | fires at 4 s |
| Forward direction | confirmed visually |
| Camera optical Z in `base_footprint` | (0.966, 0, −0.259) — 15° down, applied once |
| Binary link | 50 Hz status, 0 frame errors over thousands of frames |
| Watchdog trip | 470 ms after last command (configured 500 ms) |
| Watchdog stop | 240 ms to brake from 10 wheel-rpm |
| Travel after host goes quiet | 34 mm at 10 wheel-rpm (~130 mm at full 0.26 m/s) |
| ros2_control plugin | loads, configures, activates; both controllers active |
| `/joint_states` | publishes both wheel joints, position + velocity |
| `/diff_drive_controller/odom` | publishes on `odom` → `base_footprint` |
| Status rate through ros2_control | 49.9 Hz sustained |
| Frame loss after the bootloader fix | **0.00%** in every window, including the first |
| Closed-loop straight | commanded 0.300 m, odometry 0.3115 m, lateral drift 0.2 mm |
| Closed-loop rotation | commanded 360°, odometry 360.21° |
| Straight-line calibration | odometry 1.0208 m vs tape 1.025 m — radius correct to 0.41% |
| Spin calibration | odometry 360.43° vs real ~370° — `wheel_separation_multiplier = 0.9741` |
| Spin **verified after fix** | odometry 360.84° vs real ~360.5° — **0.09% error** (was 2.7%) |
| Tool coast after ramp fix | 0.74°/turn (was 13.73°) |
| Radius scale (2 runs, weighted) | odometry under-reads 0.35% → both multipliers >1 |
| Radius asymmetry | left wheel 0.374% larger → 20 mm right / 1.36° over 2 m |
| Straightness after radius fix | real heading 1.36° → **0.00°** over 2 m |
| Odometry heading error | +1.61° → −0.65° → **−0.09°** at `asym 0.266%` |
| Odometry lateral over 2 m | +5.9 mm → −20.9 mm → **−1.2 mm** |
| Calibration noise floor | ~0.3°/2 m — tape-limited, further iteration is resampling noise |
| Final multipliers | sep 0.9741, L radius 1.00480, R radius 1.00212 |
| Effective wheel separation | 0.2669 m (was 0.274) |
| D435i | USB 3.2, firmware 5.17.0.10, serial 242522072324 |
| Depth + colour @ 848x480 / 640x480 x30 | 29.98 / 28.07 fps, **9.4%** of one core |
| Same with `align_depth` | 29.99 / 29.98 fps, **39.4%** of one core |
| Everything incl. IMU | **66.1%** of one core |
| IMU rates (counted directly) | imu 199.5 Hz, gyro 199.5 Hz, accel 100.5 Hz |
| IMU frame_id | `camera_imu_optical_frame` - matches the URDF from M0 |
| Depth flatness (RMS about fitted plane) | 10.8-11.9 mm - factory depth calibration OK |
| Camera mount height | correct to within 6 mm |
| Camera mount tilt | CAD 15 deg wrong by 4.4 deg -> corrected to 10.60 deg |
| Mount correction verified | plane tilt 4.83 deg -> **0.29 deg**; at 2 m, 168 mm -> 10 mm |
| Floor levelness (measured incidentally) | 0.25 deg |
| Host test suites | 4 suites, 118 checks, all pass |

---

### Calibration: when to stop

The wheel-odometry calibration is **complete and deliberately stopped**, not
abandoned. Three settings were measured:

| asymmetry applied | odometry heading | odometry lateral |
|---|---|---|
| 0.000% | +0.25° | +5.9 mm |
| 0.374% | −0.65° | −20.9 mm |
| **0.266%** | **−0.09°** | **−1.2 mm** |

A linear fit predicted −0.39° at the final setting; it measured −0.09°. That
0.30° gap is the run-to-run noise of the method, and a tape is only good to
±0.27° on heading, so another run would resample noise rather than settle
anything.

**Residual bound: wheel-odometry heading bias ≤ ~0.15°/m.**

If that ever needs improving, the right instrument is already on the robot:
RTAB-Map visual odometry over a long straight is far more precise than a tape and
can be differenced against wheel odometry directly. Revisit it there, not with
more tape work.

## 7. Milestones

| ID | Task | Status |
|---|---|---|
| M0.T1 | System prep: groups, udev, rosdep | ✅ |
| M0.T2 | ROS stack + AVR toolchain + arduino-cli | ✅ |
| M0.T3 | Repair description, verify TF tree | ✅ |
| M1.T1 | MODBUS driver + per-wheel sequencing | ✅ |
| M1.T1b | Bench test: motors, encoders, link | ✅ |
| M1.T1c | Wheel direction / inversion | ✅ |
| M1.T2 | Binary Jetson↔Mega protocol, watchdog | ✅ |
| M1.T3 | ros2_control `SystemInterface` plugin | ✅ |
| M1.T4 | `diff_drive_controller`, odometry calibration | ✅ |
| M2.T1 | D435i bringup, stream verification, mount calibration | ✅ |
| M2.T2 | IMU filtering, camera extrinsics | ⬜ |
| M2.T3 | robot_localization EKF | ⬜ |
| M3.T1 | RTAB-Map RGB-D odometry | ⬜ |
| M3.T2 | Mapping, loop closure, 2D grid | ⬜ |
| M3.T3 | Map save/load, localization mode | ⬜ |
| M4.T1 | Nav2 bringup and tuning | ⬜ |
| M4.T2 | Depth costmap layers, blind zone | ⬜ |
| M4.T3 | Point navigation, waypoints | ⬜ |
| M4.T4 | Obstacle avoidance, recoveries | ⬜ |

---

### RESOLVED — the camera is not at 15 deg to the mast

Initially this looked like a body pitch: the camera is *designed* at 15 deg but
measured 10.6 deg to the ground, and a nose-up body would explain the gap. The
user then confirmed the floor is flat and the robot sits at 0 deg pitch, which
rules that out - so the remaining explanation is that the camera simply is not at
15 deg to the mast.

Settled by a direct A/B test rather than by reasoning about sign conventions -
set the URDF to each candidate and see which makes the depth-measured floor come
out flat:

| URDF `camera_pitch` | residual | verdict |
|---|---|---|
| 15.00 deg | +4.24 deg | floor reads as a 4.2 deg ramp |
| 10.60 deg | −0.21 deg | close |
| **10.78 deg** | **+0.06 deg** | **flat — correct** |

Two checks that this is real rather than an artefact:
* the residuals track the URDF setting **one-for-one** (4.44 deg of residual
  change for a 4.40 deg setting change), so the measurement is linear;
* solving for the true angle from the 10.60 and 15.00 runs *independently* gives
  10.81 and 10.76 deg — they agree.

**Final: `camera_pitch` = 10.78 deg, plus 1.06 deg of roll the design lacks.**
The user confirms this is the robot's normal, as-built state, so the URDF now
describes the hardware rather than the design intent.

Confirmed end to end: TF gives an optical Z of (0.982, 0.000, -0.187) in
base_footprint = 10.78 deg below horizontal, and the ground plane comes back flat
to **0.04-0.13 deg** at **0.8-5.5 mm**.

The **visual mesh is counter-rotated by -4.22 deg** to match. The STL has the
design's 15 deg baked in, so without that the RViz model would draw the camera
pointing 4.2 deg away from its own TF axes - a discrepancy that looks exactly
like a TF bug. Cosmetic only: collisions use `camera_pitch` and the frames come
from `camera_link`.

### SUPERSEDED — was the robot body pitched ~4.4 deg nose-up?

The camera is mechanically at **15 deg to the mast** (confirmed by hand on the
robot) but measures **10.6 deg to the ground**. Both can be true only if the body
itself is pitched nose-up by the 4.4 deg difference. The accelerometer agrees:
gravity sits 1.55 deg off a 10.6 deg model (and ~1.5 deg of that is explainable
as accel bias) versus ~4.4 deg off a 15 deg model.

If real, a 4.4 deg body pitch means **32 mm of height difference across the
420 mm base**, and **52 mm of lean at the top of the mast** - both visible to the
eye.

**To check:** put a spirit level or a phone inclinometer on the base plate.

**Why it matters beyond the camera angle:** `base_footprint` is defined as the
ground-contact frame, so a real body pitch belongs in
`base_footprint -> base_link`. Today it is folded into `camera_pitch` instead,
which gets the camera's *orientation* right but leaves its *X position* about
65 mm out - an error a ground-plane fit is structurally blind to, because
sliding a plane along itself changes nothing.

**Candidate causes:** caster height differing from the 50 mm wheel radius (the
robot is statically over-determined - two wheels at x=0 plus four casters at
x=+/-0.17 - so it rests on the wheels and one caster pair), or a mast that is not
square to the base.

## 8. Open risks

| Risk | Detail | Status |
|---|---|---|
| **No kill switch** | Only removing the battery. The comms watchdog now covers a dead Jetson (measured: stops within 34 mm at 10 rpm), but it cannot cover a Mega hang — only cutting motor power can. An inline e-stop in the motor supply is still recommended. | open |
| **Battery below drive minimum** | Drive spec is 10–30 V. A 3S pack sags to ~9.9 V when flat — *below* minimum. Needs a low-voltage cutoff at ~10.5 V (3.5 V/cell), which also protects the pack. | open |
| **Shared power rail** | Motors and Jetson share the pack; motor inrush could brown out the Jetson. | watch |
| **13 V from a 3S pack** | 3S is 11.1 V nominal / 12.6 V full, so 13 V needs a boost stage, not a buck. | clarify |
| **5 V / 3.3 V UART margin** | Drive UART is 3.3 V TTL, Mega is 5 V. The drive's TXD only just clears the ATmega's 3.0 V threshold. No errors so far (`crc=0`), but suspect this first if CRC errors appear. | monitored |
| **Camera blind zone** | Camera at 0.844 m pitched 15° down cannot see close, low obstacles. Needs explicit handling in the Nav2 costmap. | M4.T2 |
| **Modbus rate ceiling** | 9600 baud is fixed in drive firmware; ~27 Hz position updates per wheel. Adequate for a 0.31 m/s robot but it is the hard limit. | accepted |

---

## 9. Conventions

- **Frames** follow REP-103/REP-105: `map` → `odom` → `base_footprint` → `base_link`.
  Optical frames use the camera convention (Z out of the lens).
- **Units** are SI everywhere in ROS. The firmware works in encoder counts and
  base-motor rpm; conversion happens at the ros2_control boundary.
- **Wheel sign convention:** positive = that wheel drives the robot **forward**,
  on both sides. Physical mirroring is hidden inside `WheelDrive`.
- **`odom` → `base_footprint` has exactly one publisher.** Today it is
  `diff_drive_controller`; when the EKF arrives in M2.T3, set
  `enable_odom_tf: false` or the two will fight.
- **Never flash firmware whose host tests fail** — `flash.sh` enforces this.
- **Nothing in `read()`/`write()` may block.** They run in the controller_manager
  RT thread; a blocking serial write would stall every controller on the robot.
- **The wire format is defined once**, in `firmware/labrobot_firmware/labrobot_protocol.h`,
  and compiled into both the Arduino sketch and the ros2_control plugin.
