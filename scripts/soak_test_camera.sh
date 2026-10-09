#!/usr/bin/env bash
# Rigorous D435i link soak test. Run after any cable or port change.
#
#     scripts/soak_test_camera.sh [total_minutes]     (default 6)
#
# The time is split evenly between two phases, because the two questions need
# different instruments and the probe needs exclusive access to the camera:
#
#   PHASE 1  LINK HEALTH - librealsense directly, no ROS. Counts dropped frames
#            from the FRAME NUMBER, so a frame the link lost leaves a hole in the
#            sequence that no host CPU load can fake or hide. This phase decides
#            PASS/FAIL.
#
#   PHASE 2  PIPELINE HEALTH - the ROS topics the robot will actually consume.
#            Reported, and a shortfall WARNS rather than fails.
#
# Why the split, learned the hard way
# -----------------------------------
# The first version judged the cable by the frame rate on the ROS depth topic.
# That number measured 25.9, 29.2 and 29.5 fps across three consecutive runs on
# identical hardware, because it is a property of the ROS node, DDS and Jetson
# scheduling - three layers above the link. It produced a confident FAIL on a
# cable that was delivering 29.97 fps with ZERO dropped frames when asked
# directly. A rate shortfall on a ROS topic is evidence about the host, not the
# cable, and the tool now says so instead of conflating them.
#
# What is measured, and why each one earns its place
# --------------------------------------------------
#   1. LINK SPEED      - a D435i on USB 2 does not fail, it silently delivers a
#                        third of the frame rate. Checked first; aborts if wrong.
#   2. DROPPED FRAMES  - from frame numbers, at the source. The real question.
#   3. RE-ENUMERATIONS - the USB device number changing means the link dropped
#                        and came back. This is the fault that killed mapping
#                        runs, and it is invisible to a short test.
#   4. KERNEL RESETS   - dmesg USB resets for this device over the run.
#   5. TF LIVENESS     - the nastiest observed failure: the node stays alive and
#                        keeps streaming while its latched TF stops reaching new
#                        subscribers. `pgrep` says healthy; every consumer breaks.

set -eo pipefail

TOTAL_MIN="${1:-6}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
PHASE_MIN=$(awk -v m="$TOTAL_MIN" 'BEGIN{printf "%.2f", m/2}')

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

RSUSB="$HOME/opt/librealsense-rsusb"
PROBE_SRC="$WS/tools/rs_link_probe.cpp"
PROBE_BIN="$WS/build/rs_link_probe"

LOG=$(mktemp)
DEVWATCH=$(mktemp)
WATCH_PID=""
cleanup() {
    [ -n "$WATCH_PID" ] && kill "$WATCH_PID" 2>/dev/null || true
    pkill -f "realsense2[_]camera_node" 2>/dev/null || true
    sleep 1
    rm -f "$LOG" "$DEVWATCH"
}
trap cleanup EXIT

echo "=========================================================================="
echo " D435i LINK SOAK TEST - ${PHASE_MIN} min per phase, ${TOTAL_MIN} min total"
echo "=========================================================================="
echo

# --- 0. link speed, before anything else ----------------------------------
echo "--- 0. link speed ---"
"$HERE/check_camera_usb.sh" || true

# Read the speed with an explicit loop, NOT `for ... done | head -1` in a command
# substitution. Under `set -e` with pipefail that construct takes the exit status
# of the LAST loop iteration - and the last entry in the glob is never the
# RealSense - so the assignment "failed" and killed the script one line before
# the guard below could run. The abort never printed, and the resulting exit 1
# looked exactly like the guard firing.
SPEED=""
USBPATH=""
for d in /sys/bus/usb/devices/*/; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "8086" ] || continue
    SPEED=$(cat "$d/speed" 2>/dev/null || echo "")
    USBPATH=$(basename "$d")
    break
done

# Allow-list rather than blocking 480 specifically: an empty or unexpected speed
# means we could not establish the link is good, and soaking on that assumption
# is how a cable fault gets misfiled as a SLAM bug.
case "$SPEED" in
    5000|10000) ;;
    *)
        echo
        echo "ABORTING: link speed is '${SPEED:-unknown}', not USB 3."
        echo "Fix the cable first - soaking a degraded link only measures how"
        echo "consistently it underperforms."
        exit 1
        ;;
esac
echo

# --- build the link probe --------------------------------------------------
if [ ! -x "$PROBE_BIN" ] || [ "$PROBE_SRC" -nt "$PROBE_BIN" ]; then
    echo "--- building rs_link_probe ---"
    mkdir -p "$(dirname "$PROBE_BIN")"
    g++ -O2 -std=c++14 "$PROBE_SRC" -o "$PROBE_BIN" \
        -I"$RSUSB/include" -L"$RSUSB/lib" -lrealsense2 -Wl,-rpath,"$RSUSB/lib"
    echo "built $PROBE_BIN"
    echo
fi

# --- re-enumeration watcher, running across both phases --------------------
# Polls sysfs (a single cheap file read) rather than shelling out to lsusb, so it
# cannot perturb what it is measuring.
: > "$DEVWATCH"
(
    last=""
    while true; do
        cur=""
        for d in /sys/bus/usb/devices/*/; do
            [ -f "$d/idVendor" ] || continue
            [ "$(cat "$d/idVendor" 2>/dev/null)" = "8086" ] || continue
            cur=$(cat "$d/devnum" 2>/dev/null || echo "")
            break
        done
        if [ -n "$cur" ] && [ -n "$last" ] && [ "$cur" != "$last" ]; then
            echo "$(date +%s) devnum $last -> $cur" >> "$DEVWATCH"
        fi
        [ -n "$cur" ] && last="$cur"
        sleep 1
    done
) &
WATCH_PID=$!

DMESG_BEFORE=$(dmesg 2>/dev/null | grep -ci "usb ${USBPATH}.*reset" || true)

# ==========================================================================
# PHASE 1 - link health, measured at the link
# ==========================================================================
echo "--- 1. LINK HEALTH (librealsense direct, no ROS) ---"
pkill -f "realsense2[_]camera_node" 2>/dev/null || true
sleep 3

LINK_OK=0
PHASE_SEC=$(awk -v m="$PHASE_MIN" 'BEGIN{printf "%d", m*60}')
if "$PROBE_BIN" "$PHASE_SEC" 1; then
    LINK_OK=1
fi
echo

# ==========================================================================
# PHASE 2 - pipeline health, through ROS
# ==========================================================================
echo "--- 2. PIPELINE HEALTH (ROS topics the robot actually consumes) ---"
ros2 launch labrobot_bringup camera.launch.py > "$LOG" 2>&1 &
sleep 22

if ! pgrep -f "realsense2[_]camera_node" >/dev/null; then
    echo "camera node failed to start:"; tail -15 "$LOG"; exit 1
fi

set +e
python3 - "$PHASE_MIN" "$LOG" <<'PY'
import os, sys, threading, time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu
import tf2_ros

minutes, logpath = float(sys.argv[1]), sys.argv[2]
WARMUP = 5.0

# aligned_depth is what RTAB-Map RGB-D consumes; depth_raw is the unaligned
# stream. Both are measured because a shortfall in one and not the other points
# straight at the alignment stage rather than at the camera.
TOPICS = [
    ('depth_raw',   Image, '/camera/camera/depth/image_rect_raw', 30.0),
    ('depth_align', Image, '/camera/camera/aligned_depth_to_color/image_raw', 30.0),
    ('colour',      Image, '/camera/camera/color/image_raw', 30.0),
    ('imu',         Imu,   '/camera/camera/imu', 200.0),
]

rclpy.init()
n = Node('soak')
qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT)

# Record the header stamp AND the arrival time. They answer different questions:
#   header stamp -> when the CAMERA captured it.
#   arrival      -> when THIS process got round to it. Gaps here are ROS
#                   scheduling on a loaded Jetson and say nothing about the link.
rec = {name: [] for name, _, _, _ in TOPICS}

def mk(key):
    def cb(m):
        s = m.header.stamp
        rec[key].append((s.sec + s.nanosec * 1e-9, time.time()))
    return cb

for name, typ, topic, _ in TOPICS:
    n.create_subscription(typ, topic, mk(name), qos)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)

# Spin on a dedicated thread so the periodic TF checks below can never stall
# message draining - an earlier version did exactly that and reported its own
# blocking as camera stalls.
ex = SingleThreadedExecutor(); ex.add_node(n)
threading.Thread(target=ex.spin, daemon=True).start()

def node_pid():
    for p in os.listdir('/proc'):
        if not p.isdigit():
            continue
        try:
            with open(f'/proc/{p}/cmdline', 'rb') as f:
                if b'realsense2_camera_node' in f.read():
                    return int(p)
        except OSError:
            pass
    return None

def cpu_ticks(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            parts = f.read().split()
        return int(parts[13]) + int(parts[14])
    except (OSError, IndexError, ValueError):
        return None

pid = node_pid()
c0 = cpu_ticks(pid) if pid else None
tf_fail, checks = 0, 0
t0 = time.time()
end = t0 + minutes * 60

print(f'  sampling {minutes:.1f} min...')
while time.time() < end:
    time.sleep(min(10.0, max(0.0, end - time.time())))
    if time.time() >= end:
        break
    checks += 1
    try:
        buf.lookup_transform('camera_link', 'camera_depth_optical_frame',
                             rclpy.time.Time())
    except Exception:
        tf_fail += 1
        print(f'  [{time.time()-t0:6.0f}s] *** TF LOST ***')

c1 = cpu_ticks(pid) if pid else None
el = time.time() - t0

print()
rate_short = False
for name, _, _, want in TOPICS:
    r = [x for x in rec[name] if x[1] - t0 > WARMUP]
    if len(r) < 10:
        print(f'  {name:<13}  NO DATA'); rate_short = True; continue
    hdr = [x[0] for x in r]
    span = hdr[-1] - hdr[0]
    rate = (len(hdr) - 1) / span if span > 0 else 0.0
    gaps = [hdr[i+1] - hdr[i] for i in range(len(hdr)-1)]
    stalls = sum(1 for g in gaps if g > 5.0 / want)
    print(f'  {name:<13}{rate:7.2f} of {want:5.0f} Hz  ({100*rate/want:3.0f}%)'
          f'   worst gap {max(gaps)*1000:7.1f} ms   stalls {stalls}')
    if rate < want * 0.9:
        rate_short = True

if c0 is not None and c1 is not None:
    hz = os.sysconf('SC_CLK_TCK')
    print(f'\n  realsense node CPU : {100.0*(c1-c0)/hz/el:.1f}% of one core '
          f'({100.0*(c1-c0)/hz/el/os.cpu_count():.1f}% of the machine)')

print(f'  TF losses          : {tf_fail} of {checks} checks')
try:
    with open(logpath) as f:
        errs = sum(1 for l in f if 'control_transfer' in l)
    print(f'  USB ctrl errors    : {errs}  ({errs/(el/60):.1f}/min; '
          f'old cable: 8-13 per startup)')
except OSError:
    pass

# os._exit, not sys.exit: the executor spins on a daemon thread, and letting the
# interpreter finalise underneath it aborts with "could not acquire lock for
# <stderr> at interpreter shutdown" - a core dump AFTER the verdict has printed,
# which turns a clean result into an alarming one.
sys.stdout.flush()
os._exit((2 if rate_short else 0) | (1 if tf_fail else 0))
PY
PIPE_RC=$?
set -e

pkill -f "realsense2[_]camera_node" 2>/dev/null || true
kill "$WATCH_PID" 2>/dev/null || true
WATCH_PID=""
sleep 1

REENUM=$(wc -l < "$DEVWATCH" | tr -d ' ')
DMESG_AFTER=$(dmesg 2>/dev/null | grep -ci "usb ${USBPATH}.*reset" || true)
RESETS=$(( DMESG_AFTER - DMESG_BEFORE ))

# ==========================================================================
echo
echo "=========================================================================="
echo "VERDICT"
echo "=========================================================================="
echo "  link speed        : ${SPEED} Mbps at ${USBPATH}"
echo "  frame loss        : $([ "$LINK_OK" -eq 1 ] && echo 'clean (phase 1)' || echo 'DEGRADED (phase 1)')"
echo "  re-enumerations   : ${REENUM}      (old cable: ~7 per session)"
if [ -s "$DEVWATCH" ]; then cat "$DEVWATCH" | sed 's/^/      /'; fi
echo "  kernel USB resets : ${RESETS}"
echo

TF_LOST=$(( PIPE_RC & 1 ))
RATE_SHORT=$(( (PIPE_RC & 2) >> 1 ))

if [ "$LINK_OK" -eq 1 ] && [ "$REENUM" -eq 0 ] && [ "$RESETS" -eq 0 ] && [ "$TF_LOST" -eq 0 ]; then
    if [ "$RATE_SHORT" -eq 1 ]; then
        echo "  PASS (with a warning)"
        echo
        echo "  The LINK is clean: every frame the camera captured arrived, no"
        echo "  re-enumerations, no kernel resets. The cable is good."
        echo
        echo "  A ROS topic was below 90% of its configured rate. That is a HOST"
        echo "  problem - node CPU, DDS or scheduling - not the cable, and it is"
        echo "  known to vary run to run on this Jetson. Worth watching during"
        echo "  RTAB-Map bring-up; not a reason to keep swapping cables."
    else
        echo "  PASS - link clean and the ROS pipeline is at full rate. Clear for M3."
    fi
    exit 0
fi

echo "  FAIL - do not start mapping on this link; a mid-run dropout will look"
echo "         like a SLAM failure and cost hours to trace back to hardware."
exit 1
