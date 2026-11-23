#!/usr/bin/env bash
# Measure the CPU cost and achieved frame rate of a RealSense profile.
#   scripts/measure_camera_cost.sh <depth_wxh> <color_wxh> <fps> <align_depth>
#   scripts/measure_camera_cost.sh 848x480 640x480 30 false

set -eo pipefail

DEPTH="${1:-848x480}"
COLOR="${2:-640x480}"
FPS="${3:-30}"
ALIGN="${4:-false}"
DURATION="${5:-20}"

# ROS setup scripts reference unset variables, so -u must be off while sourcing.
set +u
source /opt/ros/humble/setup.bash
set -u

echo "=== profile: depth $DEPTH, color $COLOR @ ${FPS}fps, align_depth=$ALIGN ==="

LOG=$(mktemp)

cleanup() {
    pkill -f realsense2_camera_node 2>/dev/null || true
    pkill -f "rs_launch.py" 2>/dev/null || true
    for _ in $(seq 20); do
        pgrep -f realsense2_camera_node >/dev/null || break
        sleep 0.5
    done
    rm -f "$LOG"
}
trap cleanup EXIT

# Also clear any leftovers from a previous run BEFORE starting.
pkill -f realsense2_camera_node 2>/dev/null || true
for _ in $(seq 20); do
    pgrep -f realsense2_camera_node >/dev/null || break
    sleep 0.5
done
if pgrep -f realsense2_camera_node >/dev/null; then
    echo "ERROR: a realsense node is still running and will not exit." >&2
    exit 1
fi

ros2 launch realsense2_camera rs_launch.py \
    depth_module.depth_profile:="${DEPTH}x${FPS}" \
    rgb_camera.color_profile:="${COLOR}x${FPS}" \
    enable_depth:=true \
    enable_color:=true \
    enable_infra1:=false \
    enable_infra2:=false \
    align_depth.enable:="$ALIGN" \
    pointcloud.enable:=false \
    > "$LOG" 2>&1 &

sleep 12

echo
echo "--- achieved rates over ${DURATION}s ---"
for topic in /camera/camera/depth/image_rect_raw /camera/camera/color/image_raw; do
    printf "  %-42s " "$(basename $(dirname $topic))/$(basename $topic)"
    rate=$(timeout $((DURATION/2)) ros2 topic hz "$topic" --window 30 2>/dev/null \
             | grep -oE "average rate: [0-9.]+" | tail -1 || true)
    echo "${rate:-NO DATA}"
done

echo
echo "--- CPU (total across all cores; 100% = one core) ---"
total=0; n=0
for _ in $(seq 5); do
    c=$(ps -C realsense2_camera_node -o %cpu= 2>/dev/null | awk '{s+=$1} END {print s+0}')
    total=$(echo "$total + $c" | bc); n=$((n+1))
    sleep 2
done
echo "  realsense2_camera_node: $(echo "scale=1; $total/$n" | bc)%"
echo "  system load average   : $(cut -d' ' -f1-3 /proc/loadavg)"

echo
echo "--- errors in the camera log ---"
n_err=$(grep -icE "error|warn.*drop|timeout" "$LOG" || true)
echo "  $n_err error/warn lines"
if [ "$n_err" -gt 0 ]; then
    grep -iE "error|frame drop" "$LOG" | head -3 || true
    if grep -qi "resource busy" "$LOG"; then
        echo
        echo "  *** RESULT INVALID: the device was already claimed. ***"
    fi
fi
