#!/usr/bin/env bash
# Record what person following needs, for offline tests. The robot does NOT move.
#
#   scripts/record_people.sh NAME [SECONDS]     # default 90 s
#
# Writes ~/labrobot_bags/NAME/ (rosbag2, sqlite3): colour (raw rgb8), depth
# aligned to colour (raw 16UC1), colour camera_info, /tf, /tf_static and the
# EKF odometry - everything person_tracker.py consumes, so it can be replayed
# with `ros2 bag play --clock` and use_sim_time.
#
# Size: ~23 MB/s (raw images at 15 Hz), so 90 s is ~2 GB. Bags are not in git.
# Needs the stack running (scripts/bringup_nav.sh).
set -euo pipefail
NAME="${1:?usage: record_people.sh NAME [SECONDS]}"
SECS="${2:-90}"
OUT=~/labrobot_bags/"$NAME"
[[ -e "$OUT" ]] && { echo "$OUT exists - pick another name"; exit 1; }
mkdir -p ~/labrobot_bags
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
echo "recording $SECS s to $OUT (Ctrl-C stops early, the bag is kept)"
timeout --signal=INT "$SECS" ros2 bag record -o "$OUT" \
    /camera/camera/color/image_raw \
    /camera/camera/aligned_depth_to_color/image_raw \
    /camera/camera/color/camera_info \
    /tf /tf_static /odometry/filtered || true
ros2 bag info "$OUT" | grep -E "Duration|Messages|Topic:" || true
