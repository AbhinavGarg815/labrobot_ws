#!/usr/bin/env bash
# Record what person following needs, for offline tests.
#   scripts/record_people.sh NAME [SECONDS]     # default 90 s
set -euo pipefail
NAME="${1:?usage: record_people.sh NAME [SECONDS]}"
SECS="${2:-90}"
OUT=~/labrobot_bags/"$NAME"
[[ -e "$OUT" ]] && { echo "$OUT exists - pick another name"; exit 1; }
mkdir -p ~/labrobot_bags
# ROS's setup files read variables that may be unset: not under set -u.
set +u
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u
echo "recording $SECS s to $OUT (Ctrl-C stops early, the bag is kept)"
timeout --foreground --signal=INT "$SECS" ros2 bag record -o "$OUT" \
    /camera/camera/color/image_raw \
    /camera/camera/aligned_depth_to_color/image_raw \
    /camera/camera/color/camera_info \
    /tf /tf_static /odometry/filtered || true
ros2 bag info "$OUT" | grep -E "Duration|Messages|Topic:" || true
