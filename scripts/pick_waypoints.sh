#!/usr/bin/env bash
# Save waypoints by clicking them in Foxglove - the robot does NOT move.
#
#     scripts/pick_waypoints.sh a        # saves a1, a2, a3, ... until Ctrl-C
#
# In Foxglove's 3D panel use "Publish pose" with the topic set to
# /waypoint_pick (NOT /goal_pose - Nav2 listens there and would drive).
# Click the spot, drag for the direction the robot should face. Each click is
# converted into the map frame and saved to docs/maps/waypoints.yaml.
# Numbering continues after the highest existing number for that prefix, so a
# restart never overwrites waypoints already saved. Check them with
# tools/check_waypoints.py.
set -uo pipefail
cd "$(dirname "$0")/.."
PREFIX=${1:?usage: scripts/pick_waypoints.sh PREFIX   (e.g. a -> a1, a2, ...)}
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
LAST=$(grep -oE "^  ${PREFIX}[0-9]+:" docs/maps/waypoints.yaml 2>/dev/null | grep -oE "[0-9]+" | sort -n | tail -1)
START=$(( ${LAST:-0} + 1 ))
echo "  listening on /waypoint_pick - next is ${PREFIX}${START}; click in Foxglove; Ctrl-C to stop"
for i in $(seq "$START" $(( START + 98 ))); do
    ros2 run labrobot_bringup waypoints.py pick "${PREFIX}${i}" --force --timeout 3600 2>&1 | grep -v RTPS
done
