#!/usr/bin/env bash
# Bring up the robot with SLAM running, then Nav2, in the background.
#
#     scripts/bringup_nav.sh                  # lifelong SLAM: map updates as it drives
#     scripts/bringup_nav.sh --localization   # map frozen, localise only
#
# Lifelong SLAM is the default (user decision 2026-10-13: "update map in every
# navigation"). The lab is rearranged often; a frozen map keeps moved furniture
# as phantom obstacles and stops matching the room, which is what made
# relocalisation fail after objects moved. Mapping while navigating keeps both
# current. Costs: the database grows ~10 MB per metre driven, and a false loop
# closure could bend the map - back up the database before long sessions.
#
# Waits for each stage on its log rather than on a fixed sleep, and writes a
# one-line status file (docs/logs/bringup_status) so progress can be checked in
# short steps. Exists because long blocking bringup commands kept being cut off
# mid-launch, leaving the stack half-started with no record of how far it got.
set -uo pipefail
cd "$(dirname "$0")/.."
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
STAMP=$(date +%H%M)
ROBOT_LOG=docs/logs/nav_${STAMP}.log
NAV_LOG=docs/logs/nav_${STAMP}_nav2.log
STATUS=docs/logs/bringup_status
MODE_ARG="slam:=true"
if [ "${1:-}" = "--localization" ]; then MODE_ARG="localization:=true"; shift; fi
status() { echo "$(date +%T) $*" > "$STATUS"; echo "$(date +%T) $*"; }

bash scripts/stop_robot.sh >/dev/null 2>&1
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
status "starting robot ($MODE_ARG) -> $ROBOT_LOG"
setsid ros2 launch labrobot_bringup robot.launch.py "$MODE_ARG" "$@" \
    > "$ROBOT_LOG" 2>&1 < /dev/null &
for _ in $(seq 1 60); do
    grep -qE "rtabmap \([0-9]+\):" "$ROBOT_LOG" && break
    grep -qE "rtabmap-10\]: process has died|FATAL" "$ROBOT_LOG" && { status "FAILED: rtabmap died"; exit 1; }
    sleep 3
done
grep -qE "rtabmap \([0-9]+\):" "$ROBOT_LOG" || { status "FAILED: rtabmap never started"; exit 1; }
status "rtabmap up; starting nav2 -> $NAV_LOG"
setsid ros2 launch labrobot_bringup navigation.launch.py > "$NAV_LOG" 2>&1 < /dev/null &
for _ in $(seq 1 60); do
    grep -qE "Managed nodes are active" "$NAV_LOG" && { status "READY robot=$ROBOT_LOG nav=$NAV_LOG"; exit 0; }
    grep -qE "Aborting bringup" "$NAV_LOG" && { status "FAILED: nav2 bringup aborted"; exit 1; }
    sleep 2
done
status "FAILED: nav2 did not become active"; exit 1
