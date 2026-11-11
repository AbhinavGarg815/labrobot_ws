#!/usr/bin/env bash
# Bring up the robot, localise it automatically, then Nav2 - in the background.
#
#     scripts/bringup_nav.sh                  # lifelong SLAM: map updates as it drives
#     scripts/bringup_nav.sh --localization   # map frozen, localise only
#
# Both start RTAB-Map in LOCALISATION mode so only the saved map competes while
# auto_localise spins the robot to find its pose. In the default mode
# auto_localise then switches RTAB-Map to mapping (see auto_localise.py for why
# starting in SLAM mode does not work).
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
MODE_ARG="localization:=true"
THEN_MAPPING="--then-mapping"
AUTOLOC=1
while [ $# -gt 0 ]; do
    case "$1" in
        --localization) THEN_MAPPING=""; shift ;;
        --no-autoloc)   AUTOLOC=0; shift ;;     # bring up without the startup spin
        *) break ;;
    esac
done
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
    if grep -qE "Managed nodes are active" "$NAV_LOG"; then
        # Startup localisation: spins in place until the pose is trusted, then
        # stays up publishing /labrobot/localised (nav_goal.py requires it).
        MAPMODE=$([ -n "$THEN_MAPPING" ] && echo lifelong || echo frozen)   # read by scripts/localise.sh
        if [ "$AUTOLOC" = "0" ]; then
            status "READY mode=$MAPMODE robot=$ROBOT_LOG nav=$NAV_LOG (no auto-localise)"; exit 0
        fi
        LOC_LOG=docs/logs/nav_${STAMP}_autoloc.log
        # Exactly one auto_localise: a leftover instance keeps publishing its
        # own latched verdict alongside the new one.
        pkill -TERM -f "lib/labrobot_bringup/auto_[l]ocalise" 2>/dev/null || true
        sleep 1
        # Node ids below this run's FIRST node are the saved map.
        FIRST_ID=$(grep -m1 -oE "rtabmap \([0-9]+\):" "$ROBOT_LOG" | grep -oE "[0-9]+")
        setsid ros2 run labrobot_bringup auto_localise.py ${FIRST_ID:+--saved-below $FIRST_ID} $THEN_MAPPING \
            > "$LOC_LOG" 2>&1 < /dev/null &
        status "READY mode=$MAPMODE robot=$ROBOT_LOG nav=$NAV_LOG autoloc=$LOC_LOG (robot spins to localise)"
        exit 0
    fi
    grep -qE "Aborting bringup" "$NAV_LOG" && { status "FAILED: nav2 bringup aborted"; exit 1; }
    sleep 2
done
status "FAILED: nav2 did not become active"; exit 1
