#!/usr/bin/env bash
# Person following - the operator command.
#
#   scripts/follow.sh                     # follow (the robot MOVES after "start")
#   scripts/follow.sh --dry-run           # goals and plans only; the robot does NOT move
#   scripts/follow.sh --tracker-only      # perception only (debug image, markers)
#   scripts/follow.sh --max-speed 0.10    # raise the follow speed cap (default 0.06)
#
# Then, in Foxglove, Publish panel on /labrobot/follow_cmd (std_msgs/String):
#   {"data": "start"}   lock the nearest person ahead (1-3.5 m, +/-20 deg) and follow
#   {"data": "stop"}    stop now (the person is remembered; "start" resumes)
#   {"data": "release"} forget the person
# Status: /labrobot/follow_status. Image: /labrobot/follow_debug/compressed.
# Stop: "stop" in Foxglove, Ctrl-C here, or scripts/stop_robot.sh.
#
# Needs the stack up: scripts/bringup_nav.sh --localization.
set -euo pipefail
cd "$(dirname "$0")/.."

DRY=false; TRACKER_ONLY=false; MAXV=0.06
DET=yolo26s-pose; REID=osnet_ain_x1_0_msmt17
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) DRY=true ;;
        --tracker-only) TRACKER_ONLY=true ;;
        --max-speed) MAXV="$2"; shift ;;
        --detector) DET="$2"; shift ;;
        --reid) REID="$2"; shift ;;
        -h|--help) sed -n 2,17p "$0"; exit 0 ;;
        *) echo "unknown option $1 (see --help)"; exit 1 ;;
    esac
    shift
done

if ! python3 -c "import sys; v=float('$MAXV'); sys.exit(0 if 0.0 < v <= 0.26 else 1)"; then
    echo "--max-speed must be in (0, 0.26] m/s (the controller ceiling)"; exit 1
fi
# READY in bringup_status is not enough: the file survives a reboot (it said
# READY from the day before on 2026-11-12, with nothing running). It must be
# newer than this boot, and Nav2 must actually be running.
BOOT=$(( $(date +%s) - $(cut -d. -f1 /proc/uptime) ))
if ! grep -q READY docs/logs/bringup_status 2>/dev/null \
        || [[ $(stat -c %Y docs/logs/bringup_status) -lt $BOOT ]] \
        || ! pgrep -x bt_navigator >/dev/null; then
    echo "The stack is not up (no READY since this boot, or bt_navigator not running)."
    echo "Start it first: scripts/bringup_nav.sh --localization"; exit 1
fi
for f in ~/labrobot_models/engines/${DET}_480x640_fp16.engine ~/labrobot_models/engines/${REID}_fp16.engine; do
    [[ -f "$f" ]] || { echo "missing $f - build it: tools/build_follow_models.sh --engines-only ${DET} ${REID}"; exit 1; }
done

# ROS's setup files read variables that may be unset: not under set -u.
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
LOG=docs/logs/follow_$(date +%H%M).log
mkdir -p docs/logs

if [[ "$TRACKER_ONLY" == true ]]; then
    echo "Perception only - the robot will not move."
elif [[ "$DRY" == true ]]; then
    echo "DRY RUN - goals and plans are published; the robot will NOT move."
else
    echo "*** The robot WILL MOVE after \"start\" on /labrobot/follow_cmd (speed cap $MAXV m/s). ***"
    echo "*** Stop: \"stop\" in Foxglove, Ctrl-C here, or scripts/stop_robot.sh.                 ***"
fi
echo "log: $LOG"
exec ros2 launch labrobot_follow follow.launch.py detector:="$DET" reid:="$REID" \
    dry_run:="$DRY" tracker_only:="$TRACKER_ONLY" max_speed:="$MAXV" 2>&1 | tee "$LOG"
