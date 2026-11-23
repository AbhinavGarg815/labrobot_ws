#!/usr/bin/env bash
# Localise the robot again while the stack is running (after bringup_nav.sh).
#   scripts/localise.sh                  # ROBOT TURNS: spin + scan match + visual check, as at startup
#   scripts/localise.sh --confirm        # you checked the pose in Foxglove: accept it if visual
#   scripts/localise.sh --confirm-geometry   # you checked the pose, but no visual matches come
#   scripts/localise.sh --dry-run ...    # print the command, do nothing
set -euo pipefail
cd "$(dirname "$0")/.."

MODE=auto; DRY=0
for a in "$@"; do
    case "$a" in
        --confirm) MODE=confirm ;;
        --confirm-geometry) MODE=geometry ;;
        --dry-run) DRY=1 ;;
        -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "unknown option: $a (see --help)"; exit 2 ;;
    esac
done

STATUS=docs/logs/bringup_status
[ -f "$STATUS" ] || { echo "no $STATUS - start the robot with scripts/bringup_nav.sh first"; exit 1; }
ROBOT_LOG=$(grep -oE "robot=[^ ]+" "$STATUS" | tail -1 | cut -d= -f2)
[ -n "$ROBOT_LOG" ] && [ -f "$ROBOT_LOG" ] || { echo "bringup did not reach READY (see $STATUS)"; exit 1; }
FIRST_ID=$(grep -m1 -oE "rtabmap \([0-9]+\):" "$ROBOT_LOG" | grep -oE "[0-9]+" || true)
[ -n "$FIRST_ID" ] || { echo "cannot find RTAB-Map's first node id in $ROBOT_LOG"; exit 1; }
# Same map mode as the running bringup (bringup_nav.sh writes mode=frozen or mode=lifelong).
THEN_MAPPING=""
grep -q "mode=lifelong" "$STATUS" && THEN_MAPPING="--then-mapping"

case "$MODE" in
    auto)     EXTRA="" ;;
    confirm)  EXTRA="--accept-current" ;;
    geometry) EXTRA="--accept-current --min-closures 0" ;;
esac
LOG=docs/logs/localise_$(date +%m%d_%H%M%S).log
CMD="ros2 run labrobot_bringup auto_localise.py --saved-below $FIRST_ID $THEN_MAPPING $EXTRA"
echo "  $CMD"
echo "  log: $LOG"
[ "$DRY" = 1 ] && exit 0

set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
pkill -TERM -f "lib/labrobot_bringup/auto_[l]ocalise" 2>/dev/null || true
sleep 2
setsid $CMD > "$LOG" 2>&1 < /dev/null &
# Wait for the verdict (up to ~20 min for the full 4-spot search).
for _ in $(seq 1 1200); do
    if grep -qE "\[LOCALISED\]|NOT localised after|NOT accepted|could not reach|only turned|Traceback" "$LOG"; then
        break
    fi
    sleep 1
done
grep -E "\[LOCALISED\]|\[not localised\]|spot [0-9]: " "$LOG" | tail -4
