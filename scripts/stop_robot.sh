#!/usr/bin/env bash
# Stop every labrobot node without killing the shell that invoked this.
#   scripts/stop_robot.sh

set -uo pipefail

PATTERNS=(
    # Analysis/monitor scripts.
    monitor_slam
    auto_localise
    waypoints.py
    verify_wheels
    analyse_motion
    publish_cloud
    compare_odom
    # Person following (labrobot_follow).
    person_follower
    person_tracker
    follow.launch
    labrobot_bags           # ros2 bag record started by scripts/record_people.sh
    goal_relay
    controller_server
    planner_server
    behavior_server
    bt_navigator
    velocity_smoother
    lifecycle_manager
    nav2
    rtabmap
    rgbd_odometry
    realsense2_camera_node
    ros2_control_node
    ekf_node
    imu_filter_madgwick
    imu_bias_compensator
    foxglove_bridge
    robot_state_publisher
    spawner
)

if [ "${1:-}" = "--nav" ]; then
    PATTERNS=(controller_server planner_server behavior_server bt_navigator
              velocity_smoother lifecycle_manager navigation.launch waypoint_follower
              collision_monitor goal_relay person_follower)
fi

SELF=$$
PARENT=${PPID:-0}

# ONE filter, used for both killing and counting.
targets() {
    local pat pid cmd
    for pat in "${PATTERNS[@]}"; do
        while read -r pid; do
            [ -z "$pid" ] && continue
            [ "$pid" = "$SELF" ] && continue
            [ "$pid" = "$PARENT" ] && continue
            cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
            [ -z "$cmd" ] && continue
            # Never signal a shell; only the nodes themselves.
            case "$cmd" in
                /bin/bash*|/bin/sh*|bash*|sh*|*shell-snapshots*) continue ;;
            esac
            echo "$pid"
        done < <(pgrep -f "$pat" 2>/dev/null || true)
    done | sort -u
}

# How long to let rtabmap finish writing before escalating to KILL.
RTABMAP_GRACE=${RTABMAP_GRACE:-90}

cmdline_of() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null || true; }

# Record whether rtabmap was actually running BEFORE signalling anything.
rtabmap_was_running=0
while read -r pid; do
    [ -n "$pid" ] || continue
    case "$(cmdline_of "$pid")" in *rtabmap*) rtabmap_was_running=1 ;; esac
done < <(targets)

killed=0
while read -r pid; do
    [ -n "$pid" ] && kill -TERM "$pid" 2>/dev/null && killed=$((killed + 1))
done < <(targets)

# Most nodes are gone within a couple of seconds of SIGTERM.
sleep 6

# rtabmap is the ONE node that must not be killed on a timer.
waited=0
if [ "$rtabmap_was_running" -eq 1 ]; then
    while [ "$waited" -lt "$RTABMAP_GRACE" ]; do
        still=0
        while read -r pid; do
            [ -n "$pid" ] || continue
            case "$(cmdline_of "$pid")" in *rtabmap*) still=1 ;; esac
        done < <(targets)
        [ "$still" -eq 0 ] && break
        [ $((waited % 10)) -eq 0 ] && \
            echo "  waiting for rtabmap to close its database (${waited}s/${RTABMAP_GRACE}s)..."
        sleep 2
        waited=$((waited + 2))
    done
    if [ "$waited" -ge "$RTABMAP_GRACE" ]; then
        echo "  WARNING: rtabmap did not exit in ${RTABMAP_GRACE}s - killing it."
        echo "           The map database may be damaged; check the next launch for"
        echo "           'missing some words' and run rtabmap-recovery if you see it."
    else
        echo "  rtabmap closed its database cleanly after ${waited}s"
        # Keep a copy of the map as it was at this CLEAN close.
        DB="$HOME/.ros/rtabmap.db"
        DRIFT="$(dirname "$0")/../tools/map_drift.py"
        if [ -f "$DB" ]; then
            rc=0
            if [ -f "$DB.lastclean" ] && [ -f "$DRIFT" ]; then
                python3 "$DRIFT" "$DB.lastclean" "$DB" --quiet; rc=$?
            fi
            if [ "$rc" -eq 2 ]; then
                aside="$HOME/ros2_ws/docs/maps/backup/distorted_$(date +%Y%m%d_%H%M%S).db"
                mv "$DB" "$aside" && cp "$DB.lastclean" "$DB" && sync
                echo "  WARNING: this session DISTORTED the saved map - it was set aside as"
                echo "           $aside"
                echo "           and the previous known-good map restored. Saved waypoints stay valid."
            elif [ "$rc" -eq 0 ]; then
                cp "$DB" "$DB.lastclean.tmp" && mv "$DB.lastclean.tmp" "$DB.lastclean" && sync \
                    && echo "  known-good copy saved: $DB.lastclean"
            else
                echo "  WARNING: map drift check failed (exit $rc) - known-good copy NOT updated"
            fi
        fi
    fi
fi

# Anything still alive now gets SIGKILL - nothing left has data at stake.
while read -r pid; do
    [ -n "$pid" ] && kill -KILL "$pid" 2>/dev/null && killed=$((killed + 1))
done < <(targets)

sleep 2
remaining=$(targets | wc -l | tr -d ' ')

echo "  signalled ${killed} process(es)"
if [ "$remaining" -gt 0 ]; then
    echo "  WARNING: ${remaining} still matching - inspect with: pgrep -af rtabmap"
else
    echo "  all labrobot nodes stopped"
fi
