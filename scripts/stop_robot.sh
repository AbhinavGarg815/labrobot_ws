#!/usr/bin/env bash
# Stop every labrobot node without killing the shell that invoked this.
#
#     scripts/stop_robot.sh
#
# Why this exists rather than a line of pkill
# -------------------------------------------
# `pkill -f <pattern>` matches its own invoking shell whenever that shell's
# command line contains the pattern - which it does, because the pattern is IN
# the command. The result is the shell dying mid-script with exit code 144 and no
# error, having killed some of the targets and not others. This has happened four
# times in this project.
#
# The usual workaround is the bracket form, `realsense2[_]camera_node`, which
# stops the pattern matching itself. But it only helps when that is the ONLY
# occurrence: a script that also runs `ros2 launch labrobot_bringup ...` further
# down has the plain string in its command line too, and dies anyway.
#
# The reliable fix is what this file does: keep the patterns IN A FILE. A shell
# running `stop_robot.sh` has only that path in its cmdline, so it can never
# match its own targets. Self and parent are excluded as a second belt.

set -uo pipefail

PATTERNS=(
    # Analysis/monitor scripts. Added after a pkill -f on one of these matched
    # the invoking shell for the fifth time in this project - keeping every
    # pattern in THIS FILE is the only reliable defence.
    monitor_slam
    auto_localise
    waypoints.py
    verify_wheels
    analyse_motion
    publish_cloud
    compare_odom
    # Nav2. These were missing when this file was first written, so two full sets
    # of Nav2 nodes ended up running at once: duplicate node names, and the
    # lifecycle manager then failed to configure with "Failed to change state for
    # node: controller_server". Anything added to the stack must be added here.
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

# --nav: stop ONLY the Nav2 nodes, leaving camera, drivetrain, EKF, rtabmap
# and auto_localise running (auto_localise holds the latched "localised"
# status that nav_goal.py and waypoints.py require - killing it would force a
# needless re-localisation). Restarting Nav2 alone is the common case (a params change), and
# restarting everything costs a full relocalisation for nothing.
if [ "${1:-}" = "--nav" ]; then
    PATTERNS=(controller_server planner_server behavior_server bt_navigator
              velocity_smoother lifecycle_manager navigation.launch waypoint_follower)
fi

SELF=$$
PARENT=${PPID:-0}

# ONE filter, used for both killing and counting. An earlier version filtered
# only when killing, so the final tally counted this very shell under all six
# patterns and reported "6 still matching" with nothing running at all.
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
# Without this the script reported "rtabmap closed its database cleanly" on a
# machine where rtabmap had never been started - a reassuring message about
# something that did not happen, which is worse than no message.
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
#
# On SIGTERM it flushes working memory into the SQLite database. For a 1.9 GB
# map that is tens of seconds of writing. The previous version of this script
# waited 6 seconds and then sent SIGKILL unconditionally, which cut the write
# short every single time. The damage is not visible at shutdown - it shows up
# on the NEXT launch as:
#
#   Not found word 769631 (dict size=352471)
#   The dictionary is empty or missing some words from nodes in WM
#   Regenerated the dictionary with 5724 missing words
#
# and then, a couple of minutes into the run, a hard abort:
#
#   Memory.cpp:3852::addLink() Condition (fromS->getWeight() >= 0 && ...) not met!
#   terminate called after throwing an instance of 'UException'
#
# which takes map->odom with it, so Nav2 silently has no localisation and every
# goal is rejected with "no map -> base_footprint transform". Three sessions of
# navigation failures traced back to this one `sleep 6`.
#
# So: poll until rtabmap is actually gone, and only then escalate.
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
        # Keep a copy of the map as it was at this CLEAN close. If the robot is
        # later switched off mid-session, RTAB-Map's database can be left
        # logically inconsistent (2026-10-31: "Query (445239) doesn't match
        # loaded words (356548)" - the vocabulary is only fully written on a
        # clean close) and refuse to load. This copy always loads.
        DB="$HOME/.ros/rtabmap.db"
        if [ -f "$DB" ]; then
            cp "$DB" "$DB.lastclean.tmp" && mv "$DB.lastclean.tmp" "$DB.lastclean" && sync \
                && echo "  known-good copy saved: $DB.lastclean"
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
