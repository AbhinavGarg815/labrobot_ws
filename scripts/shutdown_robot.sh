#!/usr/bin/env bash
# Switch the robot off SAFELY: stop every node cleanly (RTAB-Map writes its map
# and the known-good copy), flush all disk writes, then power off the Jetson.
#
#     scripts/shutdown_robot.sh
#
# Switching off at the battery while RTAB-Map is mapping has cost two maps:
# one SQLite-corrupted (fixed by DbSqlite3 settings), one left logically
# inconsistent because RTAB-Map writes its vocabulary only on a clean close.
# Wait for "power off" below, THEN disconnect the battery.
set -u
cd "$(dirname "$0")/.."
bash scripts/stop_robot.sh
sync
echo "  disk flushed. Powering off - disconnect the battery when the Jetson's LEDs go off."
sudo -n poweroff || echo "  could not power off (sudo) - run: sudo poweroff"
