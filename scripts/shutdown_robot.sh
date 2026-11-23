#!/usr/bin/env bash
#   scripts/shutdown_robot.sh
set -u
cd "$(dirname "$0")/.."
bash scripts/stop_robot.sh
sync
echo "  disk flushed. Powering off - disconnect the battery when the Jetson's LEDs go off."
sudo -n poweroff || echo "  could not power off (sudo) - run: sudo poweroff"
