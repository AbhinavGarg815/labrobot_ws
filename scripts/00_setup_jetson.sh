#!/usr/bin/env bash
#
# Milestone 0 / Task 1+2 - one-time host provisioning for the labrobot Jetson.
#
# Installs the ROS 2 packages the robot stack needs, grants the login user
# access to the Arduino Mega's serial port, and pins that port to a stable
# device name so the ros2_control hardware interface never has to guess
# between /dev/ttyACM0 and /dev/ttyACM1.
#
# Idempotent: safe to re-run. Run as root:
#
#     sudo bash /home/zine/ros2_ws/scripts/00_setup_jetson.sh
#
set -euo pipefail

# The user that will run the robot stack. Derived from sudo, so the script
# grants dialout to the human who invoked it rather than to root.
TARGET_USER="${SUDO_USER:-${USER}}"

# Stable name for the Arduino Mega, matched on its burned-in USB serial number
# (see `udevadm info -a -n /dev/ttyACM1`). Replacing the board means updating
# MCU_USB_SERIAL below.
MCU_USB_SERIAL="11427551267196908100"
MCU_SYMLINK="labrobot_mcu"
UDEV_RULE_FILE="/etc/udev/rules.d/99-labrobot.rules"

ROS_DISTRO_NAME="humble"

# Package set, grouped by the milestone that first needs it.
ROS_PACKAGES=(
  # M1 - drivetrain: ros2_control framework + diff_drive_controller.
  "ros-${ROS_DISTRO_NAME}-ros2-control"
  "ros-${ROS_DISTRO_NAME}-ros2-controllers"
  # M2 - perception: D435i driver and depth/IMU processing.
  "ros-${ROS_DISTRO_NAME}-realsense2-camera"
  "ros-${ROS_DISTRO_NAME}-realsense2-description"
  "ros-${ROS_DISTRO_NAME}-depth-image-proc"
  "ros-${ROS_DISTRO_NAME}-imu-filter-madgwick"
  # M2 - sensor fusion: EKF over wheel odometry + IMU.
  "ros-${ROS_DISTRO_NAME}-robot-localization"
  # M3 - RGB-D SLAM.
  "ros-${ROS_DISTRO_NAME}-rtabmap-ros"
  # M4 - navigation.
  "ros-${ROS_DISTRO_NAME}-navigation2"
  "ros-${ROS_DISTRO_NAME}-nav2-bringup"
  "ros-${ROS_DISTRO_NAME}-twist-mux"
  # Description / tooling gaps in the current install.
  "ros-${ROS_DISTRO_NAME}-xacro"
  "ros-${ROS_DISTRO_NAME}-joint-state-publisher"
  "ros-${ROS_DISTRO_NAME}-joint-state-publisher-gui"
)

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }

if [[ "${EUID}" -ne 0 ]]; then
  echo "error: must run as root. Try: sudo bash $0" >&2
  exit 1
fi

if [[ -z "${TARGET_USER}" || "${TARGET_USER}" == "root" ]]; then
  echo "error: could not determine the non-root target user." >&2
  echo "       Re-run via sudo from your normal login account." >&2
  exit 1
fi

log "Target user: ${TARGET_USER}"

# ---------------------------------------------------------------------------
# 1. Serial port access.
#
# Ubuntu guards /dev/ttyACM* with group 'dialout'. Without membership the
# hardware interface gets EACCES on open().
# ---------------------------------------------------------------------------
log "Granting '${TARGET_USER}' access to serial ports (dialout group)"
if id -nG "${TARGET_USER}" | tr ' ' '\n' | grep -qx dialout; then
  echo "already a member of dialout - nothing to do"
else
  usermod -aG dialout "${TARGET_USER}"
  echo "added ${TARGET_USER} to dialout (takes effect on next login)"
fi

# ---------------------------------------------------------------------------
# 2. Stable device name for the Arduino Mega.
#
# ACM numbering depends on enumeration order, and the USB-tethered phone also
# claims an ACM node. Matching on the board's serial number makes
# /dev/labrobot_mcu unambiguous.
# ---------------------------------------------------------------------------
log "Installing udev rule -> /dev/${MCU_SYMLINK}"
cat > "${UDEV_RULE_FILE}" <<EOF
# labrobot - Arduino Mega 2560 motor-control MCU.
# Matched on the board's USB serial number so the symlink survives re-plugging
# and never collides with other CDC-ACM devices (e.g. USB-tethered phones).
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", \\
  ATTRS{serial}=="${MCU_USB_SERIAL}", SYMLINK+="${MCU_SYMLINK}", \\
  MODE="0660", GROUP="dialout"
EOF
chmod 0644 "${UDEV_RULE_FILE}"
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
echo "wrote ${UDEV_RULE_FILE}"

# ---------------------------------------------------------------------------
# 3. ROS 2 package installation.
#
# The ROS 2 apt repository is already configured on this host via the
# 'ros2-apt-source' package (/etc/apt/sources.list.d/ros2.sources).
# ---------------------------------------------------------------------------
log "Updating apt indexes"
apt-get update

log "Installing ROS 2 packages (~110 MiB download, 207 packages)"
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  "${ROS_PACKAGES[@]}"

# ---------------------------------------------------------------------------
# 4. rosdep - needed later to resolve dependencies of our own packages.
# ---------------------------------------------------------------------------
log "Initialising rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
else
  echo "rosdep already initialised"
fi
# Update the cache as the target user, not root, so ~/.ros stays user-owned.
sudo -u "${TARGET_USER}" rosdep update || \
  echo "warning: 'rosdep update' failed (network?); re-run it later"

log "Done."
cat <<EOF

Next steps:
  1. Log out and back in (or run 'newgrp dialout') so the group change applies.
  2. Verify with:
       ls -l /dev/${MCU_SYMLINK}
       id -nG | tr ' ' '\\n' | grep dialout
       source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && ros2 pkg prefix rtabmap_slam

EOF
