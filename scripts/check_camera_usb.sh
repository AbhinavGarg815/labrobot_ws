#!/usr/bin/env bash
# Check how the RealSense has negotiated its USB link. Takes about a second.
#
#     scripts/check_camera_usb.sh
#
# Use this after changing a cable or port, BEFORE running anything else. A D435i
# on USB 2 does not fail - it silently falls back and delivers a fraction of the
# frame rate, which downstream looks exactly like a slow SLAM pipeline rather
# than a cable problem.
#
# Measured on this robot: the same configuration gave 30 fps depth on USB 3 and
# 11.5 fps on USB 2.

set -eo pipefail

FOUND=0
for d in /sys/bus/usb/devices/*/; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "8086" ] || continue
    FOUND=1

    SPEED=$(cat "$d/speed" 2>/dev/null || echo "?")
    VERSION=$(tr -d ' ' < "$d/version" 2>/dev/null || echo "?")
    POWER=$(cat "$d/bMaxPower" 2>/dev/null || echo "?")
    PRODUCT=$(cat "$d/product" 2>/dev/null || echo "RealSense")

    echo "device : $PRODUCT"
    echo "path   : $(basename "$d")"
    echo "speed  : ${SPEED} Mbps   (USB version ${VERSION}, max power ${POWER})"
    echo

    case "$SPEED" in
        10000|5000)
            echo "  VERDICT: USB 3 - correct. Full frame rate is available."
            ;;
        480)
            echo "  VERDICT: *** USB 2 - WRONG. ***"
            echo
            echo "  The D435i works but is badly throttled: measured 11.5 fps depth"
            echo "  against 30 configured. librealsense also hides some profiles"
            echo "  entirely on USB 2, so modes in the launch file may not exist."
            echo
            echo "  Almost always the cable. Most USB-C cables carry only the USB 2"
            echo "  conductors - they charge and pass data, but cannot do SuperSpeed,"
            echo "  and the link quietly falls back instead of failing. Use a cable"
            echo "  explicitly marked USB 3.x / SS, ideally the one that shipped with"
            echo "  the camera, and keep it short."
            ;;
        *)
            echo "  VERDICT: unexpected speed '$SPEED' - investigate."
            ;;
    esac
done

if [ "$FOUND" -eq 0 ]; then
    echo "No Intel RealSense found on USB. Is it plugged in?"
    exit 1
fi
