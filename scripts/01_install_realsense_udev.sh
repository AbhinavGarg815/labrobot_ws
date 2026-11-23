#!/usr/bin/env bash
# Install the Intel RealSense udev rules.
#   sudo bash scripts/01_install_realsense_udev.sh

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: run me with sudo." >&2
  exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$HERE/udev/99-realsense-libusb.rules"
DST=/etc/udev/rules.d/99-realsense-libusb.rules

if [ ! -f "$SRC" ]; then
  echo "ERROR: $SRC not found." >&2
  exit 1
fi

echo "=== before ==="
for d in /dev/bus/usb/*/*; do
  :
done
ls -l /dev/bus/usb/*/* 2>/dev/null | head -0 || true
python3 - <<'PY' || true
import subprocess, re, os
out = subprocess.run(["lsusb"], capture_output=True, text=True).stdout
for line in out.splitlines():
    if "8086" in line and "RealSense" in line:
        m = re.match(r"Bus (\d+) Device (\d+):", line)
        if m:
            p = f"/dev/bus/usb/{m.group(1)}/{m.group(2)}"
            st = os.stat(p)
            print(f"  {p}  mode {oct(st.st_mode)[-4:]}")
PY

echo
echo "=== installing $DST ==="
install -m 0644 "$SRC" "$DST"

echo "=== reloading udev (scoped to Intel USB devices only) ==="
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=8086
sleep 2

echo
echo "=== after ==="
python3 - <<'PY' || true
import subprocess, re, os
out = subprocess.run(["lsusb"], capture_output=True, text=True).stdout
ok = False
for line in out.splitlines():
    if "8086" in line and "RealSense" in line:
        m = re.match(r"Bus (\d+) Device (\d+):", line)
        if m:
            p = f"/dev/bus/usb/{m.group(1)}/{m.group(2)}"
            st = os.stat(p)
            mode = oct(st.st_mode)[-4:]
            print(f"  {p}  mode {mode}")
            ok = mode in ("0666", "0664")
print()
print("  writable by the user -> IMU should now enumerate" if ok
      else "  STILL not writable - unplug and replug the camera, then re-check")
PY

echo
echo "Done. If the mode did not change, physically unplug and replug the camera:"
echo "udev applies permissions when the device appears, and a trigger does not"
echo "always re-apply them to an already-enumerated device."
