#!/usr/bin/env bash
# Compile and flash the labrobot firmware to the Arduino Mega, headless.
#
#   firmware/flash.sh              production build (binary protocol), upload
#   firmware/flash.sh --bench      bench build (human console), upload + monitor
#   firmware/flash.sh --check      compile BOTH modes, do not touch the board
#
# Production mode speaks the framed binary protocol to the Jetson at 500000 baud
# and brakes both wheels if commands stop arriving. Bench mode is the console
# used to commission the drives; see the key list printed on connect.
#
# The board is addressed through the /dev/labrobot_mcu udev symlink, which is
# pinned to its USB serial number, so it cannot accidentally target a phone or
# another CDC-ACM device.

set -euo pipefail

export PATH="$HOME/.local/bin:$PATH"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH="$HERE/labrobot_firmware"
FQBN="arduino:avr:mega:cpu=atmega2560"
PORT="/dev/labrobot_mcu"
BENCH_BAUD=115200

mode="${1:-}"

echo "=== host-side tests ==="
# Never flash firmware whose protocol logic is failing on the bench-free tests.
"$HERE/test/run_tests.sh" >/dev/null
echo "all test suites passed"
echo

compile() {  # $1 = human label, $2... = extra arduino-cli args
  local label="$1"; shift
  echo "=== compile: $label ==="
  arduino-cli compile --fqbn "$FQBN" "$@" "$SKETCH" 2>&1 \
    | grep -E "Sketch uses|Global variables|error" || true
}

if [ "$mode" = "--check" ]; then
  compile "production"
  compile "bench" --build-property "build.extra_flags=-DLABROBOT_BENCH_MODE=1"
  echo
  echo "compile-only mode, board untouched"
  exit 0
fi

if [ "$mode" = "--bench" ]; then
  compile "bench" --build-property "build.extra_flags=-DLABROBOT_BENCH_MODE=1"
  EXTRA=(--build-property "build.extra_flags=-DLABROBOT_BENCH_MODE=1")
else
  compile "production"
  EXTRA=()
fi

if [ ! -e "$PORT" ]; then
  echo "ERROR: $PORT not present. Is the Mega plugged in?" >&2
  echo "Devices seen:" >&2
  ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null >&2 || echo "  (none)" >&2
  exit 1
fi

echo
echo "=== upload to $PORT ==="
echo "SAFETY: the robot has no kill switch. Wheels clear of the bench."
arduino-cli upload -p "$PORT" --fqbn "$FQBN" "${EXTRA[@]+"${EXTRA[@]}"}" "$SKETCH"

if [ "$mode" != "--bench" ]; then
  echo
  echo "Production firmware flashed. It emits BINARY status frames at 500000"
  echo "baud, so a text monitor will show garbage - use the protocol client:"
  echo "    firmware/host_client.py --status"
  exit 0
fi

echo
echo "=== serial console @ ${BENCH_BAUD} (Ctrl-C to exit) ==="
echo "keys: f=fwd b=rev l/r=spin s=stop n=neutral +/-=speed h=zero e=estop"
echo "      ?=status d=raw-dump p=probe"
# The Mega resets when the port is opened, so give the bootloader time to hand
# over before the banner is expected.
sleep 2
arduino-cli monitor -p "$PORT" -c baudrate="$BENCH_BAUD"
