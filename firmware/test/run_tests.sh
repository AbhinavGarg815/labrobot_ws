#!/usr/bin/env bash
# Compile and run the host-side firmware tests.
#
# These exercise the RMCS-2303 MODBUS ASCII framing and the per-wheel
# sequencing natively, with a stub Arduino.h, so protocol and ordering bugs
# surface here instead of as an unexpected motor movement on the bench.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$HERE/../labrobot_firmware"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

CXXFLAGS=(-std=c++17 -Wall -Wextra -Werror -O1 -I "$HERE")

status=0

build_and_run() {
  local name="$1"; shift
  echo "=============================================================="
  g++ "${CXXFLAGS[@]}" "$@" -o "$OUT/$name"
  "$OUT/$name" || status=1
  echo
}

build_and_run test_rmcs2303 \
  "$HERE/test_rmcs2303.cpp" "$FW/rmcs2303.cpp"

build_and_run test_wheel_drive \
  "$HERE/test_wheel_drive.cpp" "$FW/wheel_drive.cpp" "$FW/rmcs2303.cpp"

build_and_run test_protocol \
  "$HERE/test_protocol.cpp" "$FW/labrobot_protocol.cpp"

build_and_run test_host_link \
  "$HERE/test_host_link.cpp" "$FW/host_link.cpp" "$FW/labrobot_protocol.cpp" \
  "$FW/wheel_drive.cpp" "$FW/rmcs2303.cpp"

echo "=============================================================="
if [ "$status" -eq 0 ]; then
  echo "ALL SUITES PASSED"
else
  echo "SOME SUITES FAILED"
fi
exit "$status"
