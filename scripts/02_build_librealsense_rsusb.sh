#!/usr/bin/env bash
# Build librealsense with the libusb (RSUSB) backend, so the D435i IMU works.
#   scripts/02_build_librealsense_rsusb.sh          # ~30-60 min on the Orin Nano

set -euo pipefail

VERSION="${LIBRS_VERSION:-v2.58.2}"
PREFIX="${LIBRS_PREFIX:-$HOME/opt/librealsense-rsusb}"
SRC="${LIBRS_SRC:-$HOME/src/librealsense}"
# Leave a core free: the Orin Nano has 8 GB and the compile is memory-hungry.
JOBS="${LIBRS_JOBS:-5}"

echo "=== librealsense RSUSB build ==="
echo "  version : $VERSION"
echo "  source  : $SRC"
echo "  prefix  : $PREFIX"
echo "  jobs    : $JOBS"
echo

mkdir -p "$(dirname "$SRC")"
if [ ! -d "$SRC/.git" ]; then
  echo "=== cloning (shallow - the full history is ~1 GB) ==="
  git clone --depth 1 --branch "$VERSION" \
    https://github.com/IntelRealSense/librealsense.git "$SRC"
else
  echo "=== source already present, reusing $SRC ==="
  git -C "$SRC" fetch --depth 1 origin tag "$VERSION" || true
  git -C "$SRC" checkout "$VERSION"
fi

BUILD="$SRC/build-rsusb"
mkdir -p "$BUILD"
cd "$BUILD"

echo
echo "=== configure ==="
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_TOOLS=ON \
  -DCHECK_FOR_UPDATES=OFF

echo
echo "=== build (this is the long part) ==="
make -j"$JOBS"

echo
echo "=== install ==="
make install

echo
echo "=== result ==="
ls -l "$PREFIX/lib/"librealsense2.so* 2>/dev/null || true
echo
echo "Verify the IMU with:"
echo "  LD_LIBRARY_PATH=$PREFIX/lib \\"
echo "      $PREFIX/bin/rs-enumerate-devices | grep -iE 'accel|gyro'"
echo
echo "To revert, simply do not set LD_LIBRARY_PATH - the packaged library is untouched."
