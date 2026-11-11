#!/usr/bin/env bash
# Rebuild the person-following models from scratch (weights -> ONNX -> TensorRT).
#
#   tools/build_follow_models.sh                 # everything
#   tools/build_follow_models.sh --engines-only  # ONNX already there
#
# Nothing here is in git (sizes, and engines are specific to this Jetson's
# TensorRT version). Layout:
#   ~/labrobot_models/download/  weights (.pt) - see "Sources" below
#   ~/labrobot_models/onnx/      exported graphs (uint8 input, preprocessing inside)
#   ~/labrobot_models/engines/   TensorRT FP16 engines + trtexec logs
#
# The export venv (~/venvs/pf_export, torch CPU + ultralytics + boxmot) is
# separate from ROS's Python; see PROJECT_REPORT M6.T0 for how it was made.
#
# Sources (sha256 in PROJECT_REPORT M6.T1):
#   yolo26{n,s}-pose.pt, yolo11{n,s}-pose.pt
#       https://github.com/ultralytics/assets/releases/download/v8.4.0/<name>.pt
#   osnet_x0_25_msmt17.pt     gdown 1sSwXSUlj4_tHZequ_iZ8w_Jh0VaRQMqF
#   osnet_ain_x1_0_msmt17.pt  gdown 1SigwBE6mPdqiJMqhuIY4aqC7--5CsMal
#       (torchreid model zoo, via boxmot's catalogue)
#
# Building takes a few minutes per model and loads the CPU; the robot can stay
# up (it does not move), but do not build while it is driving.
set -euo pipefail

M=~/labrobot_models
VENV=~/venvs/pf_export/bin/python
TRTEXEC=/usr/src/tensorrt/bin/trtexec
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$M/engines"

if [[ "${1:-}" != "--engines-only" ]]; then
    YOLO_OFFLINE=1 "$VENV" -I "$HERE/export_follow_models.py" --all
fi

build() {   # onnx engine extra-args...
    local onnx="$1" engine="$2"; shift 2
    echo "== $engine"
    "$TRTEXEC" --onnx="$onnx" --saveEngine="$engine" --fp16 \
        --memPoolSize=workspace:1024 --skipInference "$@" \
        > "${engine%.engine}.build.log" 2>&1 \
        || { echo "FAILED - see ${engine%.engine}.build.log"; return 1; }
    ls -la "$engine"
}

for p in yolo26n-pose yolo26s-pose yolo11n-pose yolo11s-pose; do
    build "$M/onnx/${p}_480x640_u8.onnx" "$M/engines/${p}_480x640_fp16.engine"
done
for r in osnet_x0_25_msmt17 osnet_ain_x1_0_msmt17; do
    build "$M/onnx/${r}_256x128_u8.onnx" "$M/engines/${r}_fp16.engine" \
        --minShapes=crops:1x256x128x3 --optShapes=crops:4x256x128x3 \
        --maxShapes=crops:16x256x128x3
done
echo "done: $M/engines"
