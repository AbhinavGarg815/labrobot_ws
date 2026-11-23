#!/usr/bin/env bash
# Rebuild the person-following models from scratch (weights -> ONNX -> TensorRT).
#   tools/build_follow_models.sh                    # everything
#   tools/build_follow_models.sh --engines-only     # ONNX already there
#   tools/build_follow_models.sh --engines-only osnet_ain_x1_0_msmt17 yolo26s-pose
set -euo pipefail

M=~/labrobot_models
VENV=~/venvs/pf_export/bin/python
TRTEXEC=/usr/src/tensorrt/bin/trtexec
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$M/engines"

if [[ "${1:-}" == "--engines-only" ]]; then
    shift
else
    YOLO_OFFLINE=1 "$VENV" -I "$HERE/export_follow_models.py" --all
fi
# Re-ID first: they are small and every configuration needs one.
WANT=("$@")
[[ ${#WANT[@]} -eq 0 ]] && WANT=(osnet_x0_25_msmt17 osnet_ain_x1_0_msmt17 \
                                 yolo26s-pose yolo26n-pose yolo11s-pose yolo11n-pose)

build() {   # onnx engine extra-args...
    local onnx="$1" engine="$2"; shift 2
    echo "== $engine"
    "$TRTEXEC" --onnx="$onnx" --saveEngine="$engine" --fp16 \
        --memPoolSize=workspace:1024 --skipInference "$@" \
        > "${engine%.engine}.build.log" 2>&1 \
        || { echo "FAILED - see ${engine%.engine}.build.log"; return 1; }
    ls -la "$engine"
}

for m in "${WANT[@]}"; do
    case "$m" in
        osnet*)
            build "$M/onnx/${m}_256x128_u8.onnx" "$M/engines/${m}_fp16.engine" \
                --minShapes=crops:1x256x128x3 --optShapes=crops:4x256x128x3 \
                --maxShapes=crops:16x256x128x3 ;;
        yolo*)
            build "$M/onnx/${m}_480x640_u8.onnx" "$M/engines/${m}_480x640_fp16.engine" ;;
        *) echo "unknown model $m"; exit 1 ;;
    esac
done
echo "done: $M/engines"
