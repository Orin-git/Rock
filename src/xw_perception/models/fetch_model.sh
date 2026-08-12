#!/usr/bin/env bash
# Download Rockchip yolov8n-pose ONNX; optionally convert if rknn-toolkit2 is present.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
ONNX_URL='https://ftrg.zbox.filez.com/v2/delivery/data/95f00b0fc900458ba134f8b180b3f7a1/examples/yolov8_pose/yolov8n-pose.onnx'
ONNX="$DIR/yolov8n-pose.onnx"
RKNN="$DIR/yolov8n-pose.rknn"

if [[ ! -f "$ONNX" ]]; then
  echo "[fetch_model] downloading ONNX → $ONNX"
  curl -fL --retry 3 -o "$ONNX" "$ONNX_URL"
else
  echo "[fetch_model] ONNX already present"
fi

if [[ -f "$RKNN" ]]; then
  echo "[fetch_model] RKNN already present: $RKNN"
  exit 0
fi

if python3 -c 'from rknn.api import RKNN' 2>/dev/null; then
  echo "[fetch_model] converting with rknn-toolkit2 → $RKNN"
  python3 - <<'PY'
from rknn.api import RKNN
import os
onnx = os.environ.get('ONNX')
rknn_path = os.environ.get('RKNN')
rknn = RKNN(verbose=True)
rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform='rk3588')
assert rknn.load_onnx(model=onnx) == 0
assert rknn.build(do_quantization=True) == 0
assert rknn.export_rknn(rknn_path) == 0
rknn.release()
print('exported', rknn_path)
PY
else
  echo "[fetch_model] rknn-toolkit2 not installed — copy a prebuilt yolov8n-pose.rknn here"
  echo "             (see models/README.md). ONNX saved at: $ONNX"
  exit 0
fi
