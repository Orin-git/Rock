#!/usr/bin/env bash
# Convert Rockchip yolov8n-pose.onnx → yolov8n-pose.rknn (needs rknn-toolkit2 on host).
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
ONNX="${1:-$DIR/yolov8n-pose.onnx}"
OUT="${2:-$DIR/yolov8n-pose.rknn}"
export ONNX OUT
python3 <<'PY'
import os, sys
from rknn.api import RKNN
onnx = os.environ['ONNX']
out = os.environ['OUT']
if not os.path.isfile(onnx):
    sys.exit(f'missing onnx: {onnx}')
rknn = RKNN(verbose=True)
rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform='rk3588')
assert rknn.load_onnx(model=onnx) == 0, 'load_onnx failed'
ret = rknn.build(do_quantization=False)
if ret != 0:
    sys.exit(f'build failed {ret}')
assert rknn.export_rknn(out) == 0, 'export failed'
rknn.release()
print('wrote', out)
PY
