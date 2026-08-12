#!/usr/bin/env bash
# Install perception runtime deps inside ros2_humble_dev (idempotent).
set -euo pipefail
CONTAINER="${XW_CONTAINER:-ros2_humble_dev}"
WS="${XW_WS:-/home/radxa/ros2_ws}"
WHL_DIR="$WS/third_party_wheels"
LITE_WHL_SRC="$WHL_DIR/rknn_toolkit_lite2-cp310.whl"
LITE_WHL_NAME='rknn_toolkit_lite2-2.3.2-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl'

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "container $CONTAINER not running"
  exit 1
fi

docker exec "$CONTAINER" bash -c 'DEBIAN_FRONTEND=noninteractive apt-get update -qq && apt-get install -y -qq python3-opencv python3-pip >/dev/null'

if [[ -f "$LITE_WHL_SRC" ]]; then
  docker cp "$LITE_WHL_SRC" "$CONTAINER:/tmp/$LITE_WHL_NAME"
  docker exec "$CONTAINER" python3 -m pip install -q "/tmp/$LITE_WHL_NAME" || true
fi

# Runtime libs from host (NPU). Resolve symlinks — docker cp of a symlink alone breaks.
_copy_host_lib() {
  local host_path="$1"
  local dest_name="${2:-}"
  if [[ ! -e "$host_path" ]]; then
    return 0
  fi
  local real
  real="$(readlink -f "$host_path")"
  local base
  base="${dest_name:-$(basename "$host_path")}"
  docker exec "$CONTAINER" mkdir -p /usr/lib /usr/lib/aarch64-linux-gnu
  docker cp "$real" "$CONTAINER:/usr/lib/$base"
  # Also drop versioned copy under multiarch path when present
  if [[ "$real" == *aarch64-linux-gnu* ]]; then
    docker cp "$real" "$CONTAINER:/usr/lib/aarch64-linux-gnu/$(basename "$real")"
  fi
}

_copy_host_lib /usr/lib/librknnrt.so librknnrt.so
_copy_host_lib /usr/lib/librknn_api.so librknn_api.so

docker exec "$CONTAINER" bash -lc '
  set -e
  python3 -c "from rknnlite.api import RKNNLite; print(\"rknnlite ok\")"
  test -f /usr/lib/librknnrt.so || test -f /usr/lib/aarch64-linux-gnu/librknnrt.so
  python3 - <<PY
from rknnlite.api import RKNNLite
import numpy as np
from pathlib import Path
model = Path("/ros2_ws/src/xw_perception/models/yolov8n-pose.rknn")
if not model.is_file():
    print("WARN: model missing", model)
    raise SystemExit(0)
r = RKNNLite()
assert r.load_rknn(str(model)) == 0
assert r.init_runtime() == 0
outs = r.inference(inputs=[np.zeros((1, 640, 640, 3), dtype=np.uint8)])
assert outs is not None and len(outs) >= 4
r.release()
print("NPU smoke OK", [tuple(o.shape) for o in outs])
PY
'
echo "done. Ensure models/yolov8n-pose.rknn exists (see models/README.md)."
