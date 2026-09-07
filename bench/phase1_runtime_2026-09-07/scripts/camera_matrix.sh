#!/usr/bin/env bash
# Camera stream matrix sampler (inside container)
set +u; source /ros2_ws/scripts/ros_env.sh; set -e
OUT="${1:?}"
mkdir -p "$OUT"
MODE="${2:-manual}"

info_subs() {
  local t="$1"
  timeout 4 ros2 topic info "$t" 2>/dev/null | awk '/Subscription count:/{print $3; exit}'
}
hz() {
  timeout 5 ros2 topic hz "$1" 2>&1 | awk '/average rate:/{print $3; exit}'
}

{
  echo "mode=$MODE ts=$(date -Iseconds)"
  timeout 3 ros2 topic echo /xw/robot_state --once 2>&1 | grep -E "mode_name|detail" || true
  timeout 2 ros2 topic echo /xw/perception/profile --once 2>&1 || true
  echo "npu=$(cat /sys/class/devfreq/fdab0000.npu/load 2>/dev/null || echo NA)"
  for cam in up down; do
    ns="front_${cam}"
    for kind in color/image_raw depth/image_raw color/image_raw/compressed depth/points; do
      topic="/camera/${ns}/${kind}"
      echo "PUB $topic hz=$(hz "$topic") subs=$(info_subs "$topic")"
    done
  done
  for t in \
    /ascamera_hp60c/camera_publisher/mjpeg0/compressed \
    /ascamera_hp60c/camera_publisher/rgb0/image \
    /ascamera_hp60c_2/camera_publisher/mjpeg0/compressed \
    /ascamera_hp60c_2/camera_publisher/rgb0/image
  do
    echo "VENDOR $t subs=$(info_subs "$t")"
  done
  # perception active?
  timeout 3 ros2 topic hz /xw/perception/tracks 2>&1 | grep average | tail -1 || echo "tracks: idle/none"
} | tee "$OUT/${MODE}.txt"
