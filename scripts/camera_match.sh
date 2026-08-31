#!/bin/bash
# Bind logical cameras to USB topology by STRUCTURE (bus numbers drift per boot):
#   physical TOP camera = direct on root port (kernel path like 4-1)      -> depth 1
#   physical BOTTOM     = behind a hub     (kernel path like 5-1.2)      -> depth 2
# Both devices share VID:PID (3482:6723), so structure is the only stable key.
# Rewrites depth_camera.yaml (front_up) and depth_camera_front_down.yaml
# BEFORE the launch tree is built. Fail-safe: skip on ambiguous topology.
set -u
LOG="${CAM_MATCH_LOG:-/tmp/camera_match.log}"
CFG_UP="/home/radxa/ros2_ws/src/xw_sensors/config/depth_camera.yaml"
CFG_DOWN="/home/radxa/ros2_ws/src/xw_sensors/config/depth_camera_front_down.yaml"
mkdir -p "$(dirname "$LOG")"

devs=()
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = 3482 ] && [ "$(cat "$d/idProduct")" = 6723 ] && devs+=("$(basename "$d")")
done

if [ "${#devs[@]}" -ne 2 ]; then
  echo "[camera_match] found ${#devs[@]} ASJ devices (expect 2) — skip, keep configs" >> "$LOG"
  exit 0
fi
up=""; down=""
for name in "${devs[@]}"; do
  bus="${name%%-*}"; path="${name#*-}"
  if [[ "$path" == *.* ]]; then down="$bus:$path"; else up="$bus:$path"; fi
done
if [ -z "$up" ] || [ -z "$down" ]; then
  echo "[camera_match] ambiguous (up=? down=?) — skip, keep configs" >> "$LOG"
  exit 0
fi
upbus="${up%%:*}"; uppath="${up##*:}"
downbus="${down%%:*}"; downpath="${down##*:}"
cur_up="$(grep -m2 -E "^(usb_bus_no|usb_path)" "$CFG_UP" | tr "\n" " ")"
cur_down="$(grep -m2 -E "^(usb_bus_no|usb_path)" "$CFG_DOWN" | tr "\n" " ")"
sed -i -E "s/^(usb_bus_no:).*/\1 $upbus/; s/^(usb_path:).*/\1 \"$uppath\"/" "$CFG_UP"
sed -i -E "s/^(usb_bus_no:).*/\1 $downbus/; s/^(usb_path:).*/\1 \"$downpath\"/" "$CFG_DOWN"
echo "[camera_match] devs= up=$upbus/$uppath down=$downbus/$downpath was-up= was-down=" >> "$LOG"
