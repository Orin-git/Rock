#!/usr/bin/env bash
# Install host udev rules: /dev/chassis /dev/imu /dev/radar
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${ROOT}/scripts/udev/99-xw-robot-usb.rules"
DEST="/etc/udev/rules.d/99-xw-robot-usb.rules"

if [[ ! -f "$SRC" ]]; then
  echo "[install_robot_udev] missing $SRC" >&2
  exit 1
fi

cp "$SRC" "$DEST"
# Retire split rules (superseded by combined file)
rm -f /etc/udev/rules.d/99-xw-chassis.rules /etc/udev/rules.d/99-rplidar-radar.rules

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty || true
sleep 0.5
echo "[install_robot_udev] installed $DEST"
ls -la /dev/chassis /dev/imu /dev/radar /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
