#!/usr/bin/env bash
# Install host udev rule so chassis MCU appears as /dev/chassis.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${ROOT}/scripts/udev/99-xw-chassis.rules"
DEST="/etc/udev/rules.d/99-xw-chassis.rules"

if [[ ! -f "$SRC" ]]; then
  echo "[install_chassis_udev] missing $SRC" >&2
  exit 1
fi

cp "$SRC" "$DEST"
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty || true
echo "[install_chassis_udev] installed $DEST"
ls -la /dev/chassis /dev/ttyACM* 2>/dev/null || true
