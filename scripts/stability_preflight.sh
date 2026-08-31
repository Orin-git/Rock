#!/usr/bin/env bash
# Lightweight preflight before launch (Docker-friendly).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/ros_env.sh" || true

echo "[preflight] ROOT=$ROOT"
[[ -d "${XW_MAPS:-$ROOT/maps}" ]] || mkdir -p "${XW_MAPS:-$ROOT/maps}"
[[ -d "${XW_LOG:-$ROOT/log}" ]] || mkdir -p "${XW_LOG:-$ROOT/log}"

if [[ ! -f "${ROOT}/install/setup.bash" ]]; then
  echo "[preflight] WARN: workspace not built. Run: cd $ROOT && colcon build --symlink-install"
fi

if [[ "${USE_SIM_HW:-true}" != "true" ]]; then
  for dev in /dev/ttyUSB0 /dev/ttyACM0 /dev/chassis /dev/imu /dev/radar; do
    if [[ -e "$dev" ]]; then
      echo "[preflight] found $dev"
    fi
  done
  # CP210x may enumerate while lidar MCU is unpowered / TX-RX open (signal-only cable).
  if [[ -e /dev/radar ]] && command -v python3 >/dev/null; then
    if python3 - <<'PY'
import serial, time, sys
try:
    s = serial.Serial('/dev/radar', 1000000, timeout=0.4)
except Exception as e:
    print('[preflight] WARN: cannot open /dev/radar:', e)
    sys.exit(0)
s.write(bytes([0xA5, 0x25])); time.sleep(0.05); s.reset_input_buffer()
s.write(bytes([0xA5, 0x50])); time.sleep(0.35)
info = s.read(32); s.close()
if not info:
    print('[preflight] WARN: /dev/radar silent (CP210x up, lidar MCU no reply). Check 5V power + GND + TX/RX on signal cable.')
    sys.exit(0)
print('[preflight] lidar GET_INFO ok, bytes=', len(info))
PY
    then true; fi
  fi
fi
echo "[preflight] ok"
