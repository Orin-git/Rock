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
  for dev in /dev/ttyUSB0 /dev/ttyACM0 /dev/chassis /dev/radar; do
    if [[ -e "$dev" ]]; then
      echo "[preflight] found $dev"
    fi
  done
fi
echo "[preflight] ok"
