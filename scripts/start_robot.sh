#!/usr/bin/env bash
# Container entry: source env and launch mock/real stack.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/ros_env.sh"
: "${USE_SIM_HW:=true}"
: "${USE_WEB:=true}"
: "${USE_FOXGLOVE:=true}"
: "${PROFILE:=normal}"

exec ros2 launch xw_bringup robot.launch.py \
  use_sim_hw:="${USE_SIM_HW}" \
  use_web:="${USE_WEB}" \
  use_foxglove:="${USE_FOXGLOVE}" \
  profile:="${PROFILE}"
