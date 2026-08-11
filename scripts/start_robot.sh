#!/usr/bin/env bash
# Container entry: source env and launch mock/real stack.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/ros_env.sh"
: "${USE_SIM_HW:=true}"
: "${USE_SIM_LIDAR:=false}"
: "${LIDAR_PORT:=/dev/radar}"
: "${LIDAR_BAUDRATE:=1000000}"
: "${USE_WEB:=true}"
: "${USE_FOXGLOVE:=true}"
: "${PROFILE:=normal}"
# Delay real lidar motor start so Web/SSH settle (lidar inrush can brown-out SBC)
: "${XW_LIDAR_START_DELAY:=25}"
export XW_LIDAR_START_DELAY

exec ros2 launch xw_bringup robot.launch.py \
  use_sim_hw:="${USE_SIM_HW}" \
  use_sim_lidar:="${USE_SIM_LIDAR}" \
  lidar_port:="${LIDAR_PORT}" \
  lidar_baudrate:="${LIDAR_BAUDRATE}" \
  use_web:="${USE_WEB}" \
  use_foxglove:="${USE_FOXGLOVE}" \
  profile:="${PROFILE}"
