#!/usr/bin/env bash
# Xiaowei Gen2 ROS environment for container (/ros2_ws).
# Auto-sourced from container ~/.bashrc — no need to run manually each time.
# Do not enable nounset before sourcing ROS (unbound AMENT_* vars).

# Already loaded in this shell?
if [[ -n "${XW_ROS_ENV_LOADED:-}" ]]; then
  return 0 2>/dev/null || exit 0
fi

export XW_WS="${XW_WS:-/ros2_ws}"
export XW_MAPS="${XW_MAPS:-${XW_WS}/maps}"
export XW_LOG="${XW_LOG:-${XW_WS}/log}"
mkdir -p "$XW_MAPS" "$XW_LOG" 2>/dev/null || true

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set +u
fi

if [[ -f "${XW_WS}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${XW_WS}/install/setup.bash"
  set +u
fi

if [[ -f /etc/robot-identity ]]; then
  set -a
  # shellcheck disable=SC1091
  source /etc/robot-identity
  set +a
elif [[ -f "${XW_WS}/config/robot-identity.example" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${XW_WS}/config/robot-identity.example"
  set +a
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DISABLE_LOANED_MESSAGES="${ROS_DISABLE_LOANED_MESSAGES:-1}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export XW_ROS_ENV_LOADED=1

# Only print once per interactive shell
if [[ $- == *i* ]] && [[ -z "${XW_ROS_ENV_QUIET:-}" ]]; then
  echo "[ros_env] ready  XW_WS=$XW_WS  DOMAIN=$ROS_DOMAIN_ID"
fi
