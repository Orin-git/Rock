#!/usr/bin/env bash
# Host-side entry: run Gen2 stack inside ros2_humble_dev (for systemd boot).
set -eo pipefail

CONTAINER="${XW_CONTAINER:-ros2_humble_dev}"
USE_SIM_HW="${USE_SIM_HW:-true}"
USE_SIM_LIDAR="${USE_SIM_LIDAR:-false}"
LIDAR_PORT="${LIDAR_PORT:-/dev/radar}"
LIDAR_BAUDRATE="${LIDAR_BAUDRATE:-1000000}"
USE_WEB="${USE_WEB:-true}"
# Mapping Live canvas needs Foxglove WS :8765. Old unit files may set false — override
# unless XW_ALLOW_NO_FOXGLOVE=1.
USE_FOXGLOVE="${USE_FOXGLOVE:-true}"
if [[ "${XW_ALLOW_NO_FOXGLOVE:-}" != "1" ]]; then
  USE_FOXGLOVE=true
fi
PROFILE="${PROFILE:-normal}"
# Delay real lidar motor start after Web is up (inrush can brown-out SBC / drop SSH)
XW_LIDAR_START_DELAY="${XW_LIDAR_START_DELAY:-30}"

# Wait for Docker daemon
for _ in $(seq 1 60); do
  if docker info >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

if ! docker info >/dev/null 2>&1; then
  echo "[start_robot_host] docker daemon not ready"
  exit 1
fi

# Ensure container is running
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    docker start "$CONTAINER"
  else
    echo "[start_robot_host] container $CONTAINER missing; create with setup_ros2_env.sh"
    exit 1
  fi
fi

for _ in $(seq 1 30); do
  if docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
    break
  fi
  sleep 1
done

# Stop any previous launch residual (best-effort; do not kill this script's shell)
docker exec "$CONTAINER" bash -c \
  'pkill -f "ros2 launch xw_bringup" 2>/dev/null || true;
   pkill -f "robot.launch.py" 2>/dev/null || true' || true
sleep 1

echo "[start_robot_host] launching Gen2 inside $CONTAINER (sim_hw=$USE_SIM_HW sim_lidar=$USE_SIM_LIDAR web=$USE_WEB lidar_delay=${XW_LIDAR_START_DELAY}s)"

# Foreground so systemd tracks the process
exec docker exec -i "$CONTAINER" bash -lc "
  set -e
  export XW_WS=/ros2_ws
  export XW_LIDAR_START_DELAY=${XW_LIDAR_START_DELAY}
  # shellcheck disable=SC1091
  source /ros2_ws/scripts/ros_env.sh
  exec ros2 launch xw_bringup robot.launch.py \
    use_sim_hw:=${USE_SIM_HW} \
    use_sim_lidar:=${USE_SIM_LIDAR} \
    lidar_port:=${LIDAR_PORT} \
    lidar_baudrate:=${LIDAR_BAUDRATE} \
    use_web:=${USE_WEB} \
    use_foxglove:=${USE_FOXGLOVE} \
    profile:=${PROFILE}
"
