#!/usr/bin/env bash
# Host-side entry: run Gen2 stack inside ros2_humble_dev (for systemd boot).
set -eo pipefail

CONTAINER="${XW_CONTAINER:-ros2_humble_dev}"
USE_SIM_HW="${USE_SIM_HW:-false}"
USE_SIM_LIDAR="${USE_SIM_LIDAR:-false}"
LIDAR_PORT="${LIDAR_PORT:-/dev/radar}"
LIDAR_BAUDRATE="${LIDAR_BAUDRATE:-1000000}"
CHASSIS_PORT="${CHASSIS_PORT:-/dev/chassis}"
CHASSIS_BAUDRATE="${CHASSIS_BAUDRATE:-115200}"
CHASSIS_FALLBACK="${CHASSIS_FALLBACK:-/dev/ttyACM0}"
USE_IMU="${USE_IMU:-true}"
IMU_PORT="${IMU_PORT:-/dev/imu}"
IMU_BAUDRATE="${IMU_BAUDRATE:-9600}"
# Default: fuse wheel odom + IMU. Set USE_EKF=false to let chassis own /odom+TF.
USE_EKF="${USE_EKF:-true}"
if [[ "${USE_EKF}" == "true" ]]; then
  CHASSIS_ODOM_TOPIC="${CHASSIS_ODOM_TOPIC:-odom/wheel}"
  CHASSIS_PUBLISH_ODOM_TF="${CHASSIS_PUBLISH_ODOM_TF:-false}"
else
  CHASSIS_ODOM_TOPIC="${CHASSIS_ODOM_TOPIC:-odom}"
  CHASSIS_PUBLISH_ODOM_TF="${CHASSIS_PUBLISH_ODOM_TF:-true}"
fi
# Dual HP60C: cams must be on different USB host controllers (Bus1 + Bus3/5).
USE_DEPTH_CAM="${USE_DEPTH_CAM:-true}"
USE_DEPTH_CAM_2="${USE_DEPTH_CAM_2:-true}"
USE_WEB="${USE_WEB:-true}"
# systemd may still export USE_SIM_HW=true; prefer real MCU when present
# (set FORCE_SIM_HW=1 to keep mock despite /dev/chassis|/dev/ttyACM0).
if [[ "${FORCE_SIM_HW:-0}" != "1" ]]; then
  if [[ -e /dev/chassis || -e /dev/ttyACM0 ]]; then
    USE_SIM_HW=false
  fi
fi
# Mapping Live canvas needs Foxglove WS :8765. Old unit files may set false — override
# unless XW_ALLOW_NO_FOXGLOVE=1.
USE_FOXGLOVE="${USE_FOXGLOVE:-true}"
if [[ "${XW_ALLOW_NO_FOXGLOVE:-}" != "1" ]]; then
  USE_FOXGLOVE=true
fi
USE_GESTURE="${USE_GESTURE:-false}"
PROFILE="${PROFILE:-normal}"
# PointCloud2 debug relay (default off). Set USE_POINTCLOUD=true to enable for Foxglove.
USE_POINTCLOUD="${USE_POINTCLOUD:-false}"
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

# Single-instance lock: avoid systemd restart racing a manual start
LOCK_DIR="${XDG_RUNTIME_DIR:-/tmp}/xw-robot.lock"
if ! mkdir "$LOCK_DIR" 2>/dev/null; then
  # Stale lock from dead PID?
  old_pid="$(cat "$LOCK_DIR/pid" 2>/dev/null || true)"
  if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
    echo "[start_robot_host] another instance running (pid=$old_pid), exit"
    exit 0
  fi
  rm -rf "$LOCK_DIR"
  mkdir "$LOCK_DIR"
fi
echo $$ >"$LOCK_DIR/pid"
WATCHDOG_PID=""
cleanup_host() {
  if [[ -n "${WATCHDOG_PID}" ]] && kill -0 "$WATCHDOG_PID" 2>/dev/null; then
    kill "$WATCHDOG_PID" 2>/dev/null || true
    wait "$WATCHDOG_PID" 2>/dev/null || true
  fi
  rm -rf "$LOCK_DIR"
}
trap cleanup_host EXIT INT TERM

# Gen1-style pin watchdog on host: poll status file only (no DDS). Default on.
HOST_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export XW_LOG="${XW_LOG:-$HOST_WS/log}"
WATCHDOG_STATE_DIR="${WATCHDOG_STATE_DIR:-${XDG_RUNTIME_DIR:-/tmp}/xw-watchdog}"
mkdir -p "$WATCHDOG_STATE_DIR"
if [[ "${XW_WATCHDOG:-1}" == "1" ]]; then
  bash "$HOST_WS/scripts/stability_preflight.sh" || true
  # Ensure pin file is readable by host user (container writes as root).
  docker exec "$CONTAINER" bash -lc '
    mkdir -p /ros2_ws/log
    chmod 755 /ros2_ws/log 2>/dev/null || true
    [[ -f /ros2_ws/log/topic_health_status ]] && chmod 644 /ros2_ws/log/topic_health_status || true
  ' 2>/dev/null || true
  export AUTO_RECOVER="${AUTO_RECOVER:-0}"
  export RECOVER_CMD="${RECOVER_CMD:-systemctl restart xw-robot}"
  export STATUS_FILE="${STATUS_FILE:-$XW_LOG/topic_health_status}"
  export WATCHDOG_LOG="${WATCHDOG_LOG:-$WATCHDOG_STATE_DIR/stability_watchdog.log}"
  nohup bash "$HOST_WS/scripts/stability_watchdog.sh" \
    >>"$WATCHDOG_LOG" 2>&1 &
  WATCHDOG_PID=$!
  echo "$WATCHDOG_PID" >"$WATCHDOG_STATE_DIR/stability_watchdog.pid"
  echo "[start_robot_host] pin watchdog pid=$WATCHDOG_PID file=$STATUS_FILE log=$WATCHDOG_LOG"
fi

# Ensure NPU runtime lib is present in the container (survives recreate if re-run)
if [[ -e /usr/lib/librknnrt.so ]]; then
  real="$(readlink -f /usr/lib/librknnrt.so)"
  docker exec "$CONTAINER" mkdir -p /usr/lib /usr/lib/aarch64-linux-gnu 2>/dev/null || true
  docker cp "$real" "$CONTAINER:/usr/lib/librknnrt.so" 2>/dev/null || true
  docker cp "$real" "$CONTAINER:/usr/lib/aarch64-linux-gnu/$(basename "$real")" 2>/dev/null || true
fi

# Stop previous launch AND orphaned children (pkill launch alone leaves web on :9000).
# Avoid `pkill -f /ros2_ws/install/` — that pattern matches THIS kill script and self-terminates.
docker exec "$CONTAINER" bash -c '
  set +e
  kill_pat() {
    # $1 = extended regex matched against /proc/PID/cmdline
    local re="$1"
    local pid cmd
    for pid in /proc/[0-9]*; do
      pid=${pid#/proc/}
      [[ "$pid" =~ ^[0-9]+$ ]] || continue
      [[ "$pid" -eq "$$" ]] && continue
      cmd=$(tr "\0" " " <"/proc/$pid/cmdline" 2>/dev/null) || continue
      [[ "$cmd" =~ $re ]] || continue
      kill -9 "$pid" 2>/dev/null
    done
  }
  kill_pat "ros2[[:space:]]+launch[[:space:]]+xw_bringup"
  kill_pat "robot\\.launch\\.py"
  kill_pat "/ros2_ws/install/.+/lib/.+"
  kill_pat "person_perception_node"
  kill_pat "perception_stub_node"
  kill_pat "foxglove_bridge"
  kill_pat "robot_state_publisher"
  kill_pat "rplidar_node"
  kill_pat "async_slam_toolbox"
  kill_pat "ascamera"
  kill_pat "wt901_imu_node"
  sleep 2
' || true
sleep 1

echo "[start_robot_host] launching Gen2 inside $CONTAINER (sim_hw=$USE_SIM_HW sim_lidar=$USE_SIM_LIDAR web=$USE_WEB gesture=$USE_GESTURE pointcloud=$USE_POINTCLOUD lidar_delay=${XW_LIDAR_START_DELAY}s chassis=$CHASSIS_PORT imu=$IMU_PORT ekf=$USE_EKF depth=$USE_DEPTH_CAM depth2=$USE_DEPTH_CAM_2)"

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
    chassis_port:=${CHASSIS_PORT} \
    chassis_baudrate:=${CHASSIS_BAUDRATE} \
    chassis_fallback:=${CHASSIS_FALLBACK} \
    use_imu:=${USE_IMU} \
    imu_port:=${IMU_PORT} \
    imu_baudrate:=${IMU_BAUDRATE} \
    use_ekf:=${USE_EKF} \
    chassis_odom_topic:=${CHASSIS_ODOM_TOPIC} \
    chassis_publish_odom_tf:=${CHASSIS_PUBLISH_ODOM_TF} \
    use_depth_cam:=${USE_DEPTH_CAM} \
    use_depth_cam_2:=${USE_DEPTH_CAM_2} \
    use_web:=${USE_WEB} \
    use_gesture:=${USE_GESTURE} \
    use_foxglove:=${USE_FOXGLOVE} \
    enable_pointcloud:=${USE_POINTCLOUD} \
    profile:=${PROFILE}
"
