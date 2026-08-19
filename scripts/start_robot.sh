#!/usr/bin/env bash
# Container entry: source env and launch mock/real stack.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/ros_env.sh"
: "${USE_SIM_HW:=false}"
: "${USE_SIM_LIDAR:=false}"
: "${LIDAR_PORT:=/dev/radar}"
: "${LIDAR_BAUDRATE:=1000000}"
: "${CHASSIS_PORT:=/dev/chassis}"
: "${CHASSIS_BAUDRATE:=115200}"
: "${CHASSIS_FALLBACK:=/dev/ttyACM0}"
: "${USE_IMU:=true}"
: "${IMU_PORT:=/dev/imu}"
: "${IMU_BAUDRATE:=9600}"
: "${USE_EKF:=true}"
if [[ "${USE_EKF}" == "true" ]]; then
  : "${CHASSIS_ODOM_TOPIC:=odom/wheel}"
  : "${CHASSIS_PUBLISH_ODOM_TF:=false}"
else
  : "${CHASSIS_ODOM_TOPIC:=odom}"
  : "${CHASSIS_PUBLISH_ODOM_TF:=true}"
fi
: "${USE_DEPTH_CAM:=true}"
: "${USE_DEPTH_CAM_2:=true}"
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
  chassis_port:="${CHASSIS_PORT}" \
  chassis_baudrate:="${CHASSIS_BAUDRATE}" \
  chassis_fallback:="${CHASSIS_FALLBACK}" \
  use_imu:="${USE_IMU}" \
  imu_port:="${IMU_PORT}" \
  imu_baudrate:="${IMU_BAUDRATE}" \
  use_ekf:="${USE_EKF}" \
  chassis_odom_topic:="${CHASSIS_ODOM_TOPIC}" \
  chassis_publish_odom_tf:="${CHASSIS_PUBLISH_ODOM_TF}" \
  use_depth_cam:="${USE_DEPTH_CAM}" \
  use_depth_cam_2:="${USE_DEPTH_CAM_2}" \
  use_web:="${USE_WEB}" \
  use_foxglove:="${USE_FOXGLOVE}" \
  profile:="${PROFILE}"
