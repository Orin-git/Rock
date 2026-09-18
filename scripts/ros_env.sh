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

# ── RMW 实现：2026-09-18 由 Fast DDS 换为 CycloneDDS ────────────────────────────
# 换的理由只有一条，但是硬的：Fast DDS 2.6.11 的 EKF futex 死锁
# （09-16 已挖到 10 线程调用链，见 memory robot189-ekf-futex-hang）。
# ⚠️ 回滚 = 把下面 RMW_IMPLEMENTATION 的默认值改回 rmw_fastrtps_cpp，
#    并把 CYCLONEDDS_URI 那行注释掉即可；其余 Fast DDS 变量原样保留，回滚即用。
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# 主域配置。三条都是 2026-09-18 实测逼出来的，见该文件内的完整注释：
#   ① 属性名是 address=，**不是 ip=**（`ip=` 被 0.10.5 解析器硬拒，participant 建不起来）
#   ② <Interfaces> 里**绝不能列 lo** —— 列了 CycloneDDS 会打印
#      "selected interface lo is not multicast-capable: disabling multicast"，
#      组播发现被整个关掉、单播发送也全失败，两端互相完全看不见
#      （对照实测：去掉 lo 后 25/25 小消息 + 8/8 条 1.2 MB 大消息，0 失败行）
#   ③ 网卡必须钉死在 192.168.0.189：本机 docker0 是 linkdown 却持有 172.17.0.1/16，
#      正是 CycloneDDS 自选网卡会踩的经典坑
# 刻意写成**无条件** export（不用 ${CYCLONEDDS_URI:-...}）：生产只允许用这一份配置，
# 任何从外部继承进来的 URI 都不该把它顶掉。
export CYCLONEDDS_URI=file:///ros2_ws/config/cyclonedds/main.xml

# ⚠️ 绝不在这里设 ROS_LOCALHOST_ONLY=1。一代（70）设了 1，因为它是刻意放弃跨机可见性；
#    189 必须跟主机 165 跨机通信。**不设即可** —— ROS 自带的 ros_environment 钩子
#    （/opt/ros/humble/share/ros_environment/environment/1.ros_localhost_only.dsv）
#    会 set-if-unset 成 0。已实测确认生效。
export ROS_DISABLE_LOANED_MESSAGES="${ROS_DISABLE_LOANED_MESSAGES:-1}"

# 以下两个是 Fast DDS 专用变量。CycloneDDS 会忽略它们，留着是为了回滚时原样可用。
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export XW_ROS_ENV_LOADED=1

# Only print once per interactive shell
if [[ $- == *i* ]] && [[ -z "${XW_ROS_ENV_QUIET:-}" ]]; then
  echo "[ros_env] ready  XW_WS=$XW_WS  DOMAIN=$ROS_DOMAIN_ID"
fi
