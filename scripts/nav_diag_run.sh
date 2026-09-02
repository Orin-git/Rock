#!/usr/bin/env bash
# 导航诊断采集：低开销 2Hz 采样，跑完一圈后 stop 自动出报告。
# 用法（在宿主机）:
#   ./scripts/nav_diag_run.sh start    # 开始采集
#   ... 你跑导航一圈 ...
#   ./scripts/nav_diag_run.sh stop     # 停止并分析
#   ./scripts/nav_diag_run.sh status   # 查看是否在采

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${NAV_DIAG_CONTAINER:-ros2_humble_dev}"
LOG_ROOT="${XW_LOG:-$ROOT/log}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${NAV_DIAG_OUT:-$LOG_ROOT/nav_diag_$STAMP}"
PID_FILE="$LOG_ROOT/nav_diag_collect.pid"
HZ="${NAV_DIAG_HZ:-2.0}"

_run_in_container() {
  docker exec "$CONTAINER" bash -lc "
    set -e
    source /ros2_ws/scripts/ros_env.sh
    export XW_ROS_ENV_QUIET=1
    $*
  "
}

cmd_start() {
  if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "[nav_diag] 已在采集 (pid=$(cat "$PID_FILE"))"
    exit 1
  fi
  mkdir -p "$LOG_ROOT"
  # OUT_DIR is on host; map into container at same path if bind-mounted, else use /ros2_ws/log
  local container_out="$OUT_DIR"
  if [[ "$OUT_DIR" != /ros2_ws/* ]]; then
    container_out="/ros2_ws/log/$(basename "$OUT_DIR")"
  fi
  echo "[nav_diag] 输出: $container_out  采样: ${HZ}Hz"
  echo "[nav_diag] 准备好后请开始导航..."
  _run_in_container "
    mkdir -p '$container_out'
    nohup python3 /ros2_ws/scripts/nav_diag_collect.py \
      --out '$container_out' --hz $HZ \
      > '$container_out/collector.log' 2>&1 &
    echo \$! > '$container_out/collector.pid'
    echo STARTED:\$!
  " | tee /tmp/nav_diag_start.log
  # Save host-side pointer
  echo "$container_out" > "$LOG_ROOT/nav_diag_last_out.txt"
  local cpid
  cpid=$(grep '^STARTED:' /tmp/nav_diag_start.log | tail -1 | cut -d: -f2)
  echo "$cpid" > "$PID_FILE"
  echo "[nav_diag] 采集已启动。跑完一圈后执行: $0 stop"
}

cmd_stop() {
  local container_out
  if [[ -f "$LOG_ROOT/nav_diag_last_out.txt" ]]; then
    container_out="$(cat "$LOG_ROOT/nav_diag_last_out.txt")"
  else
    echo "[nav_diag] 找不到上次输出目录"
    exit 1
  fi
  echo "[nav_diag] 停止采集 → $container_out"
  _run_in_container "
    if [[ -f '$container_out/collector.pid' ]]; then
      kill \$(cat '$container_out/collector.pid') 2>/dev/null || true
      sleep 1
    fi
    python3 /ros2_ws/scripts/nav_diag_analyze.py '$container_out'
  "
  rm -f "$PID_FILE"
  echo "[nav_diag] 报告: $container_out/report.txt"
}

cmd_status() {
  if [[ -f "$PID_FILE" ]]; then
    echo "[nav_diag] pid file: $(cat "$PID_FILE")"
  fi
  if [[ -f "$LOG_ROOT/nav_diag_last_out.txt" ]]; then
    local d
    d="$(cat "$LOG_ROOT/nav_diag_last_out.txt")"
    _run_in_container "
      if [[ -f '$d/collector.pid' ]] && kill -0 \$(cat '$d/collector.pid') 2>/dev/null; then
        echo RUNNING pid=\$(cat '$d/collector.pid')
        wc -l '$d/samples.csv' 2>/dev/null || true
        tail -3 '$d/collector.log' 2>/dev/null || true
      else
        echo STOPPED out=$d
      fi
    "
  else
    echo "[nav_diag] 未在采集"
  fi
}

case "${1:-}" in
  start)  cmd_start ;;
  stop)   cmd_stop ;;
  status) cmd_status ;;
  *)
    echo "Usage: $0 {start|stop|status}"
    exit 1
    ;;
esac
