#!/usr/bin/env bash
# Gen1 start_robot_stable pattern: preflight + low-CPU file watchdog (+ optional launch).
# Default: only run preflight + watchdog (SKIP_LAUNCH=1). Host boot uses start_robot_host.sh.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/ros_env.sh" || true

SKIP_LAUNCH="${SKIP_LAUNCH:-1}"
export XW_LOG="${XW_LOG:-$ROOT/log}"
mkdir -p "$XW_LOG"

bash "${ROOT}/scripts/stability_preflight.sh"

WATCHDOG_PID=""
cleanup() {
  if [[ -n "$WATCHDOG_PID" ]] && kill -0 "$WATCHDOG_PID" 2>/dev/null; then
    kill "$WATCHDOG_PID" 2>/dev/null || true
    wait "$WATCHDOG_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

bash "${ROOT}/scripts/stability_watchdog.sh" &
WATCHDOG_PID=$!
echo "[stable] watchdog pid=$WATCHDOG_PID status=${XW_LOG}/topic_health_status"

if [[ "$SKIP_LAUNCH" == "1" ]]; then
  echo "[stable] SKIP_LAUNCH=1 — watchdog only (stack via start_robot_host / systemd)"
  wait "$WATCHDOG_PID"
  exit 0
fi

exec bash "${ROOT}/scripts/start_robot.sh"
