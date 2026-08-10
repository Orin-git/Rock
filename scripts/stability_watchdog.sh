#!/usr/bin/env bash
# Read topic_health_status file; alert if stale (no new DDS each cycle).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STATUS_FILE="${STATUS_FILE:-${XW_LOG:-$ROOT/log}/topic_health_status}"
MAX_AGE="${STATUS_MAX_AGE_SEC:-30}"
INTERVAL="${WATCHDOG_INTERVAL_SEC:-15}"

echo "[watchdog] watching $STATUS_FILE every ${INTERVAL}s"
while true; do
  if [[ ! -f "$STATUS_FILE" ]]; then
    echo "[watchdog] missing status file"
  else
    now=$(date +%s)
    mtime=$(stat -c %Y "$STATUS_FILE" 2>/dev/null || stat -f %m "$STATUS_FILE")
    age=$((now - mtime))
    if (( age > MAX_AGE )); then
      echo "[watchdog] STALE status age=${age}s"
    else
      if grep -q 'dead' "$STATUS_FILE"; then
        echo "[watchdog] ALERT:"; cat "$STATUS_FILE"
      fi
    fi
  fi
  sleep "$INTERVAL"
done
