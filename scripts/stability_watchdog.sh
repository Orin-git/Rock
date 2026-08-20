#!/usr/bin/env bash
# Gen1-style pin watchdog: poll topic_health_status file only (no DDS / ros2 CLI).
# Critical pins default: scan + safety_status (+ critical_ok / file freshness).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STATUS_FILE="${STATUS_FILE:-${XW_LOG:-$ROOT/log}/topic_health_status}"
MAX_AGE="${STATUS_MAX_AGE_SEC:-30}"
INTERVAL="${WATCHDOG_INTERVAL_SEC:-15}"
MAX_MISSES="${WATCHDOG_MAX_MISSES:-3}"
# Comma-separated keys that must be "alive" (or critical_ok: 1). Empty = use critical_ok.
CRITICAL_KEYS="${WATCHDOG_CRITICAL_KEYS:-}"
AUTO_RECOVER="${AUTO_RECOVER:-0}"
RECOVER_CMD="${RECOVER_CMD:-}"
LOG_FILE="${WATCHDOG_LOG:-${XW_LOG:-$ROOT/log}/stability_watchdog.log}"

mkdir -p "$(dirname "$STATUS_FILE")" "$(dirname "$LOG_FILE")"

misses=0
recovering=0

log() {
  # Caller should redirect stdout to WATCHDOG_LOG when daemonized.
  echo "[watchdog $(date '+%F %T')] $*"
}

check_critical() {
  local content="$1"
  if [[ -n "$CRITICAL_KEYS" ]]; then
    local key
    IFS=',' read -ra keys <<<"$CRITICAL_KEYS"
    for key in "${keys[@]}"; do
      key="${key// /}"
      [[ -z "$key" ]] && continue
      if ! grep -Eq "^${key}:[[:space:]]*alive$" <<<"$content"; then
        return 1
      fi
    done
    return 0
  fi
  if grep -Eq '^critical_ok:[[:space:]]*1$' <<<"$content"; then
    return 0
  fi
  # Legacy files without critical_ok: require scan + safety_status alive
  grep -Eq '^scan:[[:space:]]*alive$' <<<"$content" \
    && grep -Eq '^safety_status:[[:space:]]*alive$' <<<"$content"
}

do_recover() {
  if [[ "$AUTO_RECOVER" != "1" || -z "$RECOVER_CMD" ]]; then
    return 0
  fi
  if (( recovering )); then
    log "recover already in progress, skip"
    return 0
  fi
  recovering=1
  log "AUTO_RECOVER: $RECOVER_CMD"
  # shellcheck disable=SC2086
  bash -c "$RECOVER_CMD" || log "RECOVER_CMD failed (exit $?)"
  recovering=0
}

log "watching $STATUS_FILE every ${INTERVAL}s max_age=${MAX_AGE}s max_misses=${MAX_MISSES} auto_recover=${AUTO_RECOVER}"

while true; do
  bad=0
  reason=""
  if [[ ! -f "$STATUS_FILE" ]]; then
    bad=1
    reason="missing status file"
  else
    now=$(date +%s)
    mtime=$(stat -c %Y "$STATUS_FILE" 2>/dev/null || stat -f %m "$STATUS_FILE")
    age=$((now - mtime))
    content="$(cat "$STATUS_FILE" 2>/dev/null || true)"
    if (( age > MAX_AGE )); then
      bad=1
      reason="STALE status age=${age}s (monitor hung?)"
    elif ! check_critical "$content"; then
      bad=1
      reason="critical pin dead"
    fi
  fi

  if (( bad )); then
    misses=$((misses + 1))
    log "MISS ${misses}/${MAX_MISSES}: ${reason}"
    if [[ -f "$STATUS_FILE" ]]; then
      log "status:"
      sed 's/^/  /' "$STATUS_FILE" || true
    fi
    if (( misses >= MAX_MISSES )); then
      log "ALERT: ${misses} consecutive misses"
      do_recover
      misses=0
    fi
  else
    if (( misses > 0 )); then
      log "recovered after ${misses} miss(es)"
    fi
    misses=0
  fi
  sleep "$INTERVAL"
done
