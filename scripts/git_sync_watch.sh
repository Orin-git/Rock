#!/usr/bin/env bash
# 处理 Docker 内触发的同步请求
set -euo pipefail

REPO="${HOME}/ros2_ws"
REQUEST="${REPO}/.git_sync_request"
SYNC="${REPO}/scripts/git_sync.sh"

[ -f "$REQUEST" ] || exit 0
[ -x "$SYNC" ] || exit 0

reason="docker-request"
if grep -q '^reason=' "$REQUEST" 2>/dev/null; then
  reason="$(grep '^reason=' "$REQUEST" | tail -1 | cut -d= -f2-)"
  reason="docker-${reason}"
fi

rm -f "$REQUEST"
exec "$SYNC" "$reason"
