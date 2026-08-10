#!/usr/bin/env bash
# 处理 Docker 内触发的同步请求
set -euo pipefail

REPO="${HOME}/ros2_ws"
REQUEST="${REPO}/.git_sync_request"
SYNC="${REPO}/scripts/git_sync.sh"

[ -f "$REQUEST" ] || exit 0
[ -x "$SYNC" ] || exit 0

reason="docker-request"
req_id=""
if grep -q '^reason=' "$REQUEST" 2>/dev/null; then
  reason="$(grep '^reason=' "$REQUEST" | tail -1 | cut -d= -f2-)"
  reason="docker-${reason}"
fi
if grep -q '^request_id=' "$REQUEST" 2>/dev/null; then
  req_id="$(grep '^request_id=' "$REQUEST" | tail -1 | cut -d= -f2-)"
fi

rm -f "$REQUEST"
# 第二参数传 request_id，便于写回 status 匹配
exec "$SYNC" "$reason" "$req_id"
