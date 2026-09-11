#!/bin/bash
# Kill the Phase2D build nodes by explicit PID. Never pkill -f: it matches the
# caller's own cmdline and has already self-terminated a shell once.
# Runs INSIDE the container, so `ps` here is the container's PID namespace.
set -u
PAT='xw_visual_db_build|xw_visual_db_capture'
PIDS=$(ps -eo pid,args | grep -E "$PAT" | grep -v grep | grep -v kill_build_nodes | awk '{print $1}')
if [ -z "$PIDS" ]; then
  echo "NO build/capture node running"
  exit 0
fi
echo "killing: $PIDS"
for p in $PIDS; do
  kill "$p" 2>/dev/null && echo "  sent TERM to $p"
done
sleep 3
LEFT=$(ps -eo pid,args | grep -E "$PAT" | grep -v grep | grep -v kill_build_nodes | awk '{print $1}')
if [ -n "$LEFT" ]; then
  echo "still alive after TERM: $LEFT -- sending KILL"
  for p in $LEFT; do kill -9 "$p" 2>/dev/null && echo "  sent KILL to $p"; done
  sleep 1
fi
echo "=== remaining ==="
ps -eo pid,args | grep -E "$PAT" | grep -v grep | grep -v kill_build_nodes || echo "  (none)"
