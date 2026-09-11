#!/usr/bin/env bash
# Stop the Phase2D visual-DB build node. Run from a FILE so this shell's own
# command line does not contain the pattern (pkill -f would otherwise match it
# and kill the calling shell before it can report anything).
set -u
PAT='xw_visual_db_build'
found=0
for p in $(pgrep -f "$PAT" || true); do
  [ "$p" = "$$" ] && continue
  [ "$p" = "$PPID" ] && continue
  if kill "$p" 2>/dev/null; then
    echo "SIGTERM -> pid $p"
    found=1
  fi
done
sleep 3
still=$(pgrep -f "$PAT" || true)
if [ -n "$still" ]; then
  echo "still alive after SIGTERM: $still -> escalating to SIGKILL"
  for p in $still; do
    [ "$p" = "$$" ] && continue
    [ "$p" = "$PPID" ] && continue
    kill -9 "$p" 2>/dev/null && echo "SIGKILL -> pid $p"
  done
  sleep 2
fi
if pgrep -f "$PAT" >/dev/null 2>&1; then
  echo "RESULT: STILL RUNNING"
  exit 1
fi
echo "RESULT: BUILD NODE STOPPED"
