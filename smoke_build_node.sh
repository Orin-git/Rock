#!/bin/bash
# End-to-end smoke test of the patched build orchestrator, in its real role.
#
# Proves the node still starts, publishes status, and answers its Trigger service
# AFTER the :411 (status_dict lock split) and motion-subs patches -- the isolated
# probes construct the class, but this exercises the shipped service path.
#
# Deliberately sends NO build command: `start` would launch a session and drive
# the robot, which is charging. Only the read-only status service is called.
#
# Cleanup uses explicit PIDs collected from ps (never pkill -f, which matches the
# caller's own cmdline).
set -u
source /ros2_ws/scripts/ros_env.sh >/dev/null 2>&1
cd /ros2_ws
LOG=/ros2_ws/log/phase2d_c2/build_smoke.log
mkdir -p /ros2_ws/log/phase2d_c2
: > "$LOG"

nohup ros2 run xw_global_reloc xw_visual_db_build >>"$LOG" 2>&1 &
LAUNCH_PID=$!
echo "STARTED launcher_pid=$LAUNCH_PID"
sleep 12

mapfile -t PIDS < <(ps -eo pid,cmd | grep -E "xw_visual_db_build" | grep -v grep | awk '{print $1}')
echo "MATCHED PIDS: ${PIDS[*]:-<none>}"
if [ "${#PIDS[@]}" -eq 0 ]; then
  echo "RESULT: FAIL - node process not found"
  exit 1
fi
echo "ALIVE: yes (${#PIDS[@]} process)"

echo "--- service call ---"
timeout -k 5 30 ros2 service call /xw/visual_db/build_status_svc std_srvs/srv/Trigger 2>&1 | tail -4

echo "--- node log ---"
grep -E "ready|ERROR|Traceback" "$LOG" | tail -4

echo "--- cleanup ---"
for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null; done
kill -TERM "$LAUNCH_PID" 2>/dev/null
sleep 4
STILL=""
for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && STILL="$STILL $p"; done
if [ -n "$STILL" ]; then
  echo "SIGTERM ignored by:$STILL - sending SIGKILL"
  for p in $STILL; do kill -9 "$p" 2>/dev/null; done
  sleep 1
fi
FINAL=""
for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && FINAL="$FINAL $p"; done
if [ -n "$FINAL" ]; then echo "RESULT: FAIL - survived SIGKILL:$FINAL"; exit 1; fi
echo "STOPPED CLEANLY (no respawn)"
