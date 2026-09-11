#!/bin/bash
# Poll the live build session until round 1 ends (round=2 planned) or the
# session goes IDLE, then print the Step 2 acceptance table.
LOG=/ros2_ws/log/build_status_trace.log
for i in $(seq 1 120); do
  sleep 30
  if grep -qE 'PLANNING.*round=2' "$LOG"; then echo "=== ROUND 2 REACHED ==="; break; fi
  if tail -1 "$LOG" | grep -q 'state=IDLE'; then echo "=== SESSION ENDED ==="; break; fi
done
source /ros2_ws/scripts/ros_env.sh >/dev/null 2>&1
python3 /ros2_ws/acceptance_report.py
