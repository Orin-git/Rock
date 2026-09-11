#!/bin/bash
# Read-only watcher: samples the canonical incident stream while the operator
# re-seeds. Touches nothing. Writes to /ros2_ws/log/operator_pose_watch.log
source /ros2_ws/scripts/ros_env.sh >/dev/null 2>&1
LOG=/ros2_ws/log/operator_pose_watch.log
: > "$LOG"
for i in $(seq 1 220); do
  printf '%s  event=' "$(date +%H:%M:%S)" >> "$LOG"
  timeout 3 ros2 topic echo --once /xw/localization/phase2c_event 2>/dev/null \
    | tr -d '\n' | tr -s ' ' >> "$LOG"
  printf '  blocked=' >> "$LOG"
  timeout 3 ros2 topic echo --once /xw/nav/goals_blocked 2>/dev/null \
    | tr -d '\n' | tr -s ' ' >> "$LOG"
  printf '  loc=' >> "$LOG"
  timeout 3 ros2 topic echo --once /xw/localization_status 2>/dev/null \
    | tr -d '\n' | tr -s ' ' >> "$LOG"
  printf '  build=' >> "$LOG"
  tail -1 /ros2_ws/log/build_status_trace.log >> "$LOG"
  sleep 2
done
echo "WATCHER DONE" >> "$LOG"
