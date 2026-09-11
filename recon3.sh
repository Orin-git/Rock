#!/bin/bash
source /ros2_ws/scripts/ros_env.sh >/dev/null 2>&1
echo "### /xw/localization_status x8 over ~12s"
for i in 1 2 3 4 5 6 7 8; do
  printf 't%02d  ' "$i"
  timeout 3 ros2 topic echo --once /xw/localization_status 2>/dev/null | tr -d '\n' | sed 's/  */ /g'
  echo
done
echo
echo "### /xw/nav/goals_blocked x4"
for i in 1 2 3 4; do
  printf 'g%02d  ' "$i"
  timeout 3 ros2 topic echo --once /xw/nav/goals_blocked 2>/dev/null | tr -d '\n'
  echo
done
echo
for t in /xw/localization/phase2c_state /xw/localization/phase2c_loc_state \
         /xw/localization/phase2c_recovery /xw/localization/phase2c_lost_result \
         /xw/localization/phase2c_event /xw/localization/last_good_write; do
  echo "=== $t"
  timeout 8 ros2 topic echo --once "$t" 2>&1 | head -20
done
echo "=== /xw/robot_state"
timeout 8 ros2 topic echo --once /xw/robot_state 2>&1 | head -40
