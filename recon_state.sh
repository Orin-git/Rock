#!/bin/bash
source /ros2_ws/scripts/ros_env.sh >/dev/null 2>&1
for t in /xw/localization_status /xw/nav/goals_blocked /xw/chassis/motor_disabled \
         /xw/chassis/charge_mode /xw/localization/phase2c_state \
         /xw/localization/phase2c_loc_state /xw/localization/phase2c_recovery; do
  echo "=== $t"
  timeout 3 ros2 topic echo --once "$t" 2>&1 | head -14
done
echo "=== /xw/robot_state"
timeout 3 ros2 topic echo --once /xw/robot_state 2>&1 | head -30
echo "=== /amcl_pose (once)"
timeout 4 ros2 topic echo --once /amcl_pose 2>&1 | head -14
