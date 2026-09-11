#!/bin/bash
source /ros2_ws/scripts/ros_env.sh >/dev/null 2>&1
echo "### topic info (pub/sub counts)"
for t in /xw/localization_status /xw/nav/goals_blocked /xw/chassis/motor_disabled \
         /xw/robot_state /amcl_pose /cmd_vel /cmd_vel_nav /xw/cmd/nav /xw/nav/enable /odom; do
  printf '%-36s ' "$t"
  timeout 6 ros2 topic info "$t" 2>&1 | tr '\n' ' '
  echo
done
echo
echo "### freshness: two consecutive --once reads (identical => latched/stale)"
for t in /xw/localization_status /xw/nav/goals_blocked /xw/chassis/motor_disabled; do
  echo "--- $t"
  timeout 6 ros2 topic echo --once "$t" 2>&1 | head -6
  timeout 6 ros2 topic echo --once "$t" 2>&1 | head -6
done
echo
echo "### tf map -> base_link"
timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -14
