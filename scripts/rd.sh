source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=99
for t in /xw/localization_status /xw/localization/phase2c_task_snapshot /xw/localization/phase2c_event; do
  echo "### $t"
  timeout 8 ros2 topic echo --once "$t" 2>&1 | head -25
  echo
done
echo "### types"
for t in /xw/localization_status /xw/localization/phase2c_task_snapshot; do
  timeout 8 ros2 topic info -v "$t" 2>&1 | grep -E "Type|Publisher count|Reliability|Durability" | head -6
  echo
done
