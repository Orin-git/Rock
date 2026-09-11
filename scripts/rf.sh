source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=99
exec python3 /ros2_ws/scripts/diag_frames.py
