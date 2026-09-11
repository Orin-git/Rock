#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=99
cd /ros2_ws
exec python3 scripts/diag_reachability.py
