# Xiaowei Gen2 — optional Dockerfile notes (strengthen later)
# Base (current): osrf/ros:humble-desktop or ros:humble
# Recommended mounts (see setup_ros2_env.sh):
#   --net=host --privileged -v /dev:/dev -v $HOME/ros2_ws:/ros2_ws
# Optional Gen1 reference:
#   -v $HOME/vs_ws1:/vs_ws1:ro
FROM ros:humble

ENV XW_WS=/ros2_ws
WORKDIR /ros2_ws

# Phase0 packages need only base ROS; install extras as needed:
# RUN apt-get update && apt-get install -y \
#   ros-humble-foxglove-bridge \
#   ros-humble-nav2-bringup \
#   ros-humble-slam-toolbox \
#   && rm -rf /var/lib/apt/lists/*

CMD ["bash"]
