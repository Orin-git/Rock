#!/bin/bash
# 容器内启动采样器（docker exec -d 调用，避免嵌套引号吞载荷）
source /ros2_ws/scripts/ros_env.sh
exec python3 /ros2_ws/mon_20260928/sampler.py >> /ros2_ws/mon_20260928/sampler.log 2>&1
