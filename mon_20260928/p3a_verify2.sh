#!/bin/bash
# P3A 核实·第二批：补齐 camera_info 实参名 / depth 开关 / rgbd_sync 与 rtabmap_odom 的实参
set -u
L=/opt/ros/humble/share/rtabmap_launch/launch/rtabmap.launch.py
echo "=== $(date -u +%FT%TZ)  P3A 核实·第二批 ==="

echo "--- [1] camera_info 相关实参【全部】 ---"
grep -n "camera_info_topic" "$L" | sed 's/^/  /'

echo
echo "--- [2] depth / stereo / subscribe_* 三个总开关的默认值 ---"
grep -n -E "DeclareLaunchArgument\('(depth|stereo|subscribe_depth|subscribe_rgbd|subscribe_rgb|compressed|rgb_image_transport|depth_image_transport|queue_size|Mem|localization)'" "$L" | sed 's/^/  /'

echo
echo "--- [3] 阳性对照 ---"
echo "  camera_info_topic 出现 $(grep -c camera_info_topic "$L") 处；DeclareLaunchArgument 共 $(grep -c DeclareLaunchArgument "$L") 处"

echo
echo "--- [4] rgbd_sync / rtabmap_odom 两个包的 launch 实参（§8-7） ---"
for P in rtabmap_sync rtabmap_odom; do
  echo "  ## ros-humble-$P"
  dpkg -L ros-humble-$P 2>/dev/null | grep -E '\.py$' | while read -r F; do
    echo "     file: $F"
    grep -n "DeclareLaunchArgument" "$F" 2>/dev/null | sed 's/^/       /' | head -30
  done
done

echo
echo "--- [5] 3d 目录 / 地图根现状（贴地检查） ---"
ls -la /ros2_ws/maps/vp/ | sed 's/^/  /'
echo "  --- 3d/ ---"
ls -la /ros2_ws/maps/vp/3d/ 2>/dev/null || echo "  (不存在，符合预期)"
echo "  --- 磁盘 ---"; df -h /ros2_ws | tail -1 | sed 's/^/  /'

echo
echo "--- [6] 录包能力 ---"
timeout 15 ros2 pkg executables rosbag2_transport 2>/dev/null | sed 's/^/  /' || echo "  (读不到)"
ls -la /ros2_ws/scripts/ 2>/dev/null | grep -iE "record|bag" | sed 's/^/  /' || echo "  scripts/ 下无 recorder 命名脚本"

echo
echo "--- [7] /odom 的发布者 QoS（决定 qos_odom 该设几） ---"
source /ros2_ws/scripts/ros_env.sh
timeout 15 ros2 topic info /odom -v 2>/dev/null | head -14 | sed 's/^/  /'

echo
echo "=== 完 $(date -u +%FT%TZ) ==="
