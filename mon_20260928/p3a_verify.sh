#!/bin/bash
# P3A 装包后核实 —— 只读，落掉方案 §4.2/§4.5 的「未核实项」
# 目的：① 拿到 deb 内 rtabmap.launch.py 的【全部实参真名】（不是 GitHub 上那份）
#       ② 核 qos_image / qos_camera_info 默认值  ③ 核 depth_camera_info 参数真名
#       ④ 核 rgbd_sync / rtabmap_odom 的参数（§8 第7条）
set -u
L=/opt/ros/humble/share/rtabmap_launch/launch/rtabmap.launch.py
echo "=== $(date -u +%FT%TZ)  P3A 装包后核实 ==="
echo "--- [0] 文件身份（证明读的是 deb 里的，不是 GitHub） ---"
ls -la "$L"; md5sum "$L"
dpkg -S "$L" 2>/dev/null

echo
echo "--- [1] 目标实参真名：rgb / depth / camera_info 全部 ---"
grep -n -E "DeclareLaunchArgument\('(rgb_topic|depth_topic|camera_info|depth_camera_info|rgbd_sync_topic|qos|qos_image|qos_camera_info|subscribe_rgb|subscribe_depth|subscribe_scan|frame_id|map_frame_id|odom_frame_id|publish_tf|publish_tf_map|database_path|localization|wait_for_transform|approx_sync|sync_queue_size)'" "$L" \
  | sed 's/^/  /'

echo
echo "--- [2] 阳性对照（必须命中；为 0 说明我 grep 写错了） ---"
for K in frame_id database_path approx_sync; do
  printf "  %-22s %s 处\n" "$K" "$(grep -c "$K" "$L")"
done

echo
echo "--- [3] qos 相关：声明 + 用在哪 ---"
grep -n -i "qos" "$L" | head -25 | sed 's/^/  /'

echo
echo "--- [4] 彩色/深度订阅点在哪一行（看 topic 是怎么接的） ---"
grep -n -E "\"rgb/image\"|\"depth/image\"|\"rgb/camera_info\"|\"depth/camera_info\"|rgb_topic|depth_topic|camera_info" "$L" | head -20 | sed 's/^/  /'

echo
echo "--- [5] §8-7：rgbd_sync / rtabmap_odom 两个包的 launch 实参 ---"
for P in rtabmap_sync rtabmap_odom; do
  echo "  ## $P"
  for F in $(dpkg -L ros-humble-$P 2>/dev/null | grep -E '\.py$'); do
    echo "     -- $F"
    grep -n "DeclareLaunchArgument" "$F" 2>/dev/null | sed 's/^/        /' | head -25
  done
done

echo
echo "--- [6] 3d 目录现状（贴地检查：绝不动 2d 那部分） ---"
ls -la /ros2_ws/maps/vp/ 2>/dev/null | sed 's/^/  /'
echo "  --- 3d/ ---"
ls -la /ros2_ws/maps/vp/3d/ 2>/dev/null || echo "  (不存在，符合预期：尚未创建)"

echo
echo "--- [7] 录包能力（§4.4 第1步 / §8 末条） ---"
which ros2 2>/dev/null && ros2 pkg executables rosbag2_transport 2>/dev/null | head -5
ls -la /ros2_ws/scripts/ 2>/dev/null | grep -i -E "record|bag" | sed 's/^/  /' || echo "  (scripts/ 下无 recorder 脚本名)"
# 已有的录制脚本（memory 提到已固化 run_recorder.sh，可能在别处）
find /ros2_ws -maxdepth 3 -name "*record*" -o -maxdepth 3 -name "*recorder*" 2>/dev/null | head -10

echo
echo "=== 完 $(date -u +%FT%TZ) ==="
