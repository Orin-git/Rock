#!/usr/bin/env bash
# P3A L2 正式轮 —— 跑中随时可调的核对闸
#
# 立意：L2 有两个隐患，必须在【运动开始后的头 1~2 分钟】就判出来，
#       否则会白推一整轮：
#   隐患A  publish_tf_odom:=false 让 rgbd_odom 帧不在 TF 树里
#          ⇒ rtabmap 报 "cannot get the corresponding TF rgbd_odom->base_link ...
#            to get more accurate pose estimation"（它自称是【精度】问题）
#          ⇒ 判据①：/tf 里【不许出现】rgbd_odom 这个 child frame（出现=撞车，必须立刻停）
#   隐患B  SFM 没有原料 ⇒ Mem/StereoFromMotion 名存实亡
#          ⇒ 判据③：/rtabmap/odom_info 的 local_map_size（F2M 维护的 3D 点地图）
#            与 features。local_map_size=0 且 features>0 ⇒ F2M 没建起 3D ⇒ SFM 无原料
#
# ★ 另注：/loc_status 是【不存在】的话题名，真名是 /xw/localization_status（上过一次当，别再踩）
set -u
source /ros2_ws/scripts/ros_env.sh
export LC_ALL=C

echo "=== $(date -u +%FT%TZ)  P3A L2 跑中核对 ==="
echo

echo "① ★★★ /tf child frame 分布（决定性：出现 rgbd_odom = TF 撞车，立刻停）"
timeout 8 ros2 topic echo /tf 2>/dev/null | grep -a "child_frame_id" \
  | sort | uniq -c | sort -rn | head -12 | sed 's/^/    /'
echo "    ^ 只应看到 base_link / odom / ascamera*_color_0；出现 rgbd_odom 就是红的"
echo

echo "② 定位闸门（真名 /xw/localization_status；0 = 正常）"
for T in /xw/localization_status /xw/nav/goals_blocked /xw/localization/phase2c_loc_state; do
  printf "    %-36s = " "$T"
  timeout 15 ros2 topic echo --once --qos-reliability reliable "$T" 2>/dev/null \
    | grep -a "data:" | head -1 || echo "(拿不到)"
done
echo

echo "③ ★★★ SFM 原料：rgbd_odometry 的 F2M 局部地图（local_map_size>0 = 有原料）"
# local_map_size / features 都在 OdomInfo 前 30 行内；后面的 local_bundle_models 极大，必须截断
OI=$(timeout 15 ros2 topic echo --once /rtabmap/odom_info 2>/dev/null | head -40)
for F in lost matches inliers features local_map_size local_key_frames type distance_travelled; do
  printf "    %-20s = " "$F"
  echo "$OI" | grep -a -m1 "^$F:" | sed "s/^$F: *//" || echo "(不在前40行)"
done
echo "    ^ 单项全空 = OdomInfo 没内容；local_map_size 长期 0 = F2M 没建起 3D"
echo

echo "④ 生产栈（应与基线相同：/odom 发布者=1）"
timeout 20 ros2 topic info /odom 2>/dev/null | grep -a Publisher | sed 's/^/    \/odom: /'
timeout 20 ros2 topic info /tf   2>/dev/null | grep -a Publisher | sed 's/^/    \/tf:   /'
echo "    ^ /tf 的 8 是【预期】(rtabmap+rgbd_odometry 注册了 publisher 对象)；判据看①"
echo

echo "⑤ rtabmap 还活着吗 / 有没有在迭代"
L=$(cat /tmp/p3a_l2_current_log 2>/dev/null || echo "")
echo "    rtabmap PID = $(pgrep -f 'rtabmap_slam/rtabmap' | head -1)"
if [ -n "$L" ] && [ -f "$L" ]; then
  echo "    rtabmap 迭代次数 = $(grep -ac 'rtabmap (' "$L")   'Did not receive data' = $(grep -ac 'Did not receive data' "$L")"
  echo "    --- 末 2 条迭代 ---"
  grep -a 'rtabmap (' "$L" | tail -2 | cut -c1-150 | sed 's/^/      /'
else
  echo "    (日志指针 /tmp/p3a_l2_current_log 不在)"
fi
echo
echo "=== 完 $(date -u +%FT%TZ) ==="
