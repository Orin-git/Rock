#!/usr/bin/env bash
# P3A L2 【正式轮 · 握手协议专用】—— 把起点站桩压到最短
#
# 背景（实测）：
#   run_l2.sh 是【核对轮】：sleep 35 固定等待 + 一大堆 param get，起点会站桩 90 秒以上。
#   而日志实测 rtabmap 从节点启动到第一次迭代只要 ~4.5 s (1790584594.377 -> 1790584598.845)。
#   L1 的教训：起点站了整整 90 帧（累计行程 0.00 m），全程 119/223 = 53% 是废帧。
#
# 本脚本只做三件事，其余全部删掉：
#   1) 起 launch（实参逐字同 run_l2.sh，零改动 —— 这是已核对过的配置）
#   2) 【轮询】等 rtabmap 真开始迭代，一有就往下走（不固定 sleep）
#   3) 推之前只验【唯一致命项】：/tf 的 child frame 里不许出现 rgbd_odom
#      （若出现 = TF 撞车，会污染 amcl/nav2 整棵树，必须立刻 stop_l2.sh）
# 其余判据（SFM 原料 local_map_size 等）留到推起来之后用 check_l2_gate.sh 看。
set -u
source /ros2_ws/scripts/ros_env.sh
export LC_ALL=C

TS=$(date -u +%Y%m%d_%H%M%S)
DB=/tmp/p3a_l2go_${TS}.db
LOG=/tmp/p3a_l2go_${TS}.log
echo "$DB"  > /tmp/p3a_l2_current_db
echo "$LOG" > /tmp/p3a_l2_current_log

echo "=== $(date -u +%FT%TZ)  P3A L2 正式轮起跑 ==="
echo "  DB  = $DB"
echo "  LOG = $LOG"
echo "  --- 残留检查（必须为空）---"
ps -eo pid,args --no-headers | grep -a rtabmap | grep -av grep | sed 's/^/    /'
echo "    ^ 空 = 干净"
echo

cd /tmp
nohup ros2 launch /ros2_ws/p3a_l2/rtabmap_l2.launch.py \
  rgb_topic:=/ascamera_hp60c/camera_publisher/rgb0/image \
  depth_topic:=/ascamera_hp60c/camera_publisher/depth0/image_raw \
  camera_info_topic:=/ascamera_hp60c/camera_publisher/rgb0/camera_info \
  depth:=true stereo:=false compressed:=false rgbd_sync:=false \
  visual_odometry:=true icp_odometry:=false \
  publish_tf_odom:=false \
  odom_topic:=rgbd_odom \
  frame_id:=base_link publish_tf_map:=false \
  rtabmap_viz:=false rviz:=false \
  database_path:=$DB \
  approx_sync:=true sync_queue_size:=10 qos:=2 \
  localization:=false wait_for_transform:=0.2 output:=screen >"$LOG" 2>&1 &
LPID=$!
echo "  launch PID=$LPID"

T0=$(date +%s)
for i in $(seq 1 45); do
  grep -aq 'rtabmap (' "$LOG" 2>/dev/null && break
  sleep 1
done
ELAPSED=$(( $(date +%s) - T0 ))
if ! grep -aq 'rtabmap (' "$LOG" 2>/dev/null; then
  echo "  ★★ 等了 ${ELAPSED}s rtabmap 仍未开始迭代 —— 日志末 30 行："
  tail -30 "$LOG" | cut -c1-180 | sed 's/^/    /'
  exit 1
fi
echo "  ★ rtabmap 已开始迭代（等了 ${ELAPSED}s，非固定 sleep）"

echo
echo "  ===== 推前唯一致命判据：/tf 里不许出现 rgbd_odom ====="
echo "  --- /tf child frame 分布 ---"
timeout 6 ros2 topic echo /tf 2>/dev/null | grep -a "child_frame_id" \
  | sort | uniq -c | sort -rn | head -8 | sed 's/^/    /'
echo
if timeout 6 ros2 topic echo /tf 2>/dev/null | grep -aq "child_frame_id: rgbd_odom"; then
  echo "  ★★★ 红了！rgbd_odom 出现在 /tf 里 = TF 撞车。立刻 bash /ros2_ws/p3a_l2/stop_l2.sh"
  exit 1
fi
echo "  ✓ 绿：/tf 里没有 rgbd_odom（只有 base_link / odom / 相机光心帧）"
echo
echo "=== 起跑完成 $(date -u +%FT%TZ)，起跑到现在共 $(( $(date +%s) - T0 ))s ==="
echo "  ⇒ 可以推了。推起来之后我再用 check_l2_gate.sh 验 SFM 原料。"
