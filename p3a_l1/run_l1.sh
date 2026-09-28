#!/usr/bin/env bash
# P3A L1 —— 单变量实验：Vis/DepthAsMask + Mem/DepthAsMask  true -> false
# 其余 15 个实参逐字同 /home/radxa/p3a_export_20260928/p3a_run3.sh:125-136
# launch 用打过补丁的副本 /ros2_ws/p3a_l1/rtabmap_l1.launch.py（与原版只差 1 行）
set -u
source /ros2_ws/scripts/ros_env.sh
export LC_ALL=C

TS=$(date -u +%Y%m%d_%H%M%S)
DB=/tmp/p3a_l1_${TS}.db
LOG=/tmp/p3a_l1_${TS}.log
echo "$DB"  > /tmp/p3a_l1_current_db
echo "$LOG" > /tmp/p3a_l1_current_log

echo "=== $(date -u +%FT%TZ)  P3A L1 起跑 ==="
echo "  DB  = $DB"
echo "  LOG = $LOG"
echo
echo "  --- 起跑前：rtabmap 残留检查（必须为空）---"
ps -eo pid,args --no-headers | grep -a rtabmap | grep -av grep | sed 's/^/    /' || true
echo "    ^ 空 = 干净"
echo
echo "  --- 起跑前：相机话题在不在 ---"
timeout 15 ros2 topic list 2>/dev/null | grep -a ascamera | sed 's/^/    /' || echo "    (话题列表拿不到)"
echo

cd /tmp
nohup ros2 launch /ros2_ws/p3a_l1/rtabmap_l1.launch.py \
  rgb_topic:=/ascamera_hp60c/camera_publisher/rgb0/image \
  depth_topic:=/ascamera_hp60c/camera_publisher/depth0/image_raw \
  camera_info_topic:=/ascamera_hp60c/camera_publisher/rgb0/camera_info \
  depth:=true stereo:=false compressed:=false rgbd_sync:=false \
  visual_odometry:=false icp_odometry:=false odom_topic:=/odom \
  frame_id:=base_link publish_tf_map:=false \
  rtabmap_viz:=false rviz:=false \
  database_path:=$DB \
  approx_sync:=true sync_queue_size:=10 qos:=2 \
  localization:=false wait_for_transform:=0.2 output:=screen >"$LOG" 2>&1 &
LPID=$!
echo "  launch PID=$LPID"
echo "  等 30 s 让节点起来……"
sleep 30

RPID=$(pgrep -f 'rtabmap_slam/rtabmap' | head -1)
echo
echo "  rtabmap PID = ${RPID:-none}"
if [ -z "${RPID:-}" ]; then
  echo "  ★★ rtabmap 没起来 —— 日志末 30 行："
  tail -30 "$LOG" | sed 's/^/    /'
  exit 1
fi

echo
echo "  ===== ★ 决定性验证 1：argv 逐 token（/proc/$RPID/cmdline，NUL 分隔）====="
tr '\0' '\n' < "/proc/$RPID/cmdline" | grep -a . | sed 's/^/    /'

echo
echo "  ===== ★ 决定性验证 2：节点名 ====="
timeout 25 ros2 node list 2>/dev/null | grep -a -i rtabmap | sed 's/^/    /' || echo "    (拿不到)"
sleep 2
timeout 25 ros2 node list 2>/dev/null | grep -a -i rtabmap | sed 's/^/    /' || true

echo
echo "  ===== ★ 决定性验证 3：活参数回读 ====="
for P in Vis/DepthAsMask Mem/DepthAsMask Vis/FeatureType Vis/MaxFeatures; do
  printf "    %-22s " "$P"
  timeout 20 ros2 param get /rtabmap/rtabmap "$P" 2>&1 | tail -1
done

echo
echo "  ===== 日志头 30 行 ====="
head -30 "$LOG" | sed 's/^/    /'
echo
echo "=== 起跑完成 $(date -u +%FT%TZ) ==="
echo "  下一步：确认上面三处验证通过后，手推一圈（同上次路线，约 36 m，不许站桩）"
echo "  收尾用：bash /ros2_ws/p3a_l1/stop_l1.sh"
