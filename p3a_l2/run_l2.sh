#!/usr/bin/env bash
# P3A L2 —— 在 L1 基础上【只多一个机制】：Mem/StereoFromMotion true
#   + 起 rtabmap 自家里程计（rgbd_odometry）
#   + ★★ 两个安全闸：publish_tf_odom:=false（防 TF 撞车）/ odom_topic:=rgbd_odom（防 /odom 撞车）
# 其余实参逐字同 L1（run_l1.sh），DB/LOG 每次换新名。
set -u
source /ros2_ws/scripts/ros_env.sh
export LC_ALL=C

TS=$(date -u +%Y%m%d_%H%M%S)
DB=/tmp/p3a_l2_${TS}.db
LOG=/tmp/p3a_l2_${TS}.log
echo "$DB"  > /tmp/p3a_l2_current_db
echo "$LOG" > /tmp/p3a_l2_current_log

echo "=== $(date -u +%FT%TZ)  P3A L2 起跑 ==="
echo "  DB  = $DB"
echo "  LOG = $LOG"
echo

echo "  ===== ★ 起跑前基线（用于跑后对比，证明没打断生产）====="
echo "  --- rtabmap 残留检查（必须为空）---"
ps -eo pid,args --no-headers | grep -a rtabmap | grep -av grep | sed 's/^/    /' || true
echo "    ^ 空 = 干净"
echo "  --- /odom 发布者数（★ 跑完必须还是这个数）---"
timeout 20 ros2 topic info /odom 2>/dev/null | sed 's/^/    /' || echo "    (拿不到)"
echo "  --- /tf 发布者数（★ 注意：跑中会从 6 涨到 8，因为 rtabmap/rgbd_odometry 会【注册】publisher 对象）---"
echo "      注册≠发布：真正判据是 /tf 里出不出 rgbd_odom 这个 child frame，见 check_l2_gate.sh"
timeout 20 ros2 topic info /tf 2>/dev/null | grep -a Publisher | sed 's/^/    /' || echo "    (拿不到)"
echo "  --- 定位闸门（★ 真名是 /xw/localization_status；/loc_status 是【不存在】的话题）---"
echo "      0 = 正常；--once 拿的是闩锁保留帧，只作快照不当计数器"
for T in /xw/localization_status /xw/nav/goals_blocked; do
  printf "      %-32s = " "$T"
  timeout 15 ros2 topic echo --once --qos-reliability reliable "$T" 2>/dev/null | grep -a "data:" | head -1 || echo "(拿不到)"
done
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
echo "  等 35 s 让三个节点起来（rtabmap + rgbd_odometry）……"
sleep 35

RPID=$(pgrep -f 'rtabmap_slam/rtabmap' | head -1)
OPID=$(pgrep -f 'rtabmap_odom/rgbd_odometry' | head -1)
echo
echo "  rtabmap PID        = ${RPID:-none}"
echo "  rgbd_odometry PID  = ${OPID:-none}"
if [ -z "${RPID:-}" ]; then
  echo "  ★★ rtabmap 没起来 —— 日志末 40 行："
  tail -40 "$LOG" | sed 's/^/    /'
  exit 1
fi

echo
echo "  ===== ★★★ 安全闸 1：【决定性的】rgbd_odometry 不许发 TF ====="
if [ -n "${OPID:-}" ]; then
  echo "  --- /proc/$OPID/cmdline（NUL 分隔，验分词）---"
  tr '\0' '\n' < "/proc/$OPID/cmdline" | grep -a . | sed 's/^/    /'
fi
printf "    publish_tf      = "
timeout 20 ros2 param get /rtabmap/rgbd_odometry publish_tf 2>&1 | tail -1
printf "    odom_frame_id   = "
timeout 20 ros2 param get /rtabmap/rgbd_odometry odom_frame_id 2>&1 | tail -1
printf "    OdomF2M/ValidDepthRatio = "
timeout 20 ros2 param get /rtabmap/rgbd_odometry OdomF2M/ValidDepthRatio 2>&1 | tail -1

echo
echo "  ===== ★★★ 安全闸 2：/odom 与 /tf 的发布者数【必须与基线相同】====="
echo "  --- /odom（应仍只有 ekf_filter_node）---"
timeout 20 ros2 topic info /odom 2>/dev/null | sed 's/^/    /' || echo "    (拿不到)"
echo "  --- /tf 发布者 ---"
timeout 20 ros2 topic info /tf 2>/dev/null | grep -a Publisher | sed 's/^/    /' || echo "    (拿不到)"
echo "  --- 全树有没有第二个 odom->base_link 发布者（查 rtabmap 是否发 TF）---"
timeout 20 ros2 topic info /tf --verbose 2>/dev/null | grep -a -i "Node name" | sed 's/^/    /' || true

echo
echo "  ===== ★ rtabmap 侧：SFM 生效 + subscribe_odom_info 打开 ====="
printf "    Mem/StereoFromMotion   = "
timeout 20 ros2 param get /rtabmap/rtabmap Mem/StereoFromMotion 2>&1 | tail -1
printf "    Vis/DepthAsMask        = "
timeout 20 ros2 param get /rtabmap/rtabmap Vis/DepthAsMask 2>&1 | tail -1
printf "    Mem/UseOdomFeatures    = "
timeout 20 ros2 param get /rtabmap/rtabmap Mem/UseOdomFeatures 2>&1 | tail -1
echo "  --- 日志里的生效行（决定性）---"
grep -a "from arguments" "$LOG" | sed 's/^/    /'
grep -a "subscribe_odom_info" "$LOG" | head -3 | sed 's/^/    /'

echo
echo "  ===== ★ 新话题在不在 ====="
timeout 20 ros2 topic list 2>/dev/null | grep -a -E "rgbd_odom|odom_info" | sed 's/^/    /' || echo "    ★ 没看到 rgbd_odom / odom_info"
echo "  --- /rtabmap/rgbd_odom 有没有在出数据（Odometry 消息，非点云/图像，红线 4 允许）---"
timeout 15 ros2 topic hz /rtabmap/rgbd_odom 2>&1 | head -3 | sed 's/^/    /' || true

echo
echo "  ===== 节点清单 ====="
timeout 25 ros2 node list 2>/dev/null | grep -a -i rtabmap | sed 's/^/    /' || echo "    (拿不到)"
sleep 2
timeout 25 ros2 node list 2>/dev/null | grep -a -i rtabmap | sed 's/^/    /' || true

echo
echo "  ===== 日志头 40 行 ====="
head -40 "$LOG" | cut -c1-200 | sed 's/^/    /'
echo
echo "=== 起跑完成 $(date -u +%FT%TZ) ==="
echo "  下一步：上面三处安全闸全绿 ⇒ 说「可以推了」；收尾用 bash /ros2_ws/p3a_l2/stop_l2.sh"
