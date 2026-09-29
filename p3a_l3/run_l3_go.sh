#!/usr/bin/env bash
# P3A L3 【正式轮】—— 与 L2 唯一的区别：把 rtabmap 的里程计从 rgbd_odometry 解耦到 EKF /odom
#
# ★ 这一轮存在的全部理由（两轮实测）：
#   L2 两轮都在起跑后 16~22 s 死于同一个机制：
#     rgbd_odometry 配准失败 ⇒ 停发里程计 ⇒ Rtabmap.cpp:1411::process() 报
#     "RGB-D SLAM mode is enabled, memory is incremental but no odometry is provided.
#      Image 0 is ignored!" ⇒ rtabmap【丢弃所有图像】（RTAB-Map 耗时 ~100ms → 0.0001ms）
#   L1 用 EKF 的 /odom 跑完 35.12 m / 223 节点，全程【零次】里程计中断。
#   ⇒ L3 = L1 的 odom 接线 + L2 的 SFM，两端解耦（只改 launch 两行，diff 已验）：
#        第 207 行  rgbd_odometry 输出 → /rtabmap/rgbd_odom（不再碰 /odom）
#        第 346 行  rtabmap 输入      → /odom（EKF，永不中断）
#        visual_odometry:=true 保留 ⇒ launch 第 296 行 subscribe_odom_info 仍为 true ⇒ SFM 原料照收
#   ⇒ 即使 rgbd_odometry 再死，rtabmap 也能用 EKF 里程计把这一圈走完，拿到真正的回环判据。
#
# ★★ 握手协议（上一轮我违反了，这里做成结构性闸门）：
#   起跑必须【显式给出用户的放行原话】，否则本脚本拒绝执行：
#       bash run_l3_go.sh --go '准备好了可以推'
#   我没有这句话就不许跑。「授权再推一轮」≠「可以起跑」——把授权当信号是我的错。
set -u
source /ros2_ws/scripts/ros_env.sh
export LC_ALL=C

GO=""
while [ $# -gt 0 ]; do
  case "$1" in
    --go) GO="${2:-}"; shift 2;;
    --no-sampler) SAMPLER=0; shift;;
    *) echo "未知参数: $1"; shift;;
  esac
done
SAMPLER="${SAMPLER:-1}"

if [ -z "$GO" ]; then
  cat <<'EOF'
★★ 拒绝起跑：缺少 --go '用户放行原话'

  本脚本要求把用户【亲口说出】的放行原话逐字传进来，例如：
      bash run_l3_go.sh --go '准备好了可以推'

  为什么：2026-09-28 我犯过一次 —— 用户说「实在不行 就再推一轮」（这是【授权】），
  我把它当成【执行信号】，11 秒后自动起跑，比用户宣告「我开始推了」早了 51 秒，
  于是整轮数据的起点站桩段被污染。授权 ≠ 信号。

  另：真正卡死的是「推」这个动作必须在 rtabmap 稳定迭代【之后】才开始。
  拿到原话后本脚本仍会打完所有安全闸才放行。
EOF
  exit 2
fi

TS=$(date -u +%Y%m%d_%H%M%S)
DB=/tmp/p3a_l3go_${TS}.db
LOG=/tmp/p3a_l3go_${TS}.log
SAMP=/tmp/p3a_l3go_${TS}_odom.txt
echo "$DB"  > /tmp/p3a_l3_current_db
echo "$LOG" > /tmp/p3a_l3_current_log
echo "$SAMP"> /tmp/p3a_l3_current_samp

echo "=== $(date -u +%FT%TZ)  P3A L3 正式轮起跑 ==="
echo "  用户放行原话（逐字）: 「$GO」"
echo "  DB  = $DB"
echo "  LOG = $LOG"
echo
echo "  --- 残留检查（必须为空）---"
ps -eo pid,args --no-headers | grep -a rtabmap | grep -av grep | sed 's/^/    /'
echo "    ^ 空 = 干净"
echo

cd /tmp
nohup ros2 launch /ros2_ws/p3a_l3/rtabmap_l3.launch.py \
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

# ───── 后台采样器：只采非点云/非图像话题（红线 4 允许），每 10 s 一条 ─────
if [ "$SAMPLER" = "1" ]; then
  nohup bash -c '
    S="'"$SAMP"'"
    while :; do
      printf "%s " "$(date -u +%FT%TZ)" >>"$S"
      timeout 6 ros2 topic echo --once /rtabmap/rgbd_odom 2>/dev/null \
        | grep -aE "^  position:|^    [xyz]:" | tr -d " \n" | tr -s " " >>"$S"
      printf " || /odom:" >>"$S"
      timeout 6 ros2 topic echo --once /odom 2>/dev/null \
        | grep -aE "^  position:|^    [xyz]:" | tr -d " \n" >>"$S"
      echo >>"$S"
      sleep 10
    done' >/dev/null 2>&1 &
  echo "  后台采样器已起（每 10s 一条，写 $SAMP）"
fi
echo

echo "  ===== 闸门①【本轮的关键自检】/odom 发布者必须仍然 = 1 ====="
echo "       若变成 2 ⇒ 说明 rgbd_odometry 还在往生产的 /odom 上写 ⇒ 解耦没生效 ⇒ 立刻 stop"
timeout 20 ros2 topic info /odom 2>/dev/null | sed 's/^/    /'
ODOM_PUB=$(timeout 20 ros2 topic info /odom 2>/dev/null | grep -a Publisher | grep -ao '[0-9]\+' | head -1)
if [ "${ODOM_PUB:-x}" != "1" ]; then
  echo "    ★★★ 红了！/odom 发布者 = ${ODOM_PUB:-?}（应为 1）⇒ 立刻 bash /ros2_ws/p3a_l3/stop_l3.sh"
  exit 1
fi
echo "    ✓ 绿：/odom 只有 1 个发布者（生产未被污染）"
echo

echo "  ===== 闸门② rgbd_odometry 的输出真的落在 /rtabmap/rgbd_odom 吗 ====="
timeout 20 ros2 topic list 2>/dev/null | grep -a -E "rgbd_odom|odom_info" | sed 's/^/    /'
if timeout 15 ros2 topic hz /rtabmap/rgbd_odom 2>/dev/null | head -2 | grep -aq "average rate"; then
  timeout 15 ros2 topic hz /rtabmap/rgbd_odom 2>/dev/null | head -2 | sed 's/^/    /'
  echo "    ✓ 绿：/rtabmap/rgbd_odom 有数据（Odometry 消息，非点云/图像，红线 4 允许）"
else
  echo "    ★ 黄：/rtabmap/rgbd_odom 没测到速率 —— 再等一下看第 ③ 闸"
fi
echo

echo "  ===== 闸门③ /tf 里不许出现 rgbd_odom（TF 撞车 = 污染 amcl/nav2 整棵树）====="
timeout 6 ros2 topic echo /tf 2>/dev/null | grep -a "child_frame_id" \
  | sort | uniq -c | sort -rn | head -8 | sed 's/^/    /'
if timeout 6 ros2 topic echo /tf 2>/dev/null | grep -aq "child_frame_id: rgbd_odom"; then
  echo "    ★★★ 红了！rgbd_odom 出现在 /tf 里 ⇒ 立刻 bash /ros2_ws/p3a_l3/stop_l3.sh"
  exit 1
fi
echo "    ✓ 绿：/tf 里没有 rgbd_odom"
echo

echo "  ===== 闸门④ SFM 的原料（OdomInfo 的 local_map_size；红线 4：只读前 40 行不做点云 echo）====="
OI=$(timeout 15 ros2 topic echo --once /rtabmap/odom_info 2>/dev/null | head -40)
for F in lost matches inliers features local_map_size type distance_travelled; do
  printf "    %-20s = " "$F"
  echo "$OI" | grep -a -m1 "^$F:" | sed "s/^$F: *//" || echo "(不在前40行)"
done
echo "    ^ local_map_size=0 且 features>0 ⇒ F2M 没建起 3D ⇒ SFM 无原料"
echo
echo "=== 起跑完成 $(date -u +%FT%TZ)，起跑到现在共 $(( $(date +%s) - T0 ))s ==="
echo "  ⇒ 现在可以推了。收尾用 bash /ros2_ws/p3a_l3/stop_l3.sh"
echo "  ⇒ 推【一整圈】。本轮判据：DB 里 type=1 且 |from_id-to_id|>50 的边数 > 0（=真回环）。"
