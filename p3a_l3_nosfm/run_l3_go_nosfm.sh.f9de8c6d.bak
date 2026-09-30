#!/usr/bin/env bash
# P3A L3【正式轮 · PROTOCOL v2】—— 与 v1 的 launch/实参【逐字相同】，只改【起跑协议】
#
# ★ v1（2026-09-28）实测暴露的问题：起跑到推之间有 33 s 站桩，造出 41 个位姿完全相同的
#   起点节点（0.00 m）。这直接导致昨天定案的「回环=0」真机制：
#     (a) 站桩 ⇒ SFM 无运动可依 ⇒ 起点/终点每帧只有 48~71 个 3D（中段 296~378）⇒ 回环两端匹配不上
#     (b) 同一位置连造 41 个同姿态节点 ⇒ 一条回到该簇的回环让 GTSAM 秩亏
#         ⇒ 「Graph optimization failed! Rejecting last loop closures added.」
#   ⇒ 结论：不要在起点/终点站桩。见记忆 robot189-loop-closure-gtsam-rollback。
#
# ★ v2 的三处改动：
#   1) 推车前等待 33 s → 目标 <10 s：安全闸改用【argv 级判据】。
#      昨天已实测证明它是决定性的：remap 一旦落到节点 argv（`odom:=/rtabmap/rgbd_odom`），
#      中间件层就不可能让 rgbd_odometry 再往 /odom 写。话题级核对（发布者计数、/tf）
#      是【确认】不是【判据】，挪到推起来之后做。
#   2) 采样器从坏的 shell 版换成 rclpy 版（odom_sampler.py）：shell 版两个 bug ——
#      grep 缩进写错 + `ros2 topic echo --once` 在 daemon 冷启动时 >12 s 被 timeout 杀掉。
#   3) 打印【回程指令】：到起点不要停，继续推过去 5~10 m。
#
# ★★ 握手协议（v1 起就有，保留）：起跑必须逐字给出用户的放行原话，否则拒绝执行。
#      bash run_l3_go.sh --go '准备好了可以推'
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
      bash run_l3_go.sh --go '准备好了可以推'
  授权 ≠ 信号。2026-09-28 我把「实在不行 就再推一轮」（授权）当成执行信号，
  11 秒后自动起跑，比用户放行早了 51 秒，白推一轮。
EOF
  exit 2
fi

TS=$(date -u +%Y%m%d_%H%M%S)
DB=/tmp/p3a_l3go_${TS}.db
LOG=/tmp/p3a_l3go_${TS}.log
SAMP=/tmp/p3a_l3go_${TS}_odom.txt
GATE=/tmp/p3a_l3go_${TS}_gate.txt
echo "$DB"  > /tmp/p3a_l3_current_db
echo "$LOG" > /tmp/p3a_l3_current_log
echo "$SAMP"> /tmp/p3a_l3_current_samp

echo "=== $(date -u +%FT%TZ)  P3A L3 正式轮 PROTOCOL=v2 起跑 ==="
echo "  用户放行原话（逐字）: 「$GO」"
echo "  DB=$DB"
echo "  LOG=$LOG"
ps -eo pid,args --no-headers | grep -a rtabmap | grep -av grep | sed 's/^/  [残留] /'
echo

# ── 预热 ROS daemon：让后面的【确认】阶段的话题查询从 ~20 s 降到 ~2 s ──
#    （v1 的 33 s 里绝大部分是 daemon 冷启动等掉的）
( timeout 60 ros2 topic list >/dev/null 2>&1 ) &
WARM=$!

cd /tmp
nohup ros2 launch /ros2_ws/p3a_l3_nosfm/rtabmap_l3.launch.py \
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
T0=$(date +%s)
echo "  launch PID=$LPID   $(date -u +%FT%TZ)"

for i in $(seq 1 45); do
  grep -aq 'rtabmap (' "$LOG" 2>/dev/null && break
  sleep 1
done
if ! grep -aq 'rtabmap (' "$LOG" 2>/dev/null; then
  echo "  ★★ 等了 $(( $(date +%s) - T0 ))s rtabmap 仍未迭代 —— 日志末 30 行："
  tail -30 "$LOG" | cut -c1-180 | sed 's/^/    /'
  exit 1
fi
echo "  ★ rtabmap 已开始迭代（等了 $(( $(date +%s) - T0 ))s）"
echo

# ═══════════ ★★★ 瞬时安全闸（argv 级，决定性，~0 秒）═══════════
RP=$(pgrep -f 'rtabmap_slam/rtabmap' | head -1)
OP=$(pgrep -f 'rtabmap_odom/rgbd_odometry' | head -1)
if [ -z "${RP:-}" ] || [ -z "${OP:-}" ]; then
  echo "  ★★★ 节点没起全（rtabmap=${RP:-none} rgbd_odometry=${OP:-none}）—— 终止"
  exit 1
fi
RARGV=$(tr '\0' '\n' < "/proc/$RP/cmdline")
OARGV=$(tr '\0' '\n' < "/proc/$OP/cmdline")

# ★ 判据分两类，别混（这是 09-28 差点犯的错）：
#   remap（`odom:=...`，launch 的 remappings=[...]）⇒ 一定在 argv ⇒ 【决定性判据】
#   参数（`publish_tf` / `subscribe_odom_info`，launch 的 parameters=[{...}]）⇒ 走 --params-file，
#        argv 里【根本不会有】⇒ 只看 argv 会误报红、把好的一轮挡掉。必须双查。
PFILES=$(printf '%s\n%s\n' "$OARGV" "$RARGV" | grep -aE 'launch_params|\.ya?ml$' | sed 's/^--params-file=//' | sort -u)
param_all() {   # $1=参数名 → 打印参数文件里该参数出现过的所有取值（去重去引号）
  local n="$1" f
  for f in $PFILES; do [ -f "$f" ] && grep -a -h "^ *$n:" "$f"; done 2>/dev/null \
    | sed "s/^ *$n: *//; s/['\"]//g; s/[[:space:]]*$//" | sort -u | tr '\n' ' '
}

FAIL=0
printf "  闸A rgbd_odometry 输出必须落 /rtabmap/rgbd_odom : "
if echo "$OARGV" | grep -qx 'odom:=/rtabmap/rgbd_odom'; then echo "✓ [remap]"; else echo "✗ 实际: $(echo "$OARGV" | grep -a '^odom:=' || echo 无)"; FAIL=1; fi
printf "  闸B rgbd_odometry 绝不许写 /odom              : "
if echo "$OARGV" | grep -qx 'odom:=/odom'; then echo "✗✗ 撞车！"; FAIL=1; else echo "✓ [remap] 无 odom:=/odom"; fi
printf "  闸C rgbd_odometry 不许发 TF                   : "
if echo "$OARGV" | grep -qx 'publish_tf:=False\|publish_tf:=false'; then echo "✓ [argv]"
else
  C=$(param_all publish_tf)
  case "$C" in
    "false "|"False ") echo "✓ [params-file] $C";;
    "")  echo "… 未确认（argv 与 params-file 都没找到）—— 不阻断，由推起后的 /tf 兜底";;
    *)   echo "✗ 实际 = $C"; FAIL=1;;
  esac
fi
EXP_ODOM_IN="${P3A_ODOM_IN_TOPIC:-/odom}"
# ★ 期望值由环境变量决定（2026-09-30 用户点名授权「nosfm 臂补 R1–R5」；不设 = 改前行为 /odom）。
#   仍是硬断言一个确定值，不是放宽。臂别可从本行日志追溯。
printf "  闸D rtabmap 输入必须读 %s（期望，由 P3A_ODOM_IN_TOPIC 决定）           : " "$EXP_ODOM_IN"
if echo "$RARGV" | grep -qx "odom:=$EXP_ODOM_IN"; then echo "✓ [remap]"; else echo "✗ 实际: $(echo "$RARGV" | grep -a '^odom:=' || echo 无)"; FAIL=1; fi
printf "  闸E rtabmap 仍订阅 OdomInfo（SFM 原料）       : "
E=$(param_all subscribe_odom_info)
if echo "$RARGV" | grep -qx 'subscribe_odom_info:=True\|subscribe_odom_info:=true'; then echo "✓ [argv] $E"
elif [ -n "$E" ]; then echo "✓ [params-file] $E"
else echo "… 未确认 —— 由推起后的 OdomInfo.local_map_size 兜底"; fi
if [ "$FAIL" != "0" ]; then
  echo
  echo "  ★★★ 有闸红了 —— 立刻收尾，不许推。"
  bash /ros2_ws/p3a_l3/stop_l3.sh >/dev/null 2>&1
  exit 1
fi
echo "  ⇒ argv 级全绿（决定性）。起跑到现在 $(( $(date +%s) - T0 ))s"
echo

# ── 采样器（rclpy 版）──
if [ "$SAMPLER" = "1" ]; then
  nohup python3 /ros2_ws/p3a_l3/odom_sampler.py "$SAMP" 2 >/dev/null 2>&1 &
  echo "  采样器已起（rclpy，每 2s）→ $SAMP"
fi

echo "════════════════════════════════════════════════════════"
echo "  ★★★ 可以推了（起跑到现在 $(( $(date +%s) - T0 ))s，v1 是 33s）"
echo "════════════════════════════════════════════════════════"
echo "  ① 立刻开始推，不要等后面那些确认（它们在后台跑，不挡你）"
echo "  ② ★★★ 走满一整圈"
echo "  ③ ★★★ 过了起点【不要停】：一边继续往前走，一边发消息告诉我（「过了」即可）。"
echo "     我收到就【立刻】杀 rtabmap。★★★ 在我杀掉之前【千万别停】。"
echo "     杀掉之后随你停 —— rtabmap 已死，站着不动也不会再产节点。"
echo "     原因：站桩会让 SFM 拿不到 3D，且在同一位置堆出大量同姿态节点。"
echo "     实证：v4 收轮时，人机往返延迟造出 node 75~87 共 13 个位姿【完全相同】的废节点，"
echo "     它们提出的候选（76<->8 … 87<->8）全军覆没。"
echo "     ★ 这条延迟现在反而是好事：你走得越远，收尾的运动节点越多。"
echo

# ═══════════ 推起来之后才做的【确认】（不是判据，红了只是警告）═══════════
{
  echo "=== PROTOCOL v2 确认报告 $(date -u +%FT%TZ) ==="
  echo "--- /odom 发布者（应为 1；变 2 = 生产被污染）---"
  timeout 25 ros2 topic info /odom 2>/dev/null
  echo "--- /tf child frame 分布（不许出现 rgbd_odom）---"
  timeout 8 ros2 topic echo /tf 2>/dev/null | grep -a 'child_frame_id' | sort | uniq -c | sort -rn | head -8
  echo "--- 新话题 ---"
  timeout 25 ros2 topic list 2>/dev/null | grep -aE 'rgbd_odom|odom_info'
  echo "--- OdomInfo（local_map_size>0 = SFM 有原料）---"
  OI=$(timeout 25 ros2 topic echo --once /rtabmap/odom_info 2>/dev/null | head -40)
  for F in lost matches inliers features local_map_size; do
    V=$(echo "$OI" | grep -a -m1 "^$F:" | sed "s/^$F: *//")
    printf "    %-18s = %s\n" "$F" "${V:-（拿不到）}"
  done
} >"$GATE" 2>&1
echo "  --- 确认报告（后台跑完，写 $GATE）---"
cat "$GATE" | sed 's/^/    /'
PUB=$(awk '/^--- \/odom 发布者/{f=1;next} /^--- \/tf/{f=0} f' "$GATE" | grep -a -m1 'Publisher count:' | grep -ao '[0-9]*$')
if [ "${PUB:-}" = "1" ]; then
  echo "    ✓ /odom 仍只有 1 个发布者（解耦成功，生产未被污染）"
elif [ -n "${PUB:-}" ]; then
  echo "    ★★★ 警告：/odom 发布者 = $PUB（应=1）！立刻 bash /ros2_ws/p3a_l3/stop_l3.sh"
else
  echo "    … /odom 发布者数没拿到 —— 报告在 $GATE"
fi
if awk '/^--- \/tf child frame/{f=1;next} /^--- 新话题/{f=0} f' "$GATE" | grep -aq 'rgbd_odom'; then
  echo "    ★★★ 警告：/tf 里出现了 rgbd_odom 帧！立刻 bash /ros2_ws/p3a_l3/stop_l3.sh"
fi
echo
echo "=== $(date -u +%FT%TZ) 起跑完成，共 $(( $(date +%s) - T0 ))s。收尾用 stop_l3.sh ==="
