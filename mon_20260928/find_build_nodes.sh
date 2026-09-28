#!/bin/bash
# 第一步：只读 —— 找出「建库节点」到底是哪几个 PID（用户要求「建库节点关掉」）
# 纪律：容器 PID 命名空间与宿主不同 ⇒ 必须在容器内 ps；禁 pkill -f ⇒ 这里只列不杀
set -u
# ★★★ 同 state_now.sh：必须 source，否则 ros2 跑在 domain 0
source /ros2_ws/scripts/ros_env.sh
echo "=== $(date -u +%FT%TZ)  建库节点普查（只读）  DOMAIN=${ROS_DOMAIN_ID:-未设} ==="
echo "--- [1] 候选进程全表（含启动时间/父子/CPU/内存） ---"
ps -eo pid,ppid,lstart,etime,pcpu,pmem,stat,cmd --sort=start_time \
  | grep -aE "build_orchestrator|capture_candidate|visual_db_capture|visual_db_build|keyframe_db_builder" \
  | grep -av grep | sed 's/^/  /'

echo
echo "--- [2] 阳性对照：确认 grep 模式本身能命中（拿别的节点试） ---"
ps -eo pid,cmd | grep -aE "xw_safety_gate|xw_supervisor" | grep -av grep | head -3 | sed 's/^/  /'

echo
echo "--- [3] 这两个节点是谁起的（父进程链） ---"
for P in $(ps -eo pid,cmd | grep -aE "build_orchestrator|capture_candidate|visual_db_capture" | grep -av grep | awk '{print $1}'); do
  echo "  PID $P 父链:"
  Q=$P
  for i in 1 2 3 4; do
    read -r PP CMD < <(ps -o ppid=,cmd= -p "$Q" 2>/dev/null | head -1)
    [ -z "${PP:-}" ] && break
    echo "    $Q <- $PP  $(echo "$CMD" | cut -c1-90)"
    Q=$PP; [ "$Q" = "1" ] && break
  done
done

echo
echo "--- [4] 会话状态（杀之前必须确认没会话在飞） ---"
timeout 12 ros2 service call /xw/visual_db/build_status_svc std_srvs/srv/Trigger "{}" 2>&1 | tail -3

echo
echo "--- [5] 抓包/录制类进程（录包时别误杀） ---"
ps -eo pid,cmd | grep -aE "ros2 bag|record" | grep -av grep | sed 's/^/  /' || echo "  (无)"

echo
echo "--- [6] 当前节点总数（基线 64） ---"
timeout 20 ros2 node list 2>/dev/null | wc -l

echo
echo "=== 完 $(date -u +%FT%TZ) ==="
