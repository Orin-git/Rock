#!/bin/bash
# 只读：/xw/visual_db/* 的服务与话题到底是谁在托管（决定「建库节点」还有没有残留在跑）
set -u
source /ros2_ws/scripts/ros_env.sh
echo "=== $(date -u +%FT%TZ)  visual_db 服务托管者追查 ==="

echo "--- [A] 逐个候选节点，看它自己提供哪些 visual_db 服务 ---"
for N in /xw_web /xw_map_manager /xw_global_reloc_poc /xw_supervisor /xw_nav_session /xw_slam_session; do
  printf "\n  ## %s\n" "$N"
  timeout 20 ros2 node info "$N" 2>/dev/null \
    | awk '/Service Servers:/,/Action Servers:/' \
    | grep -aiE "visual_db|capture|build|db" | sed 's/^/     /' \
    || echo "     (读不到 node info)"
done

echo
echo "--- [B] 哪个节点在【发布】/xw/visual_db/build_status（建库编排器的脉搏） ---"
timeout 15 ros2 topic info /xw/visual_db/build_status -v 2>/dev/null | head -20 | sed 's/^/  /'
echo "  --- 谁在订阅 /xw/visual_db/build ---"
timeout 15 ros2 topic info /xw/visual_db/build -v 2>/dev/null | head -20 | sed 's/^/  /'

echo
echo "--- [C] 直接读一次状态（这才是判据：会话到底在不在飞） ---"
timeout 20 ros2 service call /xw/visual_db/build_status_svc std_srvs/srv/Trigger "{}" 2>&1 | tail -5 | sed 's/^/  /'

echo
echo "--- [D] 容器内进程按启动时间全表（找手动起的、不在 launch 里的） ---"
ps -eo pid,ppid,lstart,etime,pcpu,cmd --sort=start_time \
  | awk 'NR==1 || $0 !~ /robot.launch.py/' \
  | grep -avE "^\s*[0-9]+\s+77\s+Mon Sep 28 01:03" \
  | head -30 | sed 's/^/  /'

echo
echo "=== 完 $(date -u +%FT%TZ) ==="
