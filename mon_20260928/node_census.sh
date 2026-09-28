#!/bin/bash
# 只读：节点/进程全表，确认「建库节点」到底在不在（用户要求关掉的那两个）
set -u
source /ros2_ws/scripts/ros_env.sh
echo "=== $(date -u +%FT%TZ)  节点/进程全表  DOMAIN=${ROS_DOMAIN_ID:-未设} ==="

echo "--- [A] ros2 node list（全量，排序） ---"
timeout 30 ros2 node list 2>/dev/null | sort | sed 's/^/  /'
echo -n "  合计: "; timeout 30 ros2 node list 2>/dev/null | wc -l

echo
echo "--- [B] 所有 xw_ / visual / capture / build / db 相关进程（全量） ---"
ps -eo pid,ppid,lstart,etime,cmd --sort=start_time \
  | grep -aE "xw_|visual|capture|build|_db|keyframe|orchestr" \
  | grep -av grep | sed 's/^/  /'

echo
echo "--- [C] 建库相关的【服务】是否还在（话题也可） ---"
for S in /xw/visual_db/build_status_svc /xw/visual_db/build /xw/visual_db/status; do
  printf "  %-38s " "$S"
  timeout 8 ros2 service list 2>/dev/null | grep -qx "$S" && echo "存在(服务)" || echo "-"
done
echo "  --- 含 visual_db / build 的话题 ---"
timeout 15 ros2 topic list 2>/dev/null | grep -aiE "visual_db|build|capture" | sed 's/^/    /' || echo "    (无)"
echo "  --- 含 visual_db 的服务 ---"
timeout 15 ros2 service list 2>/dev/null | grep -aiE "visual_db|build|capture" | sed 's/^/    /' || echo "    (无)"

echo
echo "--- [D] 阳性对照：确认上面 grep 模式能命中（应该有 xw_safety_gate） ---"
timeout 15 ros2 topic list 2>/dev/null | grep -c "xw/" | sed 's/^/  xw\/ 话题数: /'

echo
echo "=== 完 $(date -u +%FT%TZ) ==="
