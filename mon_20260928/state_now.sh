#!/bin/bash
# 只读：B 族停摆后的即时状态（定位是否还在 DEGRADED）
set -u
# ★★★ 必须 source：docker exec bash 不读 .bashrc，漏了会跑在 domain 0（全栈在 99）⇒ 话题全空、node list=0
source /ros2_ws/scripts/ros_env.sh
echo "=== $(date -u +%FT%TZ)  即时状态  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-未设} ==="
echo "--- [1] 三闸门（RELIABLE，逐条 --once） ---"
for T in /xw/localization_status /xw/nav/goals_blocked /xw/localization/phase2c_loc_state; do
  printf "  %-42s " "$T"
  timeout 12 ros2 topic echo "$T" --once 2>/dev/null | head -1
done
echo "--- [1b] 60s 复读 6 次（按纪律：不认单次读数） ---"
for i in 1 2 3 4 5 6; do
  A=$(timeout 12 ros2 topic echo /xw/localization_status --once 2>/dev/null | head -1 | tr -d ' ')
  B=$(timeout 12 ros2 topic echo /xw/localization/phase2c_loc_state --once 2>/dev/null | head -1 | tr -d ' ')
  C=$(timeout 12 ros2 topic echo /xw/nav/goals_blocked --once 2>/dev/null | head -1 | tr -d ' ')
  echo "  #$i  loc=$A  p2c=$B  blocked=$C   @$(date -u +%T)"
  sleep 8
done
echo
echo "--- [2] 定位相关节点存活 ---"
ps -eo pid,etime,pcpu,cmd | grep -aE "lost_recovery|localization_health|boot_localizer|amcl" | grep -av grep | sed 's/^/  /'
echo
echo "--- [3] 最近 20 分钟 journal 里的停摆/BMS 行（宿主） ---"
echo "  (容器内 journalctl 是空的，此项须在宿主做 —— 本脚本跑在容器里，跳过)"
echo
echo "--- [4] 节点数 + uptime ---"
echo "  uptime: $(uptime)"
echo -n "  node list: "; timeout 25 ros2 node list 2>/dev/null | wc -l
echo
echo "=== 完 $(date -u +%FT%TZ) ==="
