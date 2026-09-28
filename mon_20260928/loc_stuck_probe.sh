#!/bin/bash
# 只读：为什么停摆后 DEGRADED 不自动恢复？（决定「停摆的持久后果」能不能修）
set -u
source /ros2_ws/scripts/ros_env.sh
echo "=== $(date -u +%FT%TZ)  定位卡死探针（只读） ==="

echo "--- [1] 全部 localization / health 相关话题 ---"
timeout 20 ros2 topic list 2>/dev/null | grep -aiE "localiz|health|loc_|phase2c|reloc" | sort | sed 's/^/  /'

echo
echo "--- [2] 全部 localization 相关服务 ---"
timeout 20 ros2 service list 2>/dev/null | grep -aiE "localiz|health|phase2c|reloc" | sort | sed 's/^/  /'

echo
echo "--- [3] 关键状态量逐个 --once ---"
for T in /xw/localization_status \
         /xw/localization/phase2c_loc_state \
         /xw/health/localization \
         /xw/localization/health_detail \
         /xw/localization/health \
         /xw/health/loc_status \
         /xw/localization/abstain_reason ; do
  printf "  %-42s " "$T"
  timeout 8 ros2 topic echo "$T" --once 2>/dev/null | head -6 | tr '\n' ' ' | cut -c1-160
  echo
done

echo
echo "--- [4] 传感器新鲜度（停摆后是否真的都恢复了） ---"
for T in /odom /scan /battery_state /imu/data; do
  printf "  %-30s " "$T"
  timeout 8 ros2 topic echo "$T" --once --field header 2>/dev/null | tr '\n' ' ' | cut -c1-120
  echo
done

echo
echo "--- [5] localizer 家族的发布者普查（谁在发 loc_status） ---"
timeout 15 ros2 topic info /xw/localization_status -v 2>/dev/null | head -24 | sed 's/^/  /'

echo
echo "--- [6] lost_recovery 节点的服务/话题（重武装通路在这） ---"
timeout 20 ros2 node info /xw_lost_recovery 2>/dev/null | sed 's/^/  /' | head -40

echo
echo "=== 完 $(date -u +%FT%TZ) ==="
