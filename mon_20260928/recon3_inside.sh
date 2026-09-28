#!/bin/bash
# 容器内：①重启采样器（RELIABLE 版）②整库备份（红线 9）③候选池分组
set -u
source /ros2_ws/scripts/ros_env.sh

echo "########## 1. 重启采样器（RELIABLE 版） ##########"
OLD=$(pgrep -f "mon_20260928/sampler.py" | head -1)
if [ -n "${OLD:-}" ]; then
  echo "旧采样器 PID=$OLD  启动于: $(ps -o lstart= -p $OLD)"
  kill "$OLD"; sleep 2
  kill -0 "$OLD" 2>/dev/null && echo "STILL_ALIVE" || echo "stopped"
else
  echo "没找到旧采样器"
fi
mv -v /ros2_ws/mon_20260928/events.log /ros2_ws/mon_20260928/events.v1_besteffort.log 2>/dev/null

echo
echo "########## 2. 整库备份（红线 9） ##########"
mkdir -p /ros2_ws/backups
TS=20260928_0134_pre_promote
echo "--- 备份前指针 ---"
readlink -f /ros2_ws/maps/vp/visual/current_active_version
echo "--- 备份前 index.json / manifest.yaml 指纹 ---"
sha256sum /ros2_ws/maps/vp/visual/candidate/index.json /ros2_ws/maps/vp/visual/manifest.yaml
echo "--- tar 中 ... ---"
tar czf "/ros2_ws/backups/vp_visual_${TS}.tar.gz" -C /ros2_ws/maps/vp visual 2>&1 | tail -3
echo "--- 备份物 ---"
ls -la "/ros2_ws/backups/vp_visual_${TS}.tar.gz"
sha256sum "/ros2_ws/backups/vp_visual_${TS}.tar.gz"
echo "--- 归档内条目数（分母，0=失败） ---"
tar tzf "/ros2_ws/backups/vp_visual_${TS}.tar.gz" | wc -l
echo "--- 磁盘 ---"
df -h /ros2_ws | tail -1

echo
echo "########## 3. 候选池分组 ##########"
python3 /ros2_ws/mon_20260928/cand_group.py
