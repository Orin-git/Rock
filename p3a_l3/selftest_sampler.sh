#!/usr/bin/env bash
# 采样器自检：拿【正在跑的】生产 /odom 当靶子，验证 odom_sampler.py 真的收得到数据。
# 为什么要有这一步：2026-09-28 的 shell 版采样器 11 条全空，事后才知道是
# `ros2 topic echo --once` 在 daemon 冷启动时超时 —— 同类「静默收不到」今天
# 又会以 QoS 不兼容的形式重演（RELIABLE 订 BEST_EFFORT 是接不上的）。
# ⇒ 采样器必须在起跑【之前】被证明能采到数，不能等到复盘时才发现文件是空的。
set -u
source /ros2_ws/scripts/ros_env.sh
OUT=/tmp/p3a_sampler_selftest.txt
rm -f "$OUT"
echo "=== 采样器自检 $(date -u +%FT%TZ) ==="
echo "  靶子: 生产 /odom（ekf_filter_node 发 RELIABLE/VOLATILE/depth10）"
echo "  跑 9 秒，周期 2s …"
timeout 9 python3 /ros2_ws/p3a_l3/odom_sampler.py "$OUT" 2 >/dev/null 2>&1
N=$(grep -ac '^20' "$OUT" 2>/dev/null || echo 0)
echo "--- 输出（$OUT）---"
cut -c1-160 "$OUT" 2>/dev/null | sed 's/^/  /'
echo "--- 有效数据行 = $N ---"
if grep -aq '/odom x=' "$OUT" 2>/dev/null; then
  echo "  ✓✓ 能采到 /odom 的数 —— 采样器可用"
else
  echo "  ✗✗ 采不到 /odom！检查 QoS 兼容性 / 域号，别带着坏采样器起跑"
fi
if grep -aq '/rtabmap/rgbd_odom x=' "$OUT" 2>/dev/null; then
  echo "  ✓ 还能采到 /rtabmap/rgbd_odom"
else
  echo "  … /rtabmap/rgbd_odom 无数据（现在没起 rtabmap，属正常）"
fi
