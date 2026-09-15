#!/usr/bin/env python3
"""当前位姿到最近关键帧的距离 —— 判定 R3 视觉检索「有没有东西可匹配」。

若最近关键帧在数米之外，则 R3 检索返回 2-4 个 ORB 匹配的随机帧是【库稀疏】的
必然结果，不是代码 bug。这条判据决定优先级：建库 vs 修 R3。
"""
from __future__ import annotations

import math
import sys

sys.path.insert(0, '/ros2_ws/bench/offline')

import db_replay as R  # noqa: E402


def yaw_diff(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def main() -> int:
    if len(sys.argv) < 4:
        print('用法: nearest_kf.py X Y YAW_DEG')
        return 2
    qx, qy, qyaw = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
    fr = R.frames()
    rows = []
    for f in fr:
        d = math.hypot(f['x'] - qx, f['y'] - qy)
        rows.append((d, yaw_diff(f['yaw'], qyaw), f['id'], f['fmt']))
    rows.sort()

    print('  查询位姿 = (%.3f, %.3f, %.1f°)   DB 帧数 = %d' % (qx, qy, math.degrees(qyaw), len(fr)))
    print()
    print('  最近的 10 个关键帧：')
    print('    %-12s %-5s %8s %8s' % ('id', 'fmt', '距离m', 'Δyaw°'))
    for d, dy, kid, fmt in rows[:10]:
        print('    %-12s %-5s %8.3f %8.1f' % (kid, fmt, d, math.degrees(dy)))

    print()
    for rad in (0.5, 1.0, 2.0, 3.0, 5.0):
        n = sum(1 for d, _, _, _ in rows if d <= rad)
        print('  半径 %.1f m 内的关键帧数 = %d' % (rad, n))
    # 同时要求 yaw 接近（否则视角不同，ORB 匹配本来就难）
    print()
    for rad, yd in ((1.0, 30.0), (2.0, 30.0), (2.0, 45.0), (3.0, 45.0)):
        n = sum(1 for d, dy, _, _ in rows
                if d <= rad and math.degrees(dy) <= yd)
        print('  半径 %.1f m 且 Δyaw ≤ %.0f° 的关键帧数 = %d' % (rad, yd, n))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
