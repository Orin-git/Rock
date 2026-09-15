#!/usr/bin/env python3
"""历史 dump 的查询图亮度普查 —— 判定 R3 是否【一直】在被喂不可用的图。

139 个 fail_* 目录里各存着一张 query_rgb.jpg，那就是那次 R3 实际拿去做视觉检索的图。
按时间排序看亮度：
    旧亮新黑 -> 最近环境/相机变了（可回溯到某个时刻）
    全程都黑 -> 系统性缺陷，R3 从来没有拿到过可用的查询图
两者都指向「不是覆盖率问题」。完全只读。
"""
from __future__ import annotations

import os

import cv2
import numpy as np

DUMP = '/ros2_ws/bench/phase2a_poc_v1_2026-09-07/reloc_dumps'


def main() -> None:
    dirs = sorted(d for d in os.listdir(DUMP) if d.startswith('fail_'))
    rows = []
    for d in dirs:
        p = f'{DUMP}/{d}/query_rgb.jpg'
        if not os.path.isfile(p):
            continue
        try:
            ts = float(d.split('_')[1])
        except Exception:  # noqa: BLE001
            continue
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        orb = cv2.ORB_create(nfeatures=1000)
        kp = orb.detect(img, None)
        rows.append((ts, d, float(img.mean()), float(np.median(img)),
                     int(img.max()), len(kp)))

    rows.sort()
    print('  dump 总数 = %d（有 query_rgb.jpg 的）' % len(rows))
    if not rows:
        return
    print()
    print('  %-18s %19s %9s %9s %6s %7s' %
          ('dump 目录', 'UTC 时间', '亮度均值', '中位', 'max', 'ORB数'))
    import datetime as dt
    for ts, d, mean, med, mx, nkp in rows:
        t = dt.datetime.utcfromtimestamp(ts).strftime('%m-%d %H:%M:%S')
        flag = ''
        if mean < 30:
            flag = '  ← 暗'
        print('  %-18s %19s %9.1f %9.1f %6d %7d%s' % (d, t, mean, med, mx, nkp, flag))

    means = np.array([r[2] for r in rows])
    kps = np.array([r[5] for r in rows])
    print()
    print('  亮度：min=%.1f  P25=%.1f  中位=%.1f  P75=%.1f  max=%.1f'
          % (means.min(), np.percentile(means, 25), np.median(means),
             np.percentile(means, 75), means.max()))
    print('  ORB ：min=%d  中位=%d  max=%d' % (kps.min(), np.median(kps), kps.max()))
    print('  暗帧(<30) = %d/%d = %.1f%%'
          % (int((means < 30).sum()), len(means), 100.0 * (means < 30).sum() / len(means)))
    print()
    print('  按时间前后各半对比：')
    h = len(rows) // 2
    print('    前半(%d 个) 亮度中位=%.1f  ORB中位=%d'
          % (h, np.median(means[:h]), int(np.median(kps[:h]))))
    print('    后半(%d 个) 亮度中位=%.1f  ORB中位=%d'
          % (len(rows) - h, np.median(means[h:]), int(np.median(kps[h:]))))


if __name__ == '__main__':
    main()
