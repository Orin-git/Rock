#!/usr/bin/env python3
"""关键帧图像可用性普查 —— ORB 特征数 + 图像亮度。

为什么必须查：R3 现场干跑时查询图几乎全黑（室内未开灯），ORB 只出 2-4 个匹配，
检索必然返回随机帧。若库里的关键帧也同样是黑图，则整个视觉库建立在不可用的
图像上 —— 那样「建到 80%」只是在堆更多不可用的帧，R3 永远不会好。
"""
from __future__ import annotations

import os

import cv2
import numpy as np
import yaml

VROOT = '/ros2_ws/maps/vp/visual/versions/vp_visual_v1.4'
KDIR = f'{VROOT}/keyframes'


def main() -> None:
    ids = sorted(k for k in os.listdir(KDIR) if k.startswith('kf_'))
    rows = []
    for kid in ids:
        d = f'{KDIR}/{kid}'
        try:
            img = cv2.imread(f'{d}/rgb.jpg', cv2.IMREAD_GRAYSCALE)
            n_kp = int(np.load(f'{d}/keypoints.npy').shape[0])
            desc = np.load(f'{d}/descriptors.npy')
            meta = yaml.safe_load(open(f'{d}/meta.yaml'))
        except Exception as exc:  # noqa: BLE001
            rows.append((kid, -1, -1, -1.0, 'ERR:' + str(exc)[:30]))
            continue
        mean = float(img.mean()) if img is not None else -1.0
        fmt = 'NEW' if meta.get('scan_stamp') is not None else 'OLD'
        rows.append((kid, n_kp, int(desc.shape[0]), mean, fmt))

    print('  %-12s %-5s %8s %8s %9s' % ('id', 'fmt', 'kp数', 'desc数', '亮度均值'))
    dark = 0
    for kid, n_kp, nd, mean, fmt in rows:
        flag = ''
        if mean >= 0 and mean < 30:
            flag = '  ← 暗'
            dark += 1
        print('  %-12s %-5s %8d %8d %9.1f%s' % (kid, fmt, n_kp, nd, mean, flag))

    good = [r for r in rows if r[1] > 0]
    print()
    print('  总帧数 = %d' % len(rows))
    if good:
        kps = np.array([r[1] for r in good])
        means = np.array([r[3] for r in good])
        print('  kp 数：  min=%d  P25=%.0f  中位=%.0f  max=%d'
              % (kps.min(), np.percentile(kps, 25), np.median(kps), kps.max()))
        print('  亮度：   min=%.1f  P25=%.1f  中位=%.1f  max=%.1f'
              % (means.min(), np.percentile(means, 25), np.median(means), means.max()))
        print('  暗帧(<30) = %d/%d' % (dark, len(rows)))
        print('  kp < 100 的帧 = %d/%d' % (int((kps < 100).sum()), len(good)))


if __name__ == '__main__':
    main()
