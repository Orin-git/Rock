#!/usr/bin/env python3
"""连续帧亮度时间序列 —— 判定 ascamera 是否存在「首帧偏暗」的曝光爬升。

假设 H：R3 取武装后首帧，而该帧因自动曝光未收敛而偏暗 -> 视觉检索必然失败。
    H 成立 -> 序列应从很暗爬升到 ~100
    H 不成立 -> 序列一开始就是 ~100（相机本来就在常发且已稳定）

QoS 与 reloc 节点完全一致（BEST_EFFORT / VOLATILE / KEEP_LAST / depth 5），
否则测的不是同一条流。
"""
from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

TOPIC = '/camera/front_up/color/image_raw'
WINDOW_SEC = 30.0

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('live_series_probe')
        self.rows: list = []
        self.t0 = time.monotonic()
        self.create_subscription(Image, TOPIC, self._on, _SENSOR_QOS)

    def _on(self, m: Image) -> None:
        t = time.monotonic() - self.t0
        h, w = m.height, m.width
        buf = np.frombuffer(bytes(m.data), dtype=np.uint8)
        enc = (m.encoding or '').lower()
        try:
            if enc in ('rgb8', 'bgr8'):
                img = buf.reshape(h, w, 3)
                g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if enc == 'rgb8'
                                 else cv2.COLOR_BGR2GRAY)
            else:
                g = buf.reshape(h, w)
        except Exception:  # noqa: BLE001
            return
        self.rows.append((t, float(g.mean()), float(np.median(g)), int(g.max()),
                          str(m.encoding), w, h))


def main() -> int:
    rclpy.init()
    n = Probe()
    print('  订阅 %s ，记录 %.0f 秒 ...' % (TOPIC, WINDOW_SEC))
    while rclpy.ok() and (time.monotonic() - n.t0) < WINDOW_SEC:
        rclpy.spin_once(n, timeout_sec=0.1)

    rows = n.rows
    print('  收到 %d 帧' % len(rows))
    if not rows:
        print('  ❌ 整段时间一帧都没到 —— 相机在该话题上根本没有发布者')
        print('     （这与 reloc 节点能在 3.4s 内拿到首帧矛盾，需另查）')
        n.destroy_node(); rclpy.shutdown(); return 1
    print()
    print('  %8s %10s %10s %6s  %-8s %s' % ('t(s)', '亮度均值', '亮度中位', 'max', '编码', '尺寸'))
    for t, mean, med, mx, enc, w, h in rows[:25]:
        print('  %8.3f %10.2f %10.2f %6d  %-8s %dx%d' % (t, mean, med, mx, enc, w, h))
    if len(rows) > 25:
        print('  ...（共 %d 帧，其余略）' % len(rows))
    means = np.array([r[1] for r in rows])
    print()
    print('  首帧亮度=%.2f   末帧亮度=%.2f   全程 max=%.2f' % (means[0], means[-1], means.max()))
    if len(rows) >= 2:
        dt = rows[-1][0] - rows[0][0]
        print('  帧率 ≈ %.1f Hz（%.2fs 内 %d 帧）' % ((len(rows) - 1) / max(dt, 1e-6), dt, len(rows)))
    print()
    if means[0] < 30 and means.max() > 60:
        print('  ✅ H 成立：首帧暗(%.1f)，随后爬到 %.1f —— 存在曝光爬升。' % (means[0], means.max()))
    elif means[0] > 60:
        print('  ❌ H 不成立：首帧就是亮的(%.1f) —— 相机本来就在常发且已稳定。' % means[0])
    else:
        print('  ⚠ 全程都暗(max=%.1f) —— 是现场照明问题，不是曝光收敛。' % means.max())
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
