#!/usr/bin/env python3
"""决定性测试：R3 拿到的黑图，是【相机刚开启、曝光未收敛】还是【现场真的暗】。

做法：复刻 R3 武装时的状态变化（置 phase2c_recovery = true），然后从相机开始
出图的那一刻起记录每一帧的亮度。R3 正是在这个时刻取「武装后首帧」。

判据：
    亮度从很暗爬升到 ~100  -> 曝光收敛问题。修法是等稳定或丢弃前 N 帧。
    全程都暗（max 也很低）  -> 现场照明问题。修法是别在这时候建库。

安全性（逐条）：
  * phase2c_recovery=true 的两个消费者都是【抑制性】的：
      perception_mode_manager -> 只开 rgb_up（不动任何执行器）
      localization_health_node -> 禁止 spin + reinitialize_global_localization
    即它只会让机器人更不动，不会让它动。
  * 全程 try/finally，结束时一定发回 false（或恢复原值）。
  * 只读图像，不发任何运动/导航指令。
"""
from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

TOPIC = '/camera/front_up/color/image_raw'
REC_FLAG = '/xw/localization/phase2c_recovery'
WINDOW_SEC = 30.0

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
_LATCH = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('exposure_probe')
        self.rows: list = []
        self.flag = None
        self.t0 = time.monotonic()
        self.create_subscription(Image, TOPIC, self._on_img, _SENSOR_QOS)
        self.create_subscription(Bool, REC_FLAG, self._on_flag, _LATCH)
        self.pub = self.create_publisher(Bool, REC_FLAG, _LATCH)

    def _on_flag(self, m: Bool) -> None:
        if self.flag is None:
            self.flag = bool(m.data)

    def _on_img(self, m: Image) -> None:
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
        kp = cv2.ORB_create(nfeatures=1000).detect(g, None)
        self.rows.append((t, float(g.mean()), float(np.median(g)), int(g.max()), len(kp)))


def main() -> int:
    rclpy.init()
    n = Probe()

    print('  1) 读当前 %s 的值 ...' % REC_FLAG)
    t = time.monotonic()
    while rclpy.ok() and n.flag is None and time.monotonic() - t < 8.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    orig = n.flag
    print('     当前值 = %r' % orig)

    print('  2) 开相机前先确认它确实是关的（记录 5s）...')
    t = time.monotonic()
    while rclpy.ok() and time.monotonic() - t < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    print('     开相机前收到 %d 帧' % len(n.rows))

    ok = False
    try:
        print('  3) 置 %s = true（等价于 R3 武装）...' % REC_FLAG)
        n.pub.publish(Bool(data=True))
        n.rows.clear()
        n.t0 = time.monotonic()
        print('  4) 记录 %.0f 秒内每一帧 ...' % WINDOW_SEC)
        while rclpy.ok() and (time.monotonic() - n.t0) < WINDOW_SEC:
            rclpy.spin_once(n, timeout_sec=0.1)
        ok = True
    finally:
        restore = bool(orig) if orig is not None else False
        print('  5) 复位 %s = %r ...' % (REC_FLAG, restore))
        n.pub.publish(Bool(data=restore))
        t = time.monotonic()
        while rclpy.ok() and time.monotonic() - t < 2.0:
            rclpy.spin_once(n, timeout_sec=0.1)

    rows = n.rows
    print()
    print('  收到 %d 帧' % len(rows))
    if not rows:
        print('  ❌ 置 true 后仍无一帧 —— 相机不是由这个开关驱动的')
        n.destroy_node(); rclpy.shutdown(); return 1 if ok else 2
    print()
    print('  %8s %10s %10s %6s %7s' % ('t(s)', '亮度均值', '亮度中位', 'max', 'ORB数'))
    for t, mean, med, mx, kp in rows[:30]:
        print('  %8.2f %10.2f %10.2f %6d %7d' % (t, mean, med, mx, kp))
    if len(rows) > 30:
        print('  ...（共 %d 帧，其余略）' % len(rows))
    means = np.array([r[1] for r in rows])
    print()
    print('  首帧亮度=%.2f  第5帧=%.2f  末帧=%.2f  全程max=%.2f'
          % (means[0], means[min(4, len(means) - 1)], means[-1], means.max()))
    print()
    if means[0] < 30 and means.max() > 60:
        print('  ✅ 曝光收敛问题：首帧暗(%.1f) → 稳定在 %.1f' % (means[0], means.max()))
        print('     => R3 取武装后首帧，拿到的必然是不可用的图。修法：丢弃前 N 帧 / 等曝光稳定。')
    elif means[0] > 60:
        print('  ❌ 不是曝光问题：首帧就是亮的(%.1f)' % means[0])
        print('     => 那些黑图是现场照明所致。')
    else:
        print('  ⚠ 全程都暗(max=%.1f) —— 现场照明问题，不是曝光收敛。' % means.max())

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
