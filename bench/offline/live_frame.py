#!/usr/bin/env python3
"""抓一帧实时相机图并测亮度 —— 判定当前现场照明是否足以建库。

单次订阅，不持久 echo，不碰点云。同时对比 front_up（采集/重定位实际用的那个）
与 front_down（记忆里记为跟随相机）。
"""
from __future__ import annotations

import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

TOPICS = [
    '/camera/front_up/color/image_raw',
    '/camera/front_down/color/image_raw',
]

_SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('live_frame_probe')
        self.got: dict = {}
        self.subs = []
        for t in TOPICS:
            self.subs.append(self.create_subscription(
                Image, t, lambda m, tt=t: self.got.setdefault(tt, m), _SENSOR_QOS))

    def grab(self, topic: str, timeout: float = 12.0):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout and topic not in self.got:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.got.get(topic)


def to_bgr(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    enc = (msg.encoding or '').lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(h, w, 3)
        return img[:, :, ::-1] if enc == 'rgb8' else img
    if enc in ('mono8', '8uc1'):
        return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
    raise SystemExit(f'未预期编码 {msg.encoding!r}')


def main() -> int:
    rclpy.init()
    n = Probe()
    print('  等待两种相机的首帧（各最多 12s）...')
    time.sleep(0.5)
    for t in TOPICS:
        m = n.grab(t)
        if m is None:
            print('  %-40s  ❌ 无消息' % t)
            continue
        img = to_bgr(m)
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        orb = cv2.ORB_create(nfeatures=1000)
        kp = orb.detect(g, None)
        out = '/ros2_ws/bench/offline/live_%s.jpg' % t.replace('/', '_').strip('_')
        cv2.imwrite(out, img)
        print('  %-40s  %dx%d  亮度均值=%.1f  中位=%.1f  ORB特征=%d'
              % (t, m.width, m.height, float(g.mean()), float(np.median(g)), len(kp)))
        print('       存图: %s' % out)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
