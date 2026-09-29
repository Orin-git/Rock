#!/usr/bin/env python3
"""P3A L3 里程计采样器（rclpy 版，替代坏掉的 ros2 topic echo shell 版）。

为什么换成 rclpy：
  2026-09-28 的 shell 版采样器 11 条全是空的 —— 两个原因叠加
  (a) grep 缩进模式写错（echo 输出里 position: 是 4 空格缩进、x/y/z 是 6 空格，我写成了 2/4）
  (b) 更致命：`ros2 topic echo --once` 在 daemon 冷启动时 >12 s 才返回，被 timeout 杀掉 ⇒ 输出为空
  ⇒ 用 rclpy 直接订阅，绕开 ROS CLI 与 daemon 的全部开销。

用法: python3 odom_sampler.py <输出文件> [采样周期秒, 默认2]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class Sampler(Node):
    def __init__(self, out_path, period):
        super().__init__('p3a_odom_sampler')
        self.out = open(out_path, 'a', buffering=1)
        self.period = period
        self.last = {}          # topic -> (x, y, yaw)
        self.last_t = {}        # topic -> wall time
        # ★ QoS 兼容性是【单向】的，这决定了这里必须选 BEST_EFFORT：
        #     Offered RELIABLE  + Requested BEST_EFFORT ⇒ 兼容
        #     Offered BEST_EFFORT + Requested RELIABLE  ⇒ 【不兼容，收不到任何数据】
        #   ⇒ BEST_EFFORT 订阅者两边都能接，是严格更宽的被动采样选择。
        #   实测依据：`/odom` = ekf_filter_node 发 RELIABLE/VOLATILE/depth10（本订阅者兼容）；
        #   而 `/rtabmap/rgbd_odom` 由 rgbd_odometry 发，其发布 QoS 未核实，
        #   用 RELIABLE 去订它会有【全程静默收不到】的风险（2026-09-28 shell 采样器
        #   全空的教训已经吃过一次，不能再吃第二次）。
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, '/odom', self.cb_odom, qos)
        self.create_subscription(Odometry, '/rtabmap/rgbd_odom', self.cb_vo, qos)
        self.create_timer(period, self.tick)
        self.out.write(f'# 采样开始 {time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}  '
                       f'周期={period}s  字段: 时刻 topic x y yaw 距上次位移(m)\n')

    def _store(self, tag, msg):
        p = msg.pose.pose.position
        y = yaw_of(msg.pose.pose.orientation)
        wall = time.time()
        prev = self.last.get(tag)
        d = math.dist((p.x, p.y), prev[:2]) if prev else float('nan')
        self.last[tag] = (p.x, p.y, y)
        self.last_t[tag] = wall
        return p, y, d, wall

    def cb_odom(self, msg):
        self._store('ekf', msg)

    def cb_vo(self, msg):
        self._store('rgbd', msg)

    def tick(self):
        now = time.time()
        for tag, name in (('ekf', '/odom'), ('rgbd', '/rtabmap/rgbd_odom')):
            v = self.last.get(tag)
            age = now - self.last_t.get(tag, 0.0)
            if v is None:
                self.out.write(f'{time.strftime("%FT%TZ", time.gmtime(now))} {name} '
                               f'-- 无数据 --\n')
                continue
            # 距上次采样的位移：用本次与上一次打印的差
            prev_print = getattr(self, '_printed_' + tag, None)
            d = math.dist(v[:2], prev_print[:2]) if prev_print else float('nan')
            setattr(self, '_printed_' + tag, v)
            ds = '--' if d != d else f'{d:.3f}'      # 首个采样点无前值（nan）⇒ 打 '--'
            self.out.write(f'{time.strftime("%FT%TZ", time.gmtime(now))} {name} '
                           f'x={v[0]:+8.3f} y={v[1]:+8.3f} yaw={v[2]:+8.2f}° '
                           f'距上次={ds}m 静默={age:.1f}s\n')


def main():
    if len(sys.argv) < 2:
        print('用法: python3 odom_sampler.py <输出文件> [周期秒]')
        return 2
    out_path = sys.argv[1]
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    rclpy.init()
    n = Sampler(out_path, period)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.out.write(f'# 采样结束 {time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}\n')
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
