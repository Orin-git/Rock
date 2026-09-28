#!/usr/bin/env python3
"""189 监控采样器（2026-09-28 09:20-16:30 CST）。

只订阅，不发布、不请求、不改变任何系统状态。
刻意【不】订阅任何图像/点云话题（红线 4 + 避免惊动闸门逻辑）。

产出：
  /ros2_ws/mon_20260928/gates.csv    每 5 s 一行（CSV 本身有空洞 = 采样器死了）
  /ros2_ws/mon_20260928/events.log   只在状态变化 / 出现间隙时追加
  /ros2_ws/mon_20260928/sampler.hb   心跳（每秒 touch，用于判活）
"""
import os
import time
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Int8, Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState

OUT = '/ros2_ws/mon_20260928'
CSV = os.path.join(OUT, 'gates.csv')
EV = os.path.join(OUT, 'events.log')
HB = os.path.join(OUT, 'sampler.hb')

GAP_THRESHOLD = 0.5      # s，超过就记一次间隙事件
TICK = 5.0               # s，CSV 采样周期
SWEEP = 0.2              # s，spin 步长


def utc():
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'


def qos(durability):
    # BEST_EFFORT 订阅对 RELIABLE / BEST_EFFORT 发布者都兼容（反向不兼容）
    return QoSProfile(depth=20,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=durability)


class Mon(Node):
    def __init__(self):
        super().__init__('mon_20260928')
        self.gate = {'loc_status': None, 'goals_blocked': None, 'phase2c': None}
        self.last_recv = {}          # src -> monotonic
        self.count = {}              # src -> int
        self.batt = (None, None, None)
        self.in_gap = {}             # src -> bool

        latched = qos(DurabilityPolicy.TRANSIENT_LOCAL)
        vol = qos(DurabilityPolicy.VOLATILE)

        self.create_subscription(Int8, '/xw/localization_status',
                                 lambda m: self.on_gate('loc_status', m.data), latched)
        self.create_subscription(Bool, '/xw/nav/goals_blocked',
                                 lambda m: self.on_gate('goals_blocked', m.data), latched)
        self.create_subscription(String, '/xw/localization/phase2c_loc_state',
                                 lambda m: self.on_gate('phase2c', m.data), latched)

        self.create_subscription(Odometry, '/odom', lambda m: self.on_stream('odom'), vol)
        self.create_subscription(LaserScan, '/scan', lambda m: self.on_stream('scan'), vol)
        self.create_subscription(BatteryState, '/battery_state',
                                 lambda m: self.on_batt(m), vol)

        for src in ('odom', 'scan', 'battery'):
            self.last_recv[src] = None
            self.count[src] = 0
            self.in_gap[src] = False

        self.log('SAMPLER_START', f'gap_threshold={GAP_THRESHOLD}s tick={TICK}s')

    def log(self, kind, text):
        line = f'{utc()} {kind} {text}'
        with open(EV, 'a') as f:
            f.write(line + '\n')
            f.flush()
        print(line, flush=True)

    def on_gate(self, name, val):
        old = self.gate[name]
        if old == val:
            return
        self.gate[name] = val
        self.log('GATE', f'{name}: {old!r} -> {val!r}')

    def on_stream(self, src):
        now = time.monotonic()
        prev = self.last_recv[src]
        self.last_recv[src] = now
        self.count[src] += 1
        if prev is not None:
            gap = now - prev
            if gap > GAP_THRESHOLD:
                self.log('GAP_START', f'{src} gap={gap:.3f}s')
                self.in_gap[src] = True
        elif self.in_gap[src]:
            # 这个分支不会走到（prev None 只出现在第一条），留着表明意图
            pass

    def on_batt(self, m):
        self.batt = (int(m.power_supply_status), float(m.percentage), float(m.voltage))
        self.on_stream('battery')

    def tick(self):
        now = time.monotonic()
        ages = {}
        for src in ('odom', 'scan', 'battery'):
            lr = self.last_recv[src]
            ages[src] = (now - lr) if lr is not None else -1.0
            if ages[src] > GAP_THRESHOLD and not self.in_gap[src]:
                self.log('GAP_START', f'{src} gap>{GAP_THRESHOLD}s (age={ages[src]:.3f}s)')
                self.in_gap[src] = True
            elif ages[src] <= GAP_THRESHOLD and self.in_gap[src]:
                self.log('GAP_END', f'{src} age={ages[src]:.3f}s')
                self.in_gap[src] = False

        row = ','.join([
            utc(), f'{time.time():.3f}',
            str(self.gate['loc_status']), str(self.gate['goals_blocked']),
            str(self.gate['phase2c']),
            str(self.batt[0]), str(self.batt[1]), str(self.batt[2]),
            f'{ages["odom"]:.3f}', f'{ages["scan"]:.3f}', f'{ages["battery"]:.3f}',
            str(self.count['odom']), str(self.count['scan']), str(self.count['battery']),
        ])
        with open(CSV, 'a') as f:
            f.write(row + '\n')
            f.flush()


def main():
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(CSV):
        with open(CSV, 'w') as f:
            f.write('utc,unix,loc_status,goals_blocked,phase2c,'
                    'batt_status,batt_pct,batt_v,'
                    'age_odom,age_scan,age_battery,n_odom,n_scan,n_battery\n')

    rclpy.init()
    node = Mon()
    next_tick = time.monotonic()
    next_hb = time.monotonic()
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=SWEEP)
            now = time.monotonic()
            if now >= next_tick:
                node.tick()
                next_tick = now + TICK
            if now >= next_hb:
                with open(HB, 'w') as f:
                    f.write(f'{utc()} {time.time():.3f}\n')
                next_hb = now + 1.0
    except KeyboardInterrupt:
        pass
    finally:
        node.log('SAMPLER_STOP', 'clean exit')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
