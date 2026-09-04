#!/usr/bin/env python3
"""Per-1-degree-bin statistics of sub-0.35m hits (body-scale), 30 s."""
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

DUR = 30.0
LIM = 0.45  # body-scale reflections only

class Bin(Node):
    def __init__(self):
        super().__init__('scan_bin_probe')
        self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.n = 0
        self.hits = {}

    def cb(self, m):
        self.n += 1
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r > LIM:
                continue
            a = math.degrees(m.angle_min + m.angle_increment * i)
            b = int(round(a))
            t, s = self.hits.get(b, (0, 0.0))
            self.hits[b] = (t + 1, s + r)

def main():
    rclpy.init()
    node = Bin()
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info(f'frames={node.n}')
    keys = sorted(node.hits)
    # group consecutive bins into blocks, but print every bin line
    for k in keys:
        t, s = node.hits[k]
        pct = 100.0 * t / node.n
        node.get_logger().info(f'{k:+6d}deg  {t:4d}/{node.n:4d}  {pct:5.1f}%  avg={s / t:.3f}m')
    rclpy.shutdown()

main()
