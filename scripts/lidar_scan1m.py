#!/usr/bin/env python3
"""Collect /scan for DURATIONs; report angle bins with hits < 1.0 m."""
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

DUR = 30.0
RANGE_LIM = 1.0

class ScanCollector(Node):
    def __init__(self):
        super().__init__('scan_1m_probe')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.n = 0
        self.bins = {}   # deg_bin -> (times_seen, sum_dist)
        self.range_limits = []

    def cb(self, msg):
        self.n += 1
        self.range_limits.append((len(msg.ranges), msg.angle_min, msg.angle_increment))
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r > RANGE_LIM:
                continue
            a = math.degrees(msg.angle_min + msg.angle_increment * i)
            b = int(round(a))
            t, s = self.bins.get(b, (0, 0.0))
            self.bins[b] = (t + 1, s + r)
        if self.n % 50 == 0:
            self.get_logger().info(f'frames={self.n}')

def main():
    rclpy.init()
    node = ScanCollector()
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info(f'TOTAL frames={node.n} len={node.range_limits[0][0] if node.range_limits else 0} angle0=%.2f dinc=%.4f' % (
        node.range_limits[0][1] if node.range_limits else 0,
        node.range_limits[0][2] if node.range_limits else 0))
    if not node.bins:
        node.get_logger().info('NO points < 1.0m detected — robot is clean')
        return
    # cluster consecutive deg bins
    keys = sorted(node.bins)
    out = []
    cur = [keys[0]]
    for k in keys[1:]:
        if k == cur[-1] + 1:
            cur.append(k)
        else:
            out.append(cur); cur = [k]
    out.append(cur)
    node.get_logger().info('Clusters of angles with <1m hits (degree | seen/total | avg dist m):')
    for cl in out:
        hi_t = max(node.bins[k][0] for k in cl)
        tot_s = sum(node.bins[k][1] for k in cl)
        avg = tot_s / sum(node.bins[k][0] for k in cl)
        node.get_logger().info(f'  {cl[0]:6.0f} .. {cl[-1]:6.0f} deg  peak={hi_t}/{node.n}  avg={avg:.3f}')
    rclpy.shutdown()

main()
