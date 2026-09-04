#!/usr/bin/env python3
"""/ultrasonic_array (cm) -> /ultrasonic_scan (LaserScan, base_link).

Front two probes only for now (rear are parked). Angular positions from
mount: +-10 cm lateral, facing forward -> ~ +-6 deg.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from xw_interfaces.msg import UltrasonicArray


class UltrasonicToLaserScan(Node):
    def __init__(self):
        super().__init__('xw_ultrasonic_to_laserscan')
        self.declare_parameter('output_topic', '/ultrasonic_scan')
        self.declare_parameter('probe_angles_deg', [6.0, -6.0, 52.0, -52.0])
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 1.2)
        self.declare_parameter('filter_window', 3)   # median window (ghost killer)
        self.declare_parameter('min_valid_streak', 2)  # consecutive valid frames
        self.declare_parameter('blind_zone_m', 0.25)
        self.declare_parameter('sector_deg', 30.0)
        angles = self.get_parameter('probe_angles_deg').value
        self._angles = [math.radians(a) for a in angles]
        self._min = float(self.get_parameter('min_range').value)
        self._max = float(self.get_parameter('max_range').value)
        self._blind = float(self.get_parameter('blind_zone_m').value)
        self._sector = math.radians(float(self.get_parameter('sector_deg').value))
        self._fw = int(self.get_parameter('filter_window').value)
        self._streak = int(self.get_parameter('min_valid_streak').value)
        self._hist = [None] * 4   # recent valid distances per probe
        self._cnt = [0] * 4       # consecutive valid frames per probe
        self._pub = self.create_publisher(
            LaserScan, str(self.get_parameter('output_topic').value), 10)
        self.create_subscription(UltrasonicArray, '/ultrasonic_array', self._cb, 10)
        self.get_logger().info('ultrasonic->laserscan ready')

    def _cb(self, msg):
        out = LaserScan()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.angle_min = -math.pi
        out.angle_max = math.pi
        out.angle_increment = 0.02
        n = int(round((math.pi * 2) / 0.02))
        out.range_min = self._min
        out.range_max = self._max
        out.ranges = [float('inf')] * n
        vals = [float(v) for v in msg.ranges]
        raw = []
        for i in range(4):
            v = vals[i]
            ok = 0.15 < v < 2.55
            raw.append(v if ok else float('nan'))
        # ghost suppression: median window + consecutive-valid streak
        for i in (0, 1):  # front probes only
            h = self._hist[i]
            if h is None:
                h = []
            v = raw[i]
            if math.isnan(v):
                self._cnt[i] = 0
                h = []
                continue
            h.append(v)
            if len(h) > self._fw:
                h = h[-self._fw:]
            if len(h) < self._fw:
                continue
            med = sorted(h)[len(h) // 2]
            if not (self._min <= med <= self._max):
                self._cnt[i] = 0
                continue
            self._cnt[i] += 1
            if self._cnt[i] < self._streak:
                continue
            self._hist[i] = h
            a = self._angles[i]
            if med < 0.28:
                med = max(med, self._blind)  # blind-zone floor 25 cm
            lo = max(0.0, a - self._sector / 2)
            hi = min(2 * math.pi, a + self._sector / 2)
            k0 = int(round((lo - out.angle_min) / out.angle_increment))
            k1 = int(round((hi - out.angle_min) / out.angle_increment))
            for k in range(k0, k1 + 1):
                if 0 <= k < n:
                    out.ranges[k] = min(out.ranges[k], med)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
