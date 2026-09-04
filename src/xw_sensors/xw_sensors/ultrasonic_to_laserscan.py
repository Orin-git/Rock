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
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 1.5)
        self.declare_parameter('blind_zone_m', 0.25)
        self.declare_parameter('sector_deg', 5.0)
        angles = self.get_parameter('probe_angles_deg').value
        self._angles = [math.radians(a) for a in angles]
        self._min = float(self.get_parameter('min_range').value)
        self._max = float(self.get_parameter('max_range').value)
        self._blind = float(self.get_parameter('blind_zone_m').value)
        self._sector = math.radians(float(self.get_parameter('sector_deg').value))
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
        vals = [int(round(v * 100)) for v in msg.ranges]
        for i in (0, 1):  # front probes only
            if i >= len(vals):
                continue
            cm = vals[i]
            if cm < 15 or cm > 255:
                continue
            dist = cm / 100.0
            if dist < self._min or dist > self._max:
                continue
            a = self._angles[i]
            if dist < 0.28:
                dist = max(dist, self._blind)  # blind-zone floor 25 cm
            lo = max(0.0, a - self._sector / 2)
            hi = min(2 * math.pi, a + self._sector / 2)
            k0 = int(round((lo - out.angle_min) / out.angle_increment))
            k1 = int(round((hi - out.angle_min) / out.angle_increment))
            for k in range(k0, k1 + 1):
                if 0 <= k < n:
                    out.ranges[k] = min(out.ranges[k], dist)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
