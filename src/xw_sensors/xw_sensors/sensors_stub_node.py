#!/usr/bin/env python3
"""Sensor stubs: LaserScan, ultrasonic, depth clock ticks."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from xw_interfaces.msg import UltrasonicArray


class SensorsStubNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_sensors_stub')
        self.declare_parameter('publish_scan', True)
        self.declare_parameter('publish_ultrasonic', True)
        self.declare_parameter('scan_frame', 'lidar_link')
        self.declare_parameter('clear_ranges', True)

        self._scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self._ultra_pub = self.create_publisher(UltrasonicArray, '/ultrasonic_array', 10)
        self.create_timer(0.1, self._publish)
        self.get_logger().info('sensors stub publishing clear field')

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        if self.get_parameter('publish_scan').value:
            scan = LaserScan()
            scan.header = Header(stamp=now, frame_id=str(self.get_parameter('scan_frame').value))
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = math.radians(1.0)
            n = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
            scan.range_min = 0.1
            scan.range_max = 12.0
            # clear = far
            scan.ranges = [10.0] * n
            self._scan_pub.publish(scan)

        if self.get_parameter('publish_ultrasonic').value:
            u = UltrasonicArray()
            u.stamp = now
            u.ranges = [2.0, 2.0, 2.0, 2.0]
            u.labels = ['front', 'left', 'right', 'rear']
            self._ultra_pub.publish(u)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorsStubNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
