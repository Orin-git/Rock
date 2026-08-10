#!/usr/bin/env python3
"""Safety gate: gated cmd + scan/ultrasonic -> /cmd_vel."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from xw_interfaces.msg import UltrasonicArray


class SafetyGateNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_safety_gate')
        self.declare_parameter('safety_distance', 0.35)
        self.declare_parameter('front_angle_deg', 40.0)
        self.declare_parameter('ultrasonic_stop_m', 0.25)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('use_ultrasonic', True)
        self.declare_parameter('use_depth', False)

        self._last_cmd = Twist()
        self._scan: Optional[LaserScan] = None
        self._ultra: Optional[UltrasonicArray] = None
        self._safety_ok = True

        self.create_subscription(Twist, '/xw/cmd/gated', self._on_cmd, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_subscription(UltrasonicArray, '/ultrasonic_array', self._on_ultra, 10)

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._safe_pub = self.create_publisher(Bool, 'safety_status', 10)
        self._obs_pub = self.create_publisher(String, 'obstacle_status', 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info('safety gate ready')

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_ultra(self, msg: UltrasonicArray) -> None:
        self._ultra = msg

    def _front_min_lidar(self) -> Optional[float]:
        if not self.get_parameter('use_lidar').value or self._scan is None:
            return None
        scan = self._scan
        half = math.radians(float(self.get_parameter('front_angle_deg').value))
        mins = []
        angle = scan.angle_min
        for r in scan.ranges:
            if -half <= angle <= half:
                if scan.range_min < r < scan.range_max and math.isfinite(r):
                    mins.append(r)
            angle += scan.angle_increment
        return min(mins) if mins else None

    def _front_min_ultra(self) -> Optional[float]:
        if not self.get_parameter('use_ultrasonic').value or self._ultra is None:
            return None
        if not self._ultra.ranges:
            return None
        # Prefer labels containing front; else min of all
        vals = []
        for r, label in zip(self._ultra.ranges, self._ultra.labels or []):
            if 'front' in label.lower() or 'f' == label.lower():
                vals.append(r)
        if not vals:
            vals = list(self._ultra.ranges)
        return min(vals) if vals else None

    def _tick(self) -> None:
        stop_lidar = float(self.get_parameter('safety_distance').value)
        stop_ultra = float(self.get_parameter('ultrasonic_stop_m').value)
        d_lidar = self._front_min_lidar()
        d_ultra = self._front_min_ultra()

        blocked = False
        reason = 'clear'
        if d_lidar is not None and d_lidar < stop_lidar:
            blocked = True
            reason = f'lidar:{d_lidar:.2f}'
        if d_ultra is not None and d_ultra < stop_ultra:
            blocked = True
            reason = f'ultra:{d_ultra:.2f}'

        out = Twist()
        out.linear.x = self._last_cmd.linear.x
        out.angular.z = self._last_cmd.angular.z

        # Allow reverse and in-place turn when blocked forward
        if blocked and out.linear.x > 0.0:
            out.linear.x = 0.0
            self._safety_ok = False
        else:
            self._safety_ok = not blocked or out.linear.x <= 0.0

        self._cmd_pub.publish(out)
        st = Bool()
        st.data = self._safety_ok
        self._safe_pub.publish(st)
        obs = String()
        obs.data = f'{{"blocked": {str(blocked).lower()}, "reason": "{reason}"}}'
        self._obs_pub.publish(obs)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
