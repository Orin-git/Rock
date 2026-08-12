#!/usr/bin/env python3
"""Safety gate: gated cmd + scan/ultrasonic/depth -> /cmd_vel."""

from __future__ import annotations

import math
import struct
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

from xw_interfaces.msg import UltrasonicArray


_DEPTH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class SafetyGateNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_safety_gate')
        self.declare_parameter('safety_distance', 0.35)
        self.declare_parameter('front_angle_deg', 40.0)
        self.declare_parameter('ultrasonic_stop_m', 0.25)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('use_ultrasonic', True)
        self.declare_parameter('use_depth', False)
        self.declare_parameter('depth_topic', '/camera/front/depth/image_raw')
        self.declare_parameter('depth_stop_m', 0.40)
        self.declare_parameter('depth_roi_frac', 0.35)
        self.declare_parameter('depth_min_valid_m', 0.05)
        self.declare_parameter('depth_max_valid_m', 4.0)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('depth_min_hits', 40)

        self._last_cmd = Twist()
        self._scan: Optional[LaserScan] = None
        self._ultra: Optional[UltrasonicArray] = None
        self._depth_min: Optional[float] = None
        self._safety_ok = True

        self.create_subscription(Twist, '/xw/cmd/gated', self._on_cmd, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_subscription(UltrasonicArray, '/ultrasonic_array', self._on_ultra, 10)
        if bool(self.get_parameter('use_depth').value):
            topic = str(self.get_parameter('depth_topic').value)
            self.create_subscription(Image, topic, self._on_depth, _DEPTH_QOS)
            self.get_logger().info(f'depth safety enabled on {topic}')

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

    def _on_depth(self, msg: Image) -> None:
        self._depth_min = self._roi_min_depth(msg)

    def _roi_min_depth(self, msg: Image) -> Optional[float]:
        w = int(msg.width)
        h = int(msg.height)
        if w < 8 or h < 8:
            return None
        frac = float(self.get_parameter('depth_roi_frac').value)
        frac = max(0.1, min(0.9, frac))
        rw = max(1, int(w * frac))
        rh = max(1, int(h * frac))
        x0 = (w - rw) // 2
        y0 = (h - rh) // 2
        scale = float(self.get_parameter('depth_scale').value)
        zmin = float(self.get_parameter('depth_min_valid_m').value)
        zmax = float(self.get_parameter('depth_max_valid_m').value)
        need = int(self.get_parameter('depth_min_hits').value)

        enc = (msg.encoding or '').lower()
        data = bytes(msg.data)
        step = int(msg.step)
        vals = []

        try:
            if enc in ('16uc1', 'mono16'):
                for y in range(y0, y0 + rh):
                    row = y * step
                    for x in range(x0, x0 + rw):
                        off = row + x * 2
                        raw = struct.unpack_from('<H', data, off)[0]
                        if raw == 0:
                            continue
                        z = raw * scale
                        if zmin < z < zmax:
                            vals.append(z)
            elif enc in ('32fc1',):
                for y in range(y0, y0 + rh):
                    row = y * step
                    for x in range(x0, x0 + rw):
                        off = row + x * 4
                        z = struct.unpack_from('<f', data, off)[0]
                        if not math.isfinite(z) or z <= 0.0:
                            continue
                        if z > 20.0:
                            z *= scale
                        if zmin < z < zmax:
                            vals.append(z)
            else:
                return self._depth_min
        except (struct.error, IndexError, ValueError):
            return self._depth_min

        if len(vals) < need:
            return None
        return min(vals)

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
        stop_depth = float(self.get_parameter('depth_stop_m').value)
        d_lidar = self._front_min_lidar()
        d_ultra = self._front_min_ultra()
        d_depth = self._depth_min if bool(self.get_parameter('use_depth').value) else None

        blocked = False
        reason = 'clear'
        if d_lidar is not None and d_lidar < stop_lidar:
            blocked = True
            reason = f'lidar:{d_lidar:.2f}'
        if d_ultra is not None and d_ultra < stop_ultra:
            blocked = True
            reason = f'ultra:{d_ultra:.2f}'
        if d_depth is not None and d_depth < stop_depth:
            blocked = True
            reason = f'depth:{d_depth:.2f}'

        out = Twist()
        out.linear.x = self._last_cmd.linear.x
        out.angular.z = self._last_cmd.angular.z

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
        obs.data = (
            f'{{"blocked": {str(blocked).lower()}, "reason": "{reason}", '
            f'"depth_m": {("null" if d_depth is None else f"{d_depth:.3f}")}}}'
        )
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
