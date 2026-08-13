#!/usr/bin/env python3
"""Safety gate: gated cmd + scan/ultrasonic/depth -> /cmd_vel.

Publishes:
  safety_status (Bool) — overall gate pass
  obstacle_status (String JSON) — overall + front/rear/left/right sectors
"""

from __future__ import annotations

import json
import math
import struct
from typing import Any, Dict, Optional, Tuple

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


def _ang_diff(a: float, b: float) -> float:
    """Smallest signed difference a-b in (-pi, pi]."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class SafetyGateNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_safety_gate')
        self.declare_parameter('safety_distance', 0.35)
        self.declare_parameter('front_angle_deg', 40.0)
        self.declare_parameter('sector_angle_deg', 40.0)
        # Scan angles are in lidar_link. If lidar_joint yaw=π (180° mount), offset so
        # sector "front" matches base_link +X. Keep in sync with xw_gen2.urdf lidar_joint.
        self.declare_parameter('lidar_yaw_offset_rad', 3.141592653589793)
        self.declare_parameter('lidar_ignore_below_m', 0.20)
        self.declare_parameter('ultrasonic_stop_m', 0.25)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('use_ultrasonic', True)
        self.declare_parameter('use_depth', False)
        self.declare_parameter('depth_topic', '/camera/front_up/depth/image_raw')
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
        self.get_logger().info('safety gate ready (4-sector obstacle_status)')

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

    def _sector_min_lidar(self, center_rad: float, half_rad: float) -> Optional[float]:
        if not self.get_parameter('use_lidar').value or self._scan is None:
            return None
        scan = self._scan
        ignore_below = float(self.get_parameter('lidar_ignore_below_m').value)
        mins = []
        angle = scan.angle_min
        for r in scan.ranges:
            if abs(_ang_diff(angle, center_rad)) <= half_rad:
                lo = max(float(scan.range_min), ignore_below)
                if lo < r < scan.range_max and math.isfinite(r):
                    mins.append(r)
            angle += scan.angle_increment
        return min(mins) if mins else None

    def _ultra_min_for(self, keys: Tuple[str, ...]) -> Optional[float]:
        if not self.get_parameter('use_ultrasonic').value or self._ultra is None:
            return None
        if not self._ultra.ranges:
            return None
        vals = []
        labels = list(self._ultra.labels or [])
        for i, r in enumerate(self._ultra.ranges):
            label = (labels[i] if i < len(labels) else '').lower()
            if any(k in label for k in keys):
                if math.isfinite(r) and r > 0.0:
                    vals.append(float(r))
        return min(vals) if vals else None

    def _pick_range(
        self, *candidates: Tuple[Optional[float], str]
    ) -> Tuple[Optional[float], str]:
        best: Optional[float] = None
        src = ''
        for dist, name in candidates:
            if dist is None:
                continue
            if best is None or dist < best:
                best = dist
                src = name
        return best, src

    def _sector_info(
        self,
        name: str,
        stop_m: float,
        lidar_m: Optional[float],
        ultra_m: Optional[float],
        depth_m: Optional[float] = None,
    ) -> Dict[str, Any]:
        dist, src = self._pick_range(
            (lidar_m, 'lidar'),
            (ultra_m, 'ultra'),
            (depth_m, 'depth'),
        )
        blocked = dist is not None and dist < stop_m
        return {
            'name': name,
            'blocked': blocked,
            'range_m': None if dist is None else round(float(dist), 3),
            'source': src or None,
            'stop_m': round(float(stop_m), 3),
        }

    def _tick(self) -> None:
        stop_lidar = float(self.get_parameter('safety_distance').value)
        stop_ultra = float(self.get_parameter('ultrasonic_stop_m').value)
        stop_depth = float(self.get_parameter('depth_stop_m').value)

        half = math.radians(float(self.get_parameter('sector_angle_deg').value))
        front_half = math.radians(float(self.get_parameter('front_angle_deg').value))
        yaw_off = float(self.get_parameter('lidar_yaw_offset_rad').value)

        # Robot frame: +X forward, +Y left. Scan angles live in lidar_link; apply yaw_off.
        lidar_front = self._sector_min_lidar(0.0 + yaw_off, front_half)
        lidar_left = self._sector_min_lidar(math.pi / 2.0 + yaw_off, half)
        lidar_right = self._sector_min_lidar(-math.pi / 2.0 + yaw_off, half)
        lidar_rear = self._sector_min_lidar(math.pi + yaw_off, half)

        ultra_front = self._ultra_min_for(('front', 'f', '前'))
        ultra_rear = self._ultra_min_for(('rear', 'back', 'aft', '后'))
        ultra_left = self._ultra_min_for(('left', 'l', '左'))
        ultra_right = self._ultra_min_for(('right', 'r', '右'))

        d_depth = self._depth_min if bool(self.get_parameter('use_depth').value) else None

        # Per-sector stop: if winning source is ultra use ultra stop, else lidar/depth stop
        def sector(name: str, lidar_m, ultra_m, depth_m=None) -> Dict[str, Any]:
            dist, src = self._pick_range((lidar_m, 'lidar'), (ultra_m, 'ultra'), (depth_m, 'depth'))
            if src == 'ultra':
                stop = stop_ultra
            elif src == 'depth':
                stop = stop_depth
            else:
                stop = stop_lidar
            blocked = dist is not None and dist < stop
            return {
                'name': name,
                'blocked': blocked,
                'range_m': None if dist is None else round(float(dist), 3),
                'source': src or None,
                'stop_m': round(float(stop), 3),
            }

        sectors = {
            'front': sector('front', lidar_front, ultra_front, d_depth),
            'rear': sector('rear', lidar_rear, ultra_rear),
            'left': sector('left', lidar_left, ultra_left),
            'right': sector('right', lidar_right, ultra_right),
        }

        # Gate behavior remains forward-biased (same as before): block forward motion on front obstacle
        front = sectors['front']
        blocked = bool(front['blocked'])
        reason = 'clear'
        if blocked:
            rm = front.get('range_m')
            src = front.get('source') or 'front'
            reason = f'{src}:{rm:.2f}' if isinstance(rm, (int, float)) else src

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

        any_sector = any(s['blocked'] for s in sectors.values())
        payload = {
            'blocked': blocked,
            'any_sector_blocked': any_sector,
            'safety_ok': bool(self._safety_ok),
            'reason': reason,
            'depth_m': None if d_depth is None else round(float(d_depth), 3),
            'sectors': sectors,
        }
        obs = String()
        obs.data = json.dumps(payload, ensure_ascii=False)
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
