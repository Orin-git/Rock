#!/usr/bin/env python3
"""Arbitrate multi-source cmd to single gated stream + active source name."""

from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


# Higher number = higher priority
SOURCE_PRIORITY = {
    'teleop': 50,
    'motion': 40,
    'nav': 30,
    'follow': 20,
    'recharge': 10,
}


class CmdArbiterNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_cmd_arbiter')
        self.declare_parameter('stale_timeout_sec', 0.4)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._sources: Dict[str, Tuple[Twist, float]] = {}
        self._last_source = ''

        for name in SOURCE_PRIORITY:
            self.create_subscription(
                Twist,
                f'/xw/cmd/{name}',
                lambda msg, n=name: self._on_cmd(n, msg),
                10,
            )

        self._pub = self.create_publisher(Twist, '/xw/cmd/gated', 10)
        self._src_pub = self.create_publisher(String, '/xw/cmd/active_source', 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info('cmd arbiter ready (active_source)')

    @staticmethod
    def _is_active(msg: Twist, eps: float = 1e-3) -> bool:
        return (
            abs(msg.linear.x) > eps
            or abs(msg.linear.y) > eps
            or abs(msg.angular.z) > eps
        )

    def _on_cmd(self, name: str, msg: Twist) -> None:
        # Near-zero means this source is idle. If we kept zeros as "fresh nav",
        # a stopped controller would starve other sources and force gated=0.
        if self._is_active(msg):
            self._sources[name] = (msg, self.get_clock().now().nanoseconds * 1e-9)
        else:
            self._sources.pop(name, None)

    def _select(self) -> Tuple[Optional[Twist], str]:
        timeout = float(self.get_parameter('stale_timeout_sec').value)
        now = self.get_clock().now().nanoseconds * 1e-9
        best_pri = -1
        best_twist = None
        best_name = ''
        for name, (twist, t) in list(self._sources.items()):
            if now - t > timeout:
                self._sources.pop(name, None)
                continue
            if not self._is_active(twist):
                continue
            pri = SOURCE_PRIORITY.get(name, 0)
            if pri > best_pri:
                best_pri = pri
                best_twist = twist
                best_name = name
        return best_twist, best_name

    def _tick(self) -> None:
        selected, name = self._select()
        out = selected if selected is not None else Twist()
        self._pub.publish(out)
        if name != self._last_source:
            self._last_source = name
        src = String()
        src.data = name
        self._src_pub.publish(src)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdArbiterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
