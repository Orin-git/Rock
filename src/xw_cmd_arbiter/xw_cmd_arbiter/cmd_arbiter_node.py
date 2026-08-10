#!/usr/bin/env python3
"""Arbitrate multi-source cmd to single gated stream."""

from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


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
        self._estop = False

        for name in SOURCE_PRIORITY:
            self.create_subscription(
                Twist,
                f'/xw/cmd/{name}',
                lambda msg, n=name: self._on_cmd(n, msg),
                10,
            )

        self.create_subscription(Bool, 'emergency_stop', self._on_estop, 10)
        self._pub = self.create_publisher(Twist, '/xw/cmd/gated', 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info('cmd arbiter ready')

    def _on_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)

    def _on_cmd(self, name: str, msg: Twist) -> None:
        self._sources[name] = (msg, self.get_clock().now().nanoseconds * 1e-9)

    def _select(self) -> Optional[Twist]:
        if self._estop:
            return Twist()
        timeout = float(self.get_parameter('stale_timeout_sec').value)
        now = self.get_clock().now().nanoseconds * 1e-9
        best_name = None
        best_pri = -1
        best_twist = None
        for name, (twist, t) in list(self._sources.items()):
            if now - t > timeout:
                continue
            pri = SOURCE_PRIORITY.get(name, 0)
            if pri > best_pri:
                best_pri = pri
                best_name = name
                best_twist = twist
        return best_twist

    def _tick(self) -> None:
        selected = self._select()
        out = selected if selected is not None else Twist()
        if self._estop:
            out = Twist()
        self._pub.publish(out)


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
