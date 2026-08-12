#!/usr/bin/env python3
"""Write topic health status file for shell watchdog."""

import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool


_CAM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TopicHealthNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_topic_health')
        default_log = os.environ.get('XW_LOG', '/ros2_ws/log')
        self.declare_parameter('status_file', str(Path(default_log) / 'topic_health_status'))
        self.declare_parameter('stale_sec', 2.0)
        self.declare_parameter('watch_depth', True)

        self._last = {
            'scan': 0.0,
            'safety_status': 0.0,
            'cmd_vel': 0.0,
            'camera_depth': 0.0,
        }
        path = Path(str(self.get_parameter('status_file').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        self._path = path

        self.create_subscription(LaserScan, 'scan', lambda m: self._touch('scan'), 10)
        self.create_subscription(Bool, 'safety_status', lambda m: self._touch('safety_status'), 10)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self._touch('cmd_vel'), 10)
        if bool(self.get_parameter('watch_depth').value):
            self.create_subscription(
                Image,
                '/camera/front/depth/image_raw',
                lambda m: self._touch('camera_depth'),
                _CAM_QOS,
            )
        self.create_timer(0.5, self._write)
        self.get_logger().info(f'health -> {self._path}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _touch(self, key: str) -> None:
        self._last[key] = self._now()

    def _write(self) -> None:
        stale = float(self.get_parameter('stale_sec').value)
        now = self._now()
        lines = ['monitor_ok: alive']
        for k, t in self._last.items():
            alive = t > 0 and (now - t) < stale
            lines.append(f'{k}: {"alive" if alive else "dead"}')
        self._path.write_text('\n'.join(lines) + '\n')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicHealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
