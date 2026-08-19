#!/usr/bin/env python3
"""Write topic health status file for shell watchdog + TF / control overdue."""

import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Bool, Int8
from tf2_ros import Buffer, TransformException, TransformListener


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
        self.declare_parameter('watch_points_nav', True)
        self.declare_parameter('cmd_period_warn_sec', 0.15)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')

        self._last = {
            'scan': 0.0,
            'safety_status': 0.0,
            'cmd_vel': 0.0,
            'camera_depth': 0.0,
            'points_nav_up': 0.0,
            'points_nav_down': 0.0,
            'localization_status': 0.0,
        }
        self._cmd_intervals: list = []
        self._tf_fail = 0
        self._tf_ok = 0
        self._loc_code = -1

        path = Path(str(self.get_parameter('status_file').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        self._path = path

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self.create_subscription(LaserScan, 'scan', lambda m: self._touch('scan'), 10)
        self.create_subscription(Bool, 'safety_status', lambda m: self._touch('safety_status'), 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_subscription(
            Int8, '/xw/localization_status', self._on_loc, 10
        )
        if bool(self.get_parameter('watch_depth').value):
            self.create_subscription(
                Image,
                '/camera/front_up/depth/image_raw',
                lambda m: self._touch('camera_depth'),
                _CAM_QOS,
            )
        if bool(self.get_parameter('watch_points_nav').value):
            self.create_subscription(
                PointCloud2,
                '/camera/front_up/depth/points_nav',
                lambda m: self._touch('points_nav_up'),
                _CAM_QOS,
            )
            self.create_subscription(
                PointCloud2,
                '/camera/front_down/depth/points_nav',
                lambda m: self._touch('points_nav_down'),
                _CAM_QOS,
            )
        self.create_timer(0.5, self._write)
        self.get_logger().info(f'health -> {self._path}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _touch(self, key: str) -> None:
        self._last[key] = self._now()

    def _on_cmd(self, _msg: Twist) -> None:
        now = self._now()
        prev = self._last.get('cmd_vel', 0.0)
        if prev > 0:
            self._cmd_intervals.append(now - prev)
            if len(self._cmd_intervals) > 40:
                self._cmd_intervals = self._cmd_intervals[-40:]
        self._last['cmd_vel'] = now

    def _on_loc(self, msg: Int8) -> None:
        self._touch('localization_status')
        self._loc_code = int(msg.data)

    def _probe_tf(self) -> str:
        map_f = str(self.get_parameter('map_frame').value)
        odom_f = str(self.get_parameter('odom_frame').value)
        try:
            self._tf.lookup_transform(
                map_f, odom_f, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            self._tf_ok += 1
            return 'ok'
        except TransformException:
            self._tf_fail += 1
            return 'fail'

    def _write(self) -> None:
        stale = float(self.get_parameter('stale_sec').value)
        now = self._now()
        warn = float(self.get_parameter('cmd_period_warn_sec').value)
        overdue = 0
        if self._cmd_intervals:
            overdue = sum(1 for dt in self._cmd_intervals if dt > warn)
        tf_st = self._probe_tf()
        total_tf = max(self._tf_ok + self._tf_fail, 1)
        fail_rate = self._tf_fail / total_tf

        lines = ['monitor_ok: alive']
        for k, t in self._last.items():
            alive = t > 0 and (now - t) < stale
            lines.append(f'{k}: {"alive" if alive else "dead"}')
        lines.append(f'localization_code: {self._loc_code}')
        lines.append(f'tf_map_odom: {tf_st}')
        lines.append(f'tf_fail_rate: {fail_rate:.3f}')
        lines.append(f'cmd_overdue_count: {overdue}')
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
