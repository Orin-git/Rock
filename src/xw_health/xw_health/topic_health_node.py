#!/usr/bin/env python3
"""Gen1-style pin probe: one rclpy process, atomically write status file.

Shell watchdog polls the file only — never spawns DDS each cycle.
Critical pins (always expected): scan + safety_status.
Other keys are diagnostics and may be dead when idle / not navigating.
"""

from __future__ import annotations

import os
import tempfile
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Bool, Int8
from tf2_ros import Buffer, TransformException, TransformListener


_CAM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Always-on pins for shell watchdog (Gen1 had scan/safety/ultrasonic).
_CRITICAL = ('scan', 'safety_status')


class TopicHealthNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_topic_health')
        default_log = os.environ.get('XW_LOG', '/ros2_ws/log')
        self.declare_parameter('status_file', str(Path(default_log) / 'topic_health_status'))
        self.declare_parameter('stale_sec', 2.0)
        self.declare_parameter('write_period', 0.5)
        self.declare_parameter('watch_depth', True)
        self.declare_parameter('watch_points_nav', True)
        self.declare_parameter('cmd_period_warn_sec', 0.15)
        self.declare_parameter('tf_probe_period', 2.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')

        self._stale_sec = float(self.get_parameter('stale_sec').value)
        self._last: dict[str, Optional[float]] = {
            'scan': None,
            'safety_status': None,
            'cmd_vel': None,
            'camera_depth': None,
            'points_nav_up': None,
            'points_nav_down': None,
            'localization_status': None,
        }
        self._cmd_intervals: list[float] = []
        self._tf_fail = 0
        self._tf_ok = 0
        self._loc_code = -1
        self._last_tf_probe = 0.0
        self._tf_st = 'unknown'

        path = Path(str(self.get_parameter('status_file').value))
        path.parent.mkdir(parents=True, exist_ok=True)
        self._path = path

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self.create_subscription(
            LaserScan, 'scan', lambda _m: self._touch('scan'), qos_profile_sensor_data
        )
        self.create_subscription(
            Bool, 'safety_status', lambda _m: self._touch('safety_status'), qos_profile_sensor_data
        )
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_subscription(Int8, '/xw/localization_status', self._on_loc, 10)
        if bool(self.get_parameter('watch_depth').value):
            self.create_subscription(
                Image,
                '/camera/front_up/depth/image_raw',
                lambda _m: self._touch('camera_depth'),
                _CAM_QOS,
            )
        if bool(self.get_parameter('watch_points_nav').value):
            self.create_subscription(
                PointCloud2,
                '/camera/front_up/depth/points_nav',
                lambda _m: self._touch('points_nav_up'),
                _CAM_QOS,
            )
            self.create_subscription(
                PointCloud2,
                '/camera/front_down/depth/points_nav',
                lambda _m: self._touch('points_nav_down'),
                _CAM_QOS,
            )

        period = max(0.2, float(self.get_parameter('write_period').value))
        self.create_timer(period, self._write)
        self._write()
        self.get_logger().info(
            f'pin probe -> {self._path} stale_sec={self._stale_sec} '
            f'critical={",".join(_CRITICAL)}'
        )

    def _touch(self, key: str) -> None:
        self._last[key] = time.monotonic()

    def _on_cmd(self, _msg: Twist) -> None:
        now = time.monotonic()
        prev = self._last.get('cmd_vel')
        if prev is not None:
            self._cmd_intervals.append(now - prev)
            if len(self._cmd_intervals) > 40:
                self._cmd_intervals = self._cmd_intervals[-40:]
        self._last['cmd_vel'] = now

    def _on_loc(self, msg: Int8) -> None:
        self._touch('localization_status')
        self._loc_code = int(msg.data)

    def _maybe_probe_tf(self, now: float) -> None:
        period = float(self.get_parameter('tf_probe_period').value)
        if period <= 0 or (now - self._last_tf_probe) < period:
            return
        self._last_tf_probe = now
        map_f = str(self.get_parameter('map_frame').value)
        odom_f = str(self.get_parameter('odom_frame').value)
        try:
            self._tf.lookup_transform(
                map_f,
                odom_f,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            self._tf_ok += 1
            self._tf_st = 'ok'
        except TransformException:
            self._tf_fail += 1
            self._tf_st = 'fail'

    def _alive(self, key: str, now: float) -> bool:
        last = self._last.get(key)
        return last is not None and (now - last) <= self._stale_sec

    def _write(self) -> None:
        now = time.monotonic()
        warn = float(self.get_parameter('cmd_period_warn_sec').value)
        overdue = sum(1 for dt in self._cmd_intervals if dt > warn) if self._cmd_intervals else 0
        self._maybe_probe_tf(now)
        total_tf = max(self._tf_ok + self._tf_fail, 1)
        fail_rate = self._tf_fail / total_tf

        crit_ok = all(self._alive(k, now) for k in _CRITICAL)
        lines = [
            'monitor_ok: alive',
            f'critical_ok: {1 if crit_ok else 0}',
            f'updated_monotonic: {now:.3f}',
            f'updated_wall: {time.time():.3f}',
            f'stale_sec: {self._stale_sec:.3f}',
        ]
        for k in _CRITICAL:
            lines.append(f'{k}: {"alive" if self._alive(k, now) else "dead"}')
        for k in self._last:
            if k in _CRITICAL:
                continue
            lines.append(f'{k}: {"alive" if self._alive(k, now) else "dead"}')
        lines.append(f'localization_code: {self._loc_code}')
        lines.append(f'tf_map_odom: {self._tf_st}')
        lines.append(f'tf_fail_rate: {fail_rate:.3f}')
        lines.append(f'cmd_overdue_count: {overdue}')
        payload = '\n'.join(lines) + '\n'

        directory = str(self._path.parent)
        try:
            fd, tmp_path = tempfile.mkstemp(
                prefix='.topic_health_',
                suffix='.tmp',
                dir=directory,
            )
            try:
                with os.fdopen(fd, 'w', encoding='utf-8') as handle:
                    handle.write(payload)
                    handle.flush()
                    os.fsync(handle.fileno())
                    try:
                        os.fchmod(handle.fileno(), 0o644)
                    except OSError:
                        pass
                os.replace(tmp_path, str(self._path))
                try:
                    os.chmod(self._path, 0o644)
                except OSError:
                    pass
            except Exception:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
                raise
        except Exception as exc:
            self.get_logger().warn(f'failed to write status file: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicHealthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
