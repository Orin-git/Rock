#!/usr/bin/env python3
"""Relay Angstrong vendor topics onto Gen2 /camera/front/... contracts.

PointCloud is runtime-toggleable (default off) via:
  /xw/camera/set_pointcloud  (std_srvs/SetBool)
  /xw/camera/pointcloud_enabled  (latched Bool)
Preference persisted under XW_WS/config/enable_pointcloud.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ascamera PointCloud2 publishers are RELIABLE; must match or no data / no stream enable.
_POINTS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _default_persist_path() -> str:
    ws = os.environ.get('XW_WS', '/ros2_ws')
    return str(Path(ws) / 'config' / 'enable_pointcloud')


def _read_persist(path: str) -> Optional[bool]:
    try:
        p = Path(path)
        if not p.is_file():
            return None
        raw = p.read_text(encoding='utf-8').strip().lower()
        if raw in ('1', 'true', 'yes', 'on'):
            return True
        if raw in ('0', 'false', 'no', 'off'):
            return False
    except OSError:
        return None
    return None


def _write_persist(path: str, enabled: bool) -> None:
    p = Path(path)
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('true\n' if enabled else 'false\n', encoding='utf-8')
    except OSError:
        pass


class DepthTopicBridge(Node):
    def __init__(self) -> None:
        super().__init__('xw_depth_topic_bridge')
        self.declare_parameter('rgb_image_in', '/ascamera_hp60c/camera_publisher/rgb0/image')
        self.declare_parameter('rgb_info_in', '/ascamera_hp60c/camera_publisher/rgb0/camera_info')
        self.declare_parameter('depth_image_in', '/ascamera_hp60c/camera_publisher/depth0/image_raw')
        self.declare_parameter('depth_info_in', '/ascamera_hp60c/camera_publisher/depth0/camera_info')
        self.declare_parameter('mjpeg_in', '/ascamera_hp60c/camera_publisher/mjpeg0/compressed')
        self.declare_parameter('points_in', '/ascamera_hp60c/camera_publisher/depth0/points')
        self.declare_parameter('rgb_image_out', '/camera/front/color/image_raw')
        self.declare_parameter('rgb_info_out', '/camera/front/color/camera_info')
        self.declare_parameter('depth_image_out', '/camera/front/depth/image_raw')
        self.declare_parameter('depth_info_out', '/camera/front/depth/camera_info')
        self.declare_parameter('compressed_out', '/camera/front/color/image_raw/compressed')
        self.declare_parameter('points_out', '/camera/front/depth/points')
        self.declare_parameter('preview_fps', 5.0)
        self.declare_parameter('points_fps', 3.0)
        self.declare_parameter('relay_raw_rgb', False)
        self.declare_parameter('enable_pointcloud', False)
        self.declare_parameter('persist_path', _default_persist_path())

        self._preview_period = 1.0 / max(0.5, float(self.get_parameter('preview_fps').value))
        self._points_period = 1.0 / max(0.5, float(self.get_parameter('points_fps').value))
        self._last_preview = 0.0
        self._last_points = 0.0
        self._have_depth = False
        self._have_preview = False
        self._have_points = False
        self._preview_frames = 0
        self._points_frames = 0
        self._last_status = 0.0

        self._points_pub = None
        self._points_sub = None
        self._persist_path = str(self.get_parameter('persist_path').value)

        # Persist file wins over launch default when present (web toggle survives restart).
        launch_default = bool(self.get_parameter('enable_pointcloud').value)
        persisted = _read_persist(self._persist_path)
        self._enable_pc = launch_default if persisted is None else persisted

        self._rgb_pub = self.create_publisher(Image, str(self.get_parameter('rgb_image_out').value), _SENSOR_QOS)
        self._rgb_info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('rgb_info_out').value), _SENSOR_QOS
        )
        self._depth_pub = self.create_publisher(
            Image, str(self.get_parameter('depth_image_out').value), _SENSOR_QOS
        )
        self._depth_info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('depth_info_out').value), _SENSOR_QOS
        )
        self._comp_pub = self.create_publisher(
            CompressedImage, str(self.get_parameter('compressed_out').value), _SENSOR_QOS
        )
        self._enabled_pub = self.create_publisher(Bool, '/xw/camera/pointcloud_enabled', _LATCHED_QOS)

        self.create_subscription(
            Image, str(self.get_parameter('depth_image_in').value), self._on_depth, _SENSOR_QOS
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter('depth_info_in').value), self._on_depth_info, _SENSOR_QOS
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter('rgb_info_in').value), self._on_rgb_info, _SENSOR_QOS
        )
        self.create_subscription(
            CompressedImage, str(self.get_parameter('mjpeg_in').value), self._on_mjpeg, _SENSOR_QOS
        )
        if bool(self.get_parameter('relay_raw_rgb').value):
            self.create_subscription(
                Image, str(self.get_parameter('rgb_image_in').value), self._on_rgb, _SENSOR_QOS
            )

        # Always advertise so Foxglove can list the topic; data only when enabled + subscribed.
        self._points_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('points_out').value), _POINTS_QOS
        )

        self.create_service(SetBool, '/xw/camera/set_pointcloud', self._on_set_pointcloud)
        self.create_timer(2.0, self._status)

        if self._enable_pc:
            self._start_pointcloud()
        self._publish_enabled()

        self.get_logger().info(
            f'depth bridge ready preview_fps={self.get_parameter("preview_fps").value} '
            f'enable_pointcloud={self._enable_pc} '
            f'points_fps={self.get_parameter("points_fps").value} '
            f'persist={self._persist_path}'
        )

    def _publish_enabled(self) -> None:
        msg = Bool()
        msg.data = bool(self._enable_pc)
        self._enabled_pub.publish(msg)

    def _start_pointcloud(self) -> None:
        if self._points_sub is None:
            self._points_sub = self.create_subscription(
                PointCloud2,
                str(self.get_parameter('points_in').value),
                self._on_points,
                _POINTS_QOS,
            )
        self._enable_pc = True
        self.get_logger().info('pointcloud relay ON → /camera/front/depth/points')

    def _stop_pointcloud(self) -> None:
        if self._points_sub is not None:
            try:
                self.destroy_subscription(self._points_sub)
            except Exception:  # noqa: BLE001
                pass
            self._points_sub = None
        # Keep publisher advertised for Foxglove topic list.
        self._enable_pc = False
        self._have_points = False
        self.get_logger().info('pointcloud relay OFF')

    def _on_set_pointcloud(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        want = bool(req.data)
        if want and not self._enable_pc:
            self._start_pointcloud()
        elif (not want) and self._enable_pc:
            self._stop_pointcloud()
        _write_persist(self._persist_path, self._enable_pc)
        self._publish_enabled()
        res.success = True
        res.message = f'pointcloud={"on" if self._enable_pc else "off"}'
        return res

    def _on_rgb(self, msg: Image) -> None:
        self._rgb_pub.publish(msg)

    def _on_rgb_info(self, msg: CameraInfo) -> None:
        self._rgb_info_pub.publish(msg)

    def _on_depth(self, msg: Image) -> None:
        self._have_depth = True
        self._depth_pub.publish(msg)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._depth_info_pub.publish(msg)

    def _on_mjpeg(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        if now - self._last_preview < self._preview_period:
            return
        if self._comp_pub.get_subscription_count() < 1:
            return
        self._last_preview = now
        self._have_preview = True
        self._preview_frames += 1
        self._comp_pub.publish(msg)

    def _on_points(self, msg: PointCloud2) -> None:
        if not self._enable_pc or self._points_pub is None:
            return
        now = time.monotonic()
        if now - self._last_points < self._points_period:
            return
        if self._points_pub.get_subscription_count() < 1:
            return
        self._last_points = now
        self._have_points = True
        self._points_frames += 1
        self._points_pub.publish(msg)

    def _status(self) -> None:
        now = time.monotonic()
        if now - self._last_status < 10.0:
            return
        self._last_status = now
        # Re-latch enabled state for late subscribers
        self._publish_enabled()
        pc = 'off'
        if self._enable_pc and self._points_pub is not None:
            pc = (
                f'{"ok" if self._have_points else "wait"} '
                f'frames={self._points_frames} '
                f'subs={self._points_pub.get_subscription_count()}'
            )
        self.get_logger().info(
            f'bridge depth={"ok" if self._have_depth else "wait"} '
            f'preview={"ok" if self._have_preview else "idle"} '
            f'preview_frames={self._preview_frames} '
            f'preview_subs={self._comp_pub.get_subscription_count()} '
            f'pointcloud={pc}'
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = DepthTopicBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
