#!/usr/bin/env python3
"""Relay Angstrong vendor topics onto Gen2 /camera/front_up|front_down/... contracts.

PointCloud (front cam only when manage_pointcloud_control:=true):
  /xw/camera/set_pointcloud      — manual (persists preference)
  /xw/camera/set_pointcloud_nav  — nav auto (no persist; OR with manual)
  /xw/camera/pointcloud_enabled  — latched effective state

Raw RGB relay is gated by /xw/fall/enable OR /xw/follow/enable when
gate_rgb_on_sessions:=true (front). Depth image always relays.
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
        self.declare_parameter('rgb_image_out', '/camera/front_up/color/image_raw')
        self.declare_parameter('rgb_info_out', '/camera/front_up/color/camera_info')
        self.declare_parameter('depth_image_out', '/camera/front_up/depth/image_raw')
        self.declare_parameter('depth_info_out', '/camera/front_up/depth/camera_info')
        self.declare_parameter('compressed_out', '/camera/front_up/color/image_raw/compressed')
        self.declare_parameter('points_out', '/camera/front_up/depth/points')
        self.declare_parameter('preview_fps', 5.0)
        self.declare_parameter('points_fps', 3.0)
        self.declare_parameter('relay_raw_rgb', False)  # force always-on if true
        self.declare_parameter('enable_pointcloud', False)
        self.declare_parameter('persist_path', _default_persist_path())
        # Only one bridge should own global /xw/camera/set_pointcloud* (front cam).
        self.declare_parameter('manage_pointcloud_control', True)
        # When not managing, optionally mirror /xw/camera/pointcloud_enabled (dual-cam nav).
        self.declare_parameter('follow_pointcloud_enabled_topic', False)
        # When false, never subscribe fall/follow for raw RGB (front_down).
        self.declare_parameter('gate_rgb_on_sessions', True)

        self._preview_period = 1.0 / max(0.5, float(self.get_parameter('preview_fps').value))
        self._points_period = 1.0 / max(0.5, float(self.get_parameter('points_fps').value))
        self._last_preview = 0.0
        self._last_points = 0.0
        self._have_depth = False
        self._have_preview = False
        self._have_points = False
        self._have_rgb = False
        self._preview_frames = 0
        self._points_frames = 0
        self._rgb_frames = 0
        self._last_status = 0.0

        self._points_sub = None
        self._rgb_sub = None
        self._persist_path = str(self.get_parameter('persist_path').value)
        self._manage_pc = bool(self.get_parameter('manage_pointcloud_control').value)
        self._follow_pc_topic = bool(self.get_parameter('follow_pointcloud_enabled_topic').value)
        self._gate_rgb = bool(self.get_parameter('gate_rgb_on_sessions').value)

        # Manual preference (persisted) OR nav auto → effective pointcloud.
        launch_default = bool(self.get_parameter('enable_pointcloud').value)
        persisted = _read_persist(self._persist_path) if self._manage_pc else None
        self._manual_pc = launch_default if persisted is None else persisted
        self._nav_auto_pc = False

        self._fall_en = False
        self._follow_en = False
        self._force_rgb = bool(self.get_parameter('relay_raw_rgb').value)

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
        self._enabled_pub = None
        if self._manage_pc:
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
        if self._gate_rgb:
            self.create_subscription(Bool, '/xw/fall/enable', self._on_fall_en, _LATCHED_QOS)
            self.create_subscription(Bool, '/xw/follow/enable', self._on_follow_en, _LATCHED_QOS)
        if self._follow_pc_topic and not self._manage_pc:
            self.create_subscription(
                Bool, '/xw/camera/pointcloud_enabled', self._on_pc_enabled_mirror, _LATCHED_QOS
            )

        self._points_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('points_out').value), _POINTS_QOS
        )

        if self._manage_pc:
            self.create_service(SetBool, '/xw/camera/set_pointcloud', self._on_set_pointcloud)
            self.create_service(SetBool, '/xw/camera/set_pointcloud_nav', self._on_set_pointcloud_nav)
        self.create_timer(2.0, self._status)

        self._sync_pointcloud()
        self._sync_rgb_relay()
        self._publish_enabled()

        self.get_logger().info(
            f'depth bridge ready out={self.get_parameter("depth_image_out").value} '
            f'preview_fps={self.get_parameter("preview_fps").value} '
            f'manual_pc={self._manual_pc} manage_pc={self._manage_pc} '
            f'follow_pc_topic={self._follow_pc_topic} '
            f'gate_rgb={self._gate_rgb} '
            f'points_fps={self.get_parameter("points_fps").value} '
            f'persist={self._persist_path}'
        )

    @property
    def _pc_wanted(self) -> bool:
        return bool(self._manual_pc or self._nav_auto_pc)

    @property
    def _rgb_wanted(self) -> bool:
        return bool(self._force_rgb or self._fall_en or self._follow_en)

    def _publish_enabled(self) -> None:
        if self._enabled_pub is None:
            return
        msg = Bool()
        msg.data = bool(self._pc_wanted and self._points_sub is not None)
        self._enabled_pub.publish(msg)

    def _start_pointcloud(self) -> None:
        if self._points_sub is None:
            self._points_sub = self.create_subscription(
                PointCloud2,
                str(self.get_parameter('points_in').value),
                self._on_points,
                _POINTS_QOS,
            )
            out = str(self.get_parameter('points_out').value)
            self.get_logger().info(f'pointcloud relay ON → {out}')

    def _stop_pointcloud(self) -> None:
        if self._points_sub is not None:
            try:
                self.destroy_subscription(self._points_sub)
            except Exception:  # noqa: BLE001
                pass
            self._points_sub = None
            self._have_points = False
            self.get_logger().info('pointcloud relay OFF')

    def _sync_pointcloud(self) -> None:
        if self._pc_wanted:
            self._start_pointcloud()
        else:
            self._stop_pointcloud()
        self._publish_enabled()

    def _start_rgb(self) -> None:
        if self._rgb_sub is None:
            self._rgb_sub = self.create_subscription(
                Image, str(self.get_parameter('rgb_image_in').value), self._on_rgb, _SENSOR_QOS
            )
            out = str(self.get_parameter('rgb_image_out').value)
            self.get_logger().info(f'raw RGB relay ON → {out}')

    def _stop_rgb(self) -> None:
        if self._rgb_sub is not None:
            try:
                self.destroy_subscription(self._rgb_sub)
            except Exception:  # noqa: BLE001
                pass
            self._rgb_sub = None
            self._have_rgb = False
            self.get_logger().info('raw RGB relay OFF')

    def _sync_rgb_relay(self) -> None:
        if self._rgb_wanted:
            self._start_rgb()
        else:
            self._stop_rgb()

    def _on_fall_en(self, msg: Bool) -> None:
        self._fall_en = bool(msg.data)
        self._sync_rgb_relay()

    def _on_follow_en(self, msg: Bool) -> None:
        self._follow_en = bool(msg.data)
        self._sync_rgb_relay()

    def _on_pc_enabled_mirror(self, msg: Bool) -> None:
        """Front_2 mirrors primary bridge's effective pointcloud state (nav auto)."""
        wanted = bool(msg.data)
        if wanted == self._nav_auto_pc and wanted == self._pc_wanted:
            return
        self._nav_auto_pc = wanted
        self._manual_pc = False
        self._sync_pointcloud()

    def _on_set_pointcloud(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        """Manual toggle — persists preference."""
        self._manual_pc = bool(req.data)
        _write_persist(self._persist_path, self._manual_pc)
        self._sync_pointcloud()
        res.success = True
        res.message = (
            f'pointcloud={"on" if self._pc_wanted else "off"} '
            f'(manual={self._manual_pc}, nav_auto={self._nav_auto_pc})'
        )
        return res

    def _on_set_pointcloud_nav(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        """Nav auto toggle — does NOT write persist."""
        self._nav_auto_pc = bool(req.data)
        self._sync_pointcloud()
        res.success = True
        res.message = (
            f'pointcloud={"on" if self._pc_wanted else "off"} '
            f'(manual={self._manual_pc}, nav_auto={self._nav_auto_pc})'
        )
        return res

    def _on_rgb(self, msg: Image) -> None:
        self._have_rgb = True
        self._rgb_frames += 1
        self._rgb_pub.publish(msg)

    def _on_rgb_info(self, msg: CameraInfo) -> None:
        if self._rgb_wanted:
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
        if not self._pc_wanted or self._points_pub is None:
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
        self._publish_enabled()
        pc = 'off'
        if self._pc_wanted and self._points_pub is not None:
            pc = (
                f'{"ok" if self._have_points else "wait"} '
                f'frames={self._points_frames} '
                f'subs={self._points_pub.get_subscription_count()} '
                f'manual={self._manual_pc} nav={self._nav_auto_pc}'
            )
        self.get_logger().info(
            f'bridge depth={"ok" if self._have_depth else "wait"} '
            f'rgb={"ok" if self._have_rgb else ("gated" if not self._rgb_wanted else "wait")} '
            f'rgb_frames={self._rgb_frames} '
            f'preview={"ok" if self._have_preview else "idle"} '
            f'preview_frames={self._preview_frames} '
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
