#!/usr/bin/env python3
"""Relay Angstrong vendor topics onto Gen2 /camera/front_up|front_down/... contracts.

PointCloud (front cam only when manage_pointcloud_control:=true):
  /xw/camera/set_pointcloud      — manual (persists preference)
  /xw/camera/set_pointcloud_nav  — nav auto (no persist; OR with manual)
  /xw/camera/pointcloud_enabled  — latched effective state

Raw RGB relay is gated by /xw/fall/enable OR /xw/follow/enable when
gate_rgb_on_sessions:=true (front). Depth image relays when relay_depth:=true
(legacy). When use_legacy_depth_bridge:=false, launch remaps depth and this
node sets relay_depth:=false.

Phase1 Task D lazy rules:
  - MJPEG vendor sub only while compressed_out has subscribers
  - RGB CameraInfo only while RGB relay is wanted (cached)
  - Depth CameraInfo: optional one-shot cache then unsubscribe (legacy path)
"""

from __future__ import annotations

import os
import json
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Match SENSOR_DATA / pc_nav_filter (BEST_EFFORT) — RELIABLE mismatches drop the link.
_POINTS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
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
        # Task C: false when launch remaps vendor depth → public topics.
        self.declare_parameter('relay_depth', True)
        # Task D lazy flags (safe defaults on).
        self.declare_parameter('lazy_mjpeg', True)
        self.declare_parameter('lazy_rgb_info', True)
        self.declare_parameter('cache_depth_info', True)
        # Task E: honor /xw/perception/profile for RGB (nav must not keep RGB on via fall latch).
        self.declare_parameter('gate_rgb_on_profile', True)

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
        self._mjpeg_sub = None
        self._rgb_info_sub = None
        self._depth_info_sub = None
        self._cached_rgb_info: Optional[CameraInfo] = None
        self._cached_depth_info: Optional[CameraInfo] = None
        self._depth_info_cached_done = False

        self._persist_path = str(self.get_parameter('persist_path').value)
        self._manage_pc = bool(self.get_parameter('manage_pointcloud_control').value)
        self._follow_pc_topic = bool(self.get_parameter('follow_pointcloud_enabled_topic').value)
        self._gate_rgb = bool(self.get_parameter('gate_rgb_on_sessions').value)
        self._relay_depth = bool(self.get_parameter('relay_depth').value)
        self._lazy_mjpeg = bool(self.get_parameter('lazy_mjpeg').value)
        self._lazy_rgb_info = bool(self.get_parameter('lazy_rgb_info').value)
        self._cache_depth_info = bool(self.get_parameter('cache_depth_info').value)
        self._gate_rgb_on_profile = bool(self.get_parameter('gate_rgb_on_profile').value)
        self._profile = 'IDLE'
        self._profile_rgb_up = False
        self._profile_rgb_down = False
        # Phase2A PoC: ephemeral front_up RGB request (Relocalizer ACTIVE only).
        self._reloc_rgb_request = False
        self._camera_id = 'front_up'
        out_rgb = str(self.get_parameter('rgb_image_out').value)
        if 'front_down' in out_rgb:
            self._camera_id = 'front_down'

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
        self._depth_pub = None
        self._depth_info_pub = None
        if self._relay_depth:
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

        if self._relay_depth:
            self.create_subscription(
                Image, str(self.get_parameter('depth_image_in').value), self._on_depth, _SENSOR_QOS
            )
            self._depth_info_sub = self.create_subscription(
                CameraInfo,
                str(self.get_parameter('depth_info_in').value),
                self._on_depth_info,
                _SENSOR_QOS,
            )

        if self._gate_rgb:
            self.create_subscription(Bool, '/xw/fall/enable', self._on_fall_en, _LATCHED_QOS)
            self.create_subscription(Bool, '/xw/follow/enable', self._on_follow_en, _LATCHED_QOS)
        if self._gate_rgb_on_profile:
            self.create_subscription(
                String, '/xw/perception/profile_config', self._on_profile_cfg, _LATCHED_QOS
            )
        # OR with profile for front_up only; Relocalizer must release when done.
        self.create_subscription(
            Bool, '/xw/reloc/rgb_request', self._on_reloc_rgb_request, _LATCHED_QOS
        )
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
        # Poll outbound preview demand for true lazy MJPEG vendor subscription.
        self.create_timer(0.5, self._sync_mjpeg_relay)

        self._sync_pointcloud()
        self._sync_rgb_relay()
        if not self._lazy_mjpeg:
            self._start_mjpeg()
        self._publish_enabled()

        self.get_logger().info(
            f'depth bridge ready out={self.get_parameter("depth_image_out").value} '
            f'relay_depth={self._relay_depth} lazy_mjpeg={self._lazy_mjpeg} '
            f'lazy_rgb_info={self._lazy_rgb_info} '
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
        if self._force_rgb:
            return True
        if self._camera_id == 'front_up' and self._reloc_rgb_request:
            return True
        if self._follow_en:
            # Follow uses front_up primarily; down bridge stays off unless profile says so.
            if self._camera_id == 'front_down':
                return bool(self._gate_rgb_on_profile and self._profile_rgb_down)
            return True
        if self._gate_rgb_on_profile:
            if self._camera_id == 'front_down':
                return bool(self._profile_rgb_down)
            return bool(self._profile_rgb_up)
        return bool(self._fall_en)

    def _on_reloc_rgb_request(self, msg: Bool) -> None:
        # front_down bridge ignores; never turn on down cam for reloc.
        if self._camera_id != 'front_up':
            return
        wanted = bool(msg.data)
        if wanted == self._reloc_rgb_request:
            return
        self._reloc_rgb_request = wanted
        self.get_logger().info(f'reloc rgb_request → {wanted}')
        self._sync_rgb_relay()

    def _on_profile_cfg(self, msg: String) -> None:
        try:
            cfg = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            return
        self._profile = str(cfg.get('profile') or self._profile)
        self._profile_rgb_up = bool(cfg.get('rgb_up', False))
        self._profile_rgb_down = bool(cfg.get('rgb_down', False))
        self._sync_rgb_relay()

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
        self._sync_rgb_info()

    def _stop_rgb(self) -> None:
        if self._rgb_sub is not None:
            try:
                self.destroy_subscription(self._rgb_sub)
            except Exception:  # noqa: BLE001
                pass
            self._rgb_sub = None
            self._have_rgb = False
            self.get_logger().info('raw RGB relay OFF')
        self._sync_rgb_info()

    def _sync_rgb_relay(self) -> None:
        if self._rgb_wanted:
            self._start_rgb()
        else:
            self._stop_rgb()

    def _start_rgb_info(self) -> None:
        if self._rgb_info_sub is not None:
            return
        self._rgb_info_sub = self.create_subscription(
            CameraInfo,
            str(self.get_parameter('rgb_info_in').value),
            self._on_rgb_info,
            _SENSOR_QOS,
        )

    def _stop_rgb_info(self) -> None:
        if self._rgb_info_sub is None:
            return
        try:
            self.destroy_subscription(self._rgb_info_sub)
        except Exception:  # noqa: BLE001
            pass
        self._rgb_info_sub = None

    def _sync_rgb_info(self) -> None:
        """Avoid permanent rgb CameraInfo sub lighting vendor RGB stream."""
        if not self._lazy_rgb_info:
            if self._rgb_info_sub is None:
                self._start_rgb_info()
            return
        if self._rgb_wanted:
            if self._cached_rgb_info is not None:
                # Republish cache; optional refresh sub briefly not required.
                self._rgb_info_pub.publish(self._cached_rgb_info)
            self._start_rgb_info()
        else:
            self._stop_rgb_info()

    def _start_mjpeg(self) -> None:
        if self._mjpeg_sub is not None:
            return
        self._mjpeg_sub = self.create_subscription(
            CompressedImage,
            str(self.get_parameter('mjpeg_in').value),
            self._on_mjpeg,
            _SENSOR_QOS,
        )
        self.get_logger().info('mjpeg vendor sub ON (preview demand)')

    def _stop_mjpeg(self) -> None:
        if self._mjpeg_sub is None:
            return
        try:
            self.destroy_subscription(self._mjpeg_sub)
        except Exception:  # noqa: BLE001
            pass
        self._mjpeg_sub = None
        self.get_logger().info('mjpeg vendor sub OFF (no preview consumers)')

    def _sync_mjpeg_relay(self) -> None:
        if not self._lazy_mjpeg:
            return
        want = self._comp_pub.get_subscription_count() >= 1
        if want:
            self._start_mjpeg()
        else:
            self._stop_mjpeg()

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
        if self._cached_rgb_info is not None:
            info = self._cached_rgb_info
            info.header = msg.header
            self._rgb_info_pub.publish(info)

    def _on_rgb_info(self, msg: CameraInfo) -> None:
        self._cached_rgb_info = msg
        if self._rgb_wanted:
            self._rgb_info_pub.publish(msg)

    def _on_depth(self, msg: Image) -> None:
        if self._depth_pub is None:
            return
        self._have_depth = True
        self._depth_pub.publish(msg)
        if self._cached_depth_info is not None and self._depth_info_pub is not None:
            info = self._cached_depth_info
            info.header = msg.header
            self._depth_info_pub.publish(info)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        if self._depth_info_pub is None:
            return
        self._cached_depth_info = msg
        self._depth_info_pub.publish(msg)
        # After first sample, drop vendor CameraInfo sub so it cannot alone
        # keep lighting streams if image sub is later removed.
        if self._cache_depth_info and not self._depth_info_cached_done and self._depth_info_sub is not None:
            self._depth_info_cached_done = True
            try:
                self.destroy_subscription(self._depth_info_sub)
            except Exception:  # noqa: BLE001
                pass
            self._depth_info_sub = None
            self.get_logger().info('depth CameraInfo cached; vendor info sub released')

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
        depth_s = 'remap' if not self._relay_depth else ('ok' if self._have_depth else 'wait')
        self.get_logger().info(
            f'bridge depth={depth_s} '
            f'rgb={"ok" if self._have_rgb else ("gated" if not self._rgb_wanted else "wait")} '
            f'rgb_frames={self._rgb_frames} '
            f'preview={"ok" if self._have_preview else "idle"} '
            f'mjpeg_sub={"on" if self._mjpeg_sub else "off"} '
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
