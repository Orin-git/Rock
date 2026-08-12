#!/usr/bin/env python3
"""Person perception: YOLOv8n-pose (RKNN) → tracks + fall (geometry debounce).

Subscribes /camera/front/{color,depth}/image_raw when fall or follow is enabled.
Publishes /xw/perception/tracks and /xw/perception/fall.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

from xw_interfaces.msg import FallStatus, PersonTrack, PersonTracks

from xw_perception.fall_geometry import FallGeometryParams, passes_fall_geometry
from xw_perception.yolov8_pose_post import (
    OBJECT_THRESH,
    decode_rknn_outputs,
    letterbox_resize,
    map_to_original,
)


_SENSOR_QOS = QoSProfile(
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


def _default_model_path() -> str:
    # Prefer share/models, then source tree models/
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('xw_perception'))
        candidates.append(share / 'models' / 'yolov8n-pose.rknn')
    except Exception:  # noqa: BLE001
        pass
    ws = Path(os.environ.get('XW_WS', '/ros2_ws'))
    candidates.append(ws / 'src' / 'xw_perception' / 'models' / 'yolov8n-pose.rknn')
    # Host mirror when running tools outside install
    candidates.append(Path('/home/radxa/ros2_ws/src/xw_perception/models/yolov8n-pose.rknn'))
    for c in candidates:
        if c.is_file():
            return str(c)
    return str(candidates[0] if candidates else 'yolov8n-pose.rknn')


def _imgmsg_to_rgb(msg: Image) -> Optional[np.ndarray]:
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        return None
    data = np.frombuffer(msg.data, dtype=np.uint8)
    enc = (msg.encoding or '').lower()
    try:
        if enc in ('rgb8',):
            return data.reshape(h, w, 3).copy()
        if enc in ('bgr8',):
            import cv2

            bgr = data.reshape(h, w, 3)
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if enc in ('rgba8',):
            return data.reshape(h, w, 4)[:, :, :3].copy()
        if enc in ('bgra8',):
            import cv2

            bgra = data.reshape(h, w, 4)
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2RGB)
        if enc in ('mono8',):
            import cv2

            gray = data.reshape(h, w)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        # Fallback: assume rgb8 packed
        if data.size >= h * w * 3:
            return data[: h * w * 3].reshape(h, w, 3).copy()
    except Exception:  # noqa: BLE001
        return None
    return None


def _depth_median_m(
    depth_msg: Image,
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
) -> float:
    """Median depth (meters) in central ROI of bbox."""
    h, w = int(depth_msg.height), int(depth_msg.width)
    if h <= 0 or w <= 0:
        return 0.0
    x0 = max(0, int(xmin + 0.35 * (xmax - xmin)))
    x1 = min(w, int(xmin + 0.65 * (xmax - xmin)))
    y0 = max(0, int(ymin + 0.35 * (ymax - ymin)))
    y1 = min(h, int(ymin + 0.65 * (ymax - ymin)))
    if x1 <= x0 or y1 <= y0:
        return 0.0
    enc = (depth_msg.encoding or '').lower()
    try:
        if enc in ('16uc1', 'mono16'):
            arr = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(h, w)
            patch = arr[y0:y1, x0:x1].astype(np.float32)
            # Angstrong / common USB depth: mm
            vals = patch[patch > 0] / 1000.0
        elif enc in ('32fc1',):
            arr = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(h, w)
            patch = arr[y0:y1, x0:x1]
            vals = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 8.0)]
        else:
            return 0.0
        if vals.size == 0:
            return 0.0
        return float(np.median(vals))
    except Exception:  # noqa: BLE001
        return 0.0


class RknnPoseBackend:
    def __init__(self, model_path: str, logger) -> None:
        self.ok = False
        self._rknn = None
        self._logger = logger
        try:
            from rknnlite.api import RKNNLite
        except ImportError as exc:
            logger.error(f'rknnlite not available: {exc}')
            return
        if not Path(model_path).is_file():
            logger.error(f'RKNN model missing: {model_path}')
            return
        rknn = RKNNLite()
        ret = rknn.load_rknn(model_path)
        if ret != 0:
            logger.error(f'load_rknn failed ret={ret}')
            return
        ret = rknn.init_runtime()
        if ret != 0:
            logger.error(f'init_runtime failed ret={ret}')
            rknn.release()
            return
        self._rknn = rknn
        self.ok = True
        logger.info(f'RKNN pose ready: {model_path}')

    def infer(self, rgb: np.ndarray, object_thresh: float):
        if not self.ok or self._rknn is None:
            return []
        letterbox_img, aspect, ox, oy = letterbox_resize(rgb, (640, 640), 56)
        # RKNNLite on RK3588 wants 4-D NHWC uint8: [1, H, W, C]
        nhwc = np.ascontiguousarray(letterbox_img[np.newaxis, ...])
        outputs = self._rknn.inference(inputs=[nhwc])
        if outputs is None:
            return []
        boxes = decode_rknn_outputs(outputs, object_thresh=object_thresh)
        return [map_to_original(b, aspect, ox, oy) for b in boxes]

    def release(self) -> None:
        if self._rknn is not None:
            try:
                self._rknn.release()
            except Exception:  # noqa: BLE001
                pass
            self._rknn = None
            self.ok = False


class PersonPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_perception')
        self.declare_parameter('model_path', _default_model_path())
        self.declare_parameter('infer_fps', 6.0)
        self.declare_parameter('object_thresh', 0.35)
        self.declare_parameter('trigger_consecutive_frames', 6)
        self.declare_parameter('clear_consecutive_frames', 6)
        self.declare_parameter('clear_timeout_seconds', 8.0)
        self.declare_parameter('frame_id', 'camera_front_link')
        self.declare_parameter('color_topic', '/camera/front/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/front/depth/image_raw')
        self.declare_parameter('kp_conf_min', 0.35)
        self.declare_parameter('kp_min_visible', 4)
        self.declare_parameter('bbox_aspect_min', 0.55)
        self.declare_parameter('bbox_aspect_min_relaxed', 0.4)
        self.declare_parameter('torso_angle_min_deg', 30.0)
        self.declare_parameter('torso_compression_max', 0.40)
        self.declare_parameter('torso_inversion_margin_ratio', 0.0)
        self.declare_parameter('flat_aspect_min', 1.15)

        self._follow_en = False
        self._fall_en = False
        self._last_infer = 0.0
        self._infer_period = 1.0 / max(1.0, float(self.get_parameter('infer_fps').value))
        self._latest_depth: Optional[Image] = None
        self._img_w = 640
        self._img_h = 480

        self._consec = 0
        self._clear_consec = 0
        self._last_fall_ids: List[int] = []
        self._fallen = False
        self._last_fall_time = 0.0
        self._primary_id = 1
        self._last_primary: Optional[Tuple[float, float, float, float]] = None  # xmin..ymax

        self._geom = FallGeometryParams(
            kp_conf_min=float(self.get_parameter('kp_conf_min').value),
            kp_min_visible=int(self.get_parameter('kp_min_visible').value),
            bbox_aspect_min=float(self.get_parameter('bbox_aspect_min').value),
            bbox_aspect_min_relaxed=float(self.get_parameter('bbox_aspect_min_relaxed').value),
            torso_angle_min_deg=float(self.get_parameter('torso_angle_min_deg').value),
            torso_compression_max=float(self.get_parameter('torso_compression_max').value),
            torso_inversion_margin_ratio=float(
                self.get_parameter('torso_inversion_margin_ratio').value
            ),
            flat_aspect_min=float(self.get_parameter('flat_aspect_min').value),
        )

        self._last_decode_warn = 0.0
        self._last_infer_err = 0.0
        self._last_unavailable_log = 0.0
        self._last_debug_log = 0.0
        self._last_geom_reason = 'init'
        self._last_det_n = 0

        model = str(self.get_parameter('model_path').value)
        self._backend = RknnPoseBackend(model, self.get_logger())

        self._tracks_pub = self.create_publisher(PersonTracks, '/xw/perception/tracks', 10)
        self._fall_pub = self.create_publisher(FallStatus, '/xw/perception/fall', 10)

        self.create_subscription(Bool, '/xw/follow/enable', self._on_follow, _LATCHED_QOS)
        self.create_subscription(Bool, '/xw/fall/enable', self._on_fall, _LATCHED_QOS)
        self.create_subscription(
            Image, str(self.get_parameter('color_topic').value), self._on_color, _SENSOR_QOS
        )
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value), self._on_depth, _SENSOR_QOS
        )
        self.create_timer(1.0, self._tick_clear)

        status = 'RKNN ok' if self._backend.ok else 'RKNN UNAVAILABLE (idle until model/runtime ready)'
        self.get_logger().info(f'person perception ready — {status} model={model}')

    def _on_follow(self, msg: Bool) -> None:
        self._follow_en = bool(msg.data)

    def _on_fall(self, msg: Bool) -> None:
        self._fall_en = bool(msg.data)

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth = msg

    def _active(self) -> bool:
        return bool(self._follow_en or self._fall_en)

    def _on_color(self, msg: Image) -> None:
        if not self._active():
            return
        now = time.monotonic()
        if now - self._last_infer < self._infer_period:
            return
        self._last_infer = now
        self._img_w = int(msg.width) or self._img_w
        self._img_h = int(msg.height) or self._img_h

        if not self._backend.ok:
            if now - self._last_unavailable_log > 10.0:
                self._last_unavailable_log = now
                self.get_logger().warn('perception active but RKNN model/runtime unavailable')
            self._publish_empty()
            return

        rgb = _imgmsg_to_rgb(msg)
        if rgb is None:
            if now - self._last_decode_warn > 5.0:
                self._last_decode_warn = now
                self.get_logger().warn('color decode failed')
            return

        thresh = float(self.get_parameter('object_thresh').value)
        try:
            boxes = self._backend.infer(rgb, thresh)
        except Exception as exc:  # noqa: BLE001
            if now - self._last_infer_err > 2.0:
                self._last_infer_err = now
                self.get_logger().error(f'infer failed: {exc}')
            return

        self._handle_detections(boxes)

    def _match_track_id(self, xmin: float, ymin: float, xmax: float, ymax: float) -> int:
        """Simple IoU sticky id for primary continuity."""
        if self._last_primary is None:
            self._last_primary = (xmin, ymin, xmax, ymax)
            return self._primary_id
        lx0, ly0, lx1, ly1 = self._last_primary
        ix0, iy0 = max(xmin, lx0), max(ymin, ly0)
        ix1, iy1 = min(xmax, lx1), min(ymax, ly1)
        inter = max(0.0, ix1 - ix0) * max(0.0, iy1 - iy0)
        a1 = max(1.0, (xmax - xmin) * (ymax - ymin))
        a2 = max(1.0, (lx1 - lx0) * (ly1 - ly0))
        iou = inter / (a1 + a2 - inter)
        self._last_primary = (xmin, ymin, xmax, ymax)
        if iou <= 0.2:
            self._primary_id += 1
        return self._primary_id

    def _handle_detections(self, boxes) -> None:
        frame_id = str(self.get_parameter('frame_id').value)
        stamp = self.get_clock().now().to_msg()
        cx_img = self._img_w * 0.5

        # Prefer nearest (by depth) else largest bbox area
        scored = []
        for b in boxes:
            dist = 0.0
            if self._latest_depth is not None:
                dist = _depth_median_m(self._latest_depth, b.xmin, b.ymin, b.xmax, b.ymax)
            area = max(1.0, (b.xmax - b.xmin) * (b.ymax - b.ymin))
            scored.append((dist if dist > 0 else 1e6, -area, b, dist))
        scored.sort(key=lambda t: (t[0], t[1]))

        tracks = PersonTracks()
        tracks.stamp = stamp
        tracks.frame_id = frame_id

        fall_ids: List[int] = []
        geom_reasons: List[str] = []
        self._last_det_n = len(scored)
        for i, (_k, _a, b, dist) in enumerate(scored):
            tid = self._match_track_id(b.xmin, b.ymin, b.xmax, b.ymax) if i == 0 else (i + 2)
            # Normalized bearing: -1 left … +1 right
            bx = ((b.xmin + b.xmax) * 0.5 - cx_img) / max(1.0, cx_img)
            if dist <= 0.0:
                # Fallback: inverse bbox height proxy (rough)
                dist = max(0.5, 2.5 * (self._img_h / max(1.0, b.ymax - b.ymin)))
            pt = PersonTrack()
            pt.track_id = tid
            pt.x = float(bx)
            pt.y = 0.0
            pt.z = float(dist)
            pt.distance = float(dist)
            pt.confidence = float(b.score)
            pt.is_primary = i == 0
            tracks.tracks.append(pt)

            if self._fall_en:
                ok, reason = passes_fall_geometry(
                    b.keypoints, b.xmin, b.ymin, b.xmax, b.ymax, self._geom
                )
                geom_reasons.append(reason)
                if ok:
                    fall_ids.append(tid)

        if not scored:
            self._last_geom_reason = 'no_det'
        elif geom_reasons:
            self._last_geom_reason = geom_reasons[0]

        if self._follow_en:
            self._tracks_pub.publish(tracks)

        if self._fall_en:
            self._update_fall(fall_ids, stamp)
            now = time.monotonic()
            if now - self._last_debug_log > 3.0:
                self._last_debug_log = now
                self.get_logger().info(
                    f'fall dbg det={self._last_det_n} consec={self._consec} '
                    f'reason={self._last_geom_reason}'
                )

    def _update_fall(self, fall_ids: List[int], stamp) -> None:
        need = max(1, int(self.get_parameter('trigger_consecutive_frames').value))
        clear_need = max(1, int(self.get_parameter('clear_consecutive_frames').value))
        if fall_ids:
            self._clear_consec = 0
            if self._last_fall_ids and set(fall_ids).isdisjoint(self._last_fall_ids):
                self._consec = 0
            self._consec += 1
            self._last_fall_ids = fall_ids
            if self._consec >= need:
                self._last_fall_time = time.time()
                if not self._fallen:
                    self._fallen = True
                    self.get_logger().warn(
                        f'fall confirmed after {self._consec} frames ids={fall_ids}'
                    )
        else:
            # No fall pose this frame (standing / lost track / no_det) → recovery path
            self._consec = 0
            self._last_fall_ids = []
            if self._fallen:
                self._clear_consec += 1
                if self._clear_consec >= clear_need:
                    self._fallen = False
                    self._clear_consec = 0
                    self.get_logger().info(
                        f'fall cleared after {clear_need} non-fall frames '
                        f'(reason={self._last_geom_reason})'
                    )
            else:
                self._clear_consec = 0

        msg = FallStatus()
        msg.stamp = stamp
        msg.is_fallen = bool(self._fallen)
        msg.confidence = 0.9 if self._fallen else (0.4 if self._consec > 0 else 0.1)
        msg.source = 'yolov8n-pose+geometry'
        if self._fallen:
            msg.detail = (
                f'fallen clear={self._clear_consec}/{clear_need} det={self._last_det_n} '
                f'reason={self._last_geom_reason}'
            )
        else:
            msg.detail = (
                f'consec={self._consec}/{need} det={self._last_det_n} '
                f'reason={self._last_geom_reason}'
            )
        self._fall_pub.publish(msg)

    def _tick_clear(self) -> None:
        """Backup clear if person vanishes and frames stop updating recovery counter."""
        if not self._fallen:
            return
        clear_s = float(self.get_parameter('clear_timeout_seconds').value)
        if time.time() - self._last_fall_time >= clear_s:
            self._fallen = False
            self._consec = 0
            self._clear_consec = 0
            self.get_logger().info(f'fall status cleared by timeout ({clear_s:.0f}s)')

    def _publish_empty(self) -> None:
        stamp = self.get_clock().now().to_msg()
        if self._follow_en:
            t = PersonTracks()
            t.stamp = stamp
            t.frame_id = str(self.get_parameter('frame_id').value)
            self._tracks_pub.publish(t)
        if self._fall_en:
            f = FallStatus()
            f.stamp = stamp
            f.is_fallen = False
            f.confidence = 0.0
            f.source = 'unavailable'
            f.detail = 'rknn model/runtime not ready'
            self._fall_pub.publish(f)

    def destroy_node(self) -> bool:
        self._backend.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
