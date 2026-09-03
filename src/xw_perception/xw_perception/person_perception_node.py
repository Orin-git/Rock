#!/usr/bin/env python3
"""Person perception: YOLOv8n-pose (RKNN) → tracks + fall (geometry debounce).

Cameras
  - front_up   : body-follow (on demand) + fall
  - front_down : fall only
Fall = OR across both cams (either confirms → fallen).
Follow tracks are published only from front_up.

One shared RKNN runtime; cams are time-multiplexed to limit NPU/CPU load.
  fall-only  : alternate up/down @ fall_infer_fps each
  follow on  : prefer up @ follow_infer_fps; still sample down for fall
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from xw_interfaces.msg import FallStatus, PersonTrack, PersonTracks

from xw_perception.fall_geometry import FallGeometryParams, passes_fall_geometry
from xw_perception.tracker import Detection, PersonLockTracker, bbox_iou
from xw_perception.yolov8_pose_post import (
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
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('xw_perception'))
        candidates.append(share / 'models' / 'yolov8n-pose.rknn')
    except Exception:  # noqa: BLE001
        pass
    ws = Path(os.environ.get('XW_WS', '/ros2_ws'))
    candidates.append(ws / 'src' / 'xw_perception' / 'models' / 'yolov8n-pose.rknn')
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
        if data.size >= h * w * 3:
            return data[: h * w * 3].reshape(h, w, 3).copy()
    except Exception:  # noqa: BLE001
        return None
    return None


def _map_bbox_to_raw(
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
    w: int,
    h: int,
    rotate_180: bool,
) -> Tuple[float, float, float, float]:
    if not rotate_180:
        return xmin, ymin, xmax, ymax
    return float(w) - xmax, float(h) - ymax, float(w) - xmin, float(h) - ymin


def _depth_median_m(
    depth_msg: Image,
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
    *,
    rotate_180: bool = False,
) -> float:
    h, w = int(depth_msg.height), int(depth_msg.width)
    if h <= 0 or w <= 0:
        return 0.0
    xmin, ymin, xmax, ymax = _map_bbox_to_raw(xmin, ymin, xmax, ymax, w, h, rotate_180)
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


@dataclass
class CamSlot:
    cam_id: str
    color_topic: str
    depth_topic: str
    frame_id: str
    rotate_180: bool
    latest_color: Optional[Image] = None
    latest_depth: Optional[Image] = None
    last_infer_t: float = 0.0
    color_seq: int = 0
    last_used_seq: int = -1
    img_w: int = 640
    img_h: int = 480


@dataclass
class CamFallState:
    consec: int = 0
    clear_consec: int = 0
    fallen: bool = False
    last_fall_time: float = 0.0
    last_fall_ids: List[int] = field(default_factory=list)
    reason: str = 'init'
    det_n: int = 0


class PersonPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_perception')
        self.declare_parameter('model_path', _default_model_path())
        self.declare_parameter('object_thresh', 0.22)
        self.declare_parameter('trigger_consecutive_frames', 6)
        self.declare_parameter('clear_consecutive_frames', 6)
        self.declare_parameter('clear_timeout_seconds', 8.0)
        # Dual-cam topics / orientation
        self.declare_parameter('up_color_topic', '/camera/front_up/color/image_raw')
        self.declare_parameter('up_depth_topic', '/camera/front_up/depth/image_raw')
        self.declare_parameter('up_frame_id', 'camera_front_up_link')
        # Live front_up RGB is upright; do NOT rotate (rotate breaks YOLO → empty tracks).
        self.declare_parameter('up_rotate_180', False)
        self.declare_parameter('down_color_topic', '/camera/front_down/color/image_raw')
        self.declare_parameter('down_depth_topic', '/camera/front_down/depth/image_raw')
        self.declare_parameter('down_frame_id', 'camera_front_down_link')
        self.declare_parameter('down_rotate_180', True)
        self.declare_parameter('follow_cam', 'up')  # up | down | auto
        # Budget: shared NPU — fall-only alternates; follow prioritizes follow cam.
        self.declare_parameter('follow_infer_fps', 8.0)
        self.declare_parameter('fall_infer_fps', 3.5)
        self.declare_parameter('infer_fps', 8.0)  # legacy alias → follow_infer_fps
        self.declare_parameter('kp_conf_min', 0.25)
        self.declare_parameter('kp_min_visible', 3)
        self.declare_parameter('bbox_aspect_min', 0.55)
        self.declare_parameter('bbox_aspect_min_relaxed', 0.4)
        self.declare_parameter('torso_angle_min_deg', 30.0)
        self.declare_parameter('torso_compression_max', 0.40)
        self.declare_parameter('torso_inversion_margin_ratio', 0.0)
        self.declare_parameter('flat_aspect_min', 1.15)
        self.declare_parameter('lock_strategy', 'nearest')  # initial lock: nearest person
        self.declare_parameter('min_lock_conf', 0.28)
        self.declare_parameter('min_lock_area_frac', 0.02)
        self.declare_parameter('min_lock_width_ratio', 0.08)
        self.declare_parameter('hfov_deg', 70.0)  # for width→distance
        self.declare_parameter('coast_frames', 24)  # ~2–3 s coast before lost
        self.declare_parameter('relock_after_lost_s', 3.0)  # stick to ID until lost this long
        self.declare_parameter('assoc_iou_thresh', 0.03)
        self.declare_parameter('assoc_maha_thresh', 100.0)
        self.declare_parameter('assoc_center_frac', 0.50)

        self._follow_en = False
        self._follow_was_en = False
        self._fall_en = False
        self._npu_busy_until = 0.0
        self._infer_lock = threading.Lock()
        self._last_det_t = 0.0
        self._alt_cam = 'up'  # fall-only round-robin pointer
        self._need_lock = False
        self._follow_active_cam = 'up'
        self._last_follow_hit_t = 0.0  # last time follow cam saw ≥1 person
        self._lost_since = 0.0  # monotonic time when target first went lost; 0 = not lost

        self._cams: Dict[str, CamSlot] = {
            'up': CamSlot(
                cam_id='up',
                color_topic=str(self.get_parameter('up_color_topic').value),
                depth_topic=str(self.get_parameter('up_depth_topic').value),
                frame_id=str(self.get_parameter('up_frame_id').value),
                rotate_180=bool(self.get_parameter('up_rotate_180').value),
            ),
            'down': CamSlot(
                cam_id='down',
                color_topic=str(self.get_parameter('down_color_topic').value),
                depth_topic=str(self.get_parameter('down_depth_topic').value),
                frame_id=str(self.get_parameter('down_frame_id').value),
                rotate_180=bool(self.get_parameter('down_rotate_180').value),
            ),
        }
        self._fall_by_cam: Dict[str, CamFallState] = {
            'up': CamFallState(),
            'down': CamFallState(),
        }

        self._tracker = PersonLockTracker(
            lock_strategy=str(self.get_parameter('lock_strategy').value),
            coast_frames=int(self.get_parameter('coast_frames').value),
            iou_thresh=float(self.get_parameter('assoc_iou_thresh').value),
            maha_thresh=float(self.get_parameter('assoc_maha_thresh').value),
            center_frac=float(self.get_parameter('assoc_center_frac').value),
            min_lock_conf=float(self.get_parameter('min_lock_conf').value),
            min_lock_area_frac=float(self.get_parameter('min_lock_area_frac').value),
            min_lock_width_ratio=float(self.get_parameter('min_lock_width_ratio').value),
        )
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
        self._last_sched_log = 0.0

        model = str(self.get_parameter('model_path').value)
        self._backend = RknnPoseBackend(model, self.get_logger())

        self._tracks_pub = self.create_publisher(PersonTracks, '/xw/perception/tracks', 10)
        self._fall_pub = self.create_publisher(FallStatus, '/xw/perception/fall', 10)

        self.create_subscription(Bool, '/xw/follow/enable', self._on_follow, _LATCHED_QOS)
        self.create_subscription(Bool, '/xw/fall/enable', self._on_fall, _LATCHED_QOS)

        for cam_id, slot in self._cams.items():
            self.create_subscription(
                Image,
                slot.color_topic,
                lambda msg, c=cam_id: self._on_color(c, msg),
                _SENSOR_QOS,
            )
            self.create_subscription(
                Image,
                slot.depth_topic,
                lambda msg, c=cam_id: self._on_depth(c, msg),
                _SENSOR_QOS,
            )

        self.create_timer(1.0, self._tick_clear)
        self.create_timer(0.05, self._maybe_infer)

        status = 'RKNN ok' if self._backend.ok else 'RKNN UNAVAILABLE'
        follow_cam = str(self.get_parameter('follow_cam').value)
        self.get_logger().info(
            f'person perception ready — {status} model={model} '
            f'follow_cam={follow_cam} '
            f'up={self._cams["up"].color_topic} rot={self._cams["up"].rotate_180} '
            f'down={self._cams["down"].color_topic} rot={self._cams["down"].rotate_180} '
            f'follow_fps={float(self.get_parameter("follow_infer_fps").value):.1f} '
            f'fall_fps/cam={float(self.get_parameter("fall_infer_fps").value):.1f}'
        )

    def _follow_mode(self) -> str:
        raw = str(self.get_parameter('follow_cam').value).strip().lower()
        if raw in ('down', 'front_down', 'bottom'):
            return 'down'
        if raw in ('auto', 'both', 'any'):
            return 'auto'
        return 'up'

    def _follow_cam_id(self) -> str:
        mode = self._follow_mode()
        if mode == 'down':
            return 'down'
        if mode == 'auto':
            return self._follow_active_cam if self._follow_active_cam in ('up', 'down') else 'up'
        return 'up'

    def _follow_period(self) -> float:
        fps = float(self.get_parameter('follow_infer_fps').value)
        if fps <= 0.0:
            fps = float(self.get_parameter('infer_fps').value)
        return 1.0 / max(1.0, fps)

    def _fall_period(self) -> float:
        return 1.0 / max(1.0, float(self.get_parameter('fall_infer_fps').value))

    def _on_follow(self, msg: Bool) -> None:
        en = bool(msg.data)
        if en and not self._follow_was_en:
            self._need_lock = True
            self._tracker.reset()
            self._lost_since = 0.0
            mode = self._follow_mode()
            self._follow_active_cam = 'down' if mode == 'down' else 'up'
            self._last_follow_hit_t = 0.0
            self.get_logger().info(
                f'follow enable → arm target lock (mode={mode} cam={self._follow_active_cam} '
                f'strategy={self.get_parameter("lock_strategy").value})'
            )
        if not en and self._follow_was_en:
            self._tracker.reset()
            self._need_lock = False
            self._lost_since = 0.0
            self.get_logger().info('follow disable → clear target lock')
        self._follow_en = en
        self._follow_was_en = en

    def _on_fall(self, msg: Bool) -> None:
        en = bool(msg.data)
        if not en and self._fall_en:
            for st in self._fall_by_cam.values():
                st.consec = 0
                st.clear_consec = 0
                st.fallen = False
                st.last_fall_ids = []
                st.reason = 'disabled'
        self._fall_en = en

    def _on_depth(self, cam_id: str, msg: Image) -> None:
        slot = self._cams.get(cam_id)
        if slot is not None:
            slot.latest_depth = msg

    def _on_color(self, cam_id: str, msg: Image) -> None:
        if not (self._follow_en or self._fall_en):
            return
        slot = self._cams.get(cam_id)
        if slot is None:
            return
        slot.latest_color = msg
        slot.color_seq += 1
        slot.img_w = int(msg.width) or slot.img_w
        slot.img_h = int(msg.height) or slot.img_h

    def _active(self) -> bool:
        return bool(self._follow_en or self._fall_en)

    def _cam_due(self, cam_id: str, now: float) -> bool:
        slot = self._cams[cam_id]
        if slot.latest_color is None or slot.color_seq == slot.last_used_seq:
            return False
        follow_id = self._follow_cam_id()
        if self._follow_en and cam_id == follow_id:
            return (now - slot.last_infer_t) >= self._follow_period()
        if self._fall_en and not self._follow_en:
            return (now - slot.last_infer_t) >= self._fall_period()
        # While following: probe the other cam often enough to failover in auto mode.
        if self._fall_en and self._follow_en and cam_id != follow_id:
            if self._follow_mode() == 'auto':
                miss = (now - self._last_follow_hit_t) if self._last_follow_hit_t > 0 else 999.0
                period = 0.45 if miss > 0.8 else max(1.0, self._fall_period() * 3.0)
                return (now - slot.last_infer_t) >= period
            return (now - slot.last_infer_t) >= max(1.2, self._fall_period() * 4.0)
        if self._follow_en and self._follow_mode() == 'auto' and cam_id != follow_id:
            miss = (now - self._last_follow_hit_t) if self._last_follow_hit_t > 0 else 999.0
            return (now - slot.last_infer_t) >= (0.45 if miss > 0.8 else 1.0)
        return False

    def _pick_cam(self, now: float) -> Optional[str]:
        """Choose next cam for one NPU slot."""
        follow_id = self._follow_cam_id()
        # Body-follow wins the NPU: always prefer follow cam when due.
        if self._follow_en and self._cam_due(follow_id, now):
            return follow_id
        if self._follow_en and self._follow_mode() == 'auto':
            other = 'down' if follow_id == 'up' else 'up'
            if self._cam_due(other, now):
                return other
        if self._fall_en:
            if self._follow_en:
                other = 'down' if follow_id == 'up' else 'up'
                if self._cam_due(other, now):
                    return other
                return None
            order = [self._alt_cam, 'down' if self._alt_cam == 'up' else 'up']
            for cam_id in order:
                if self._cam_due(cam_id, now):
                    self._alt_cam = 'down' if cam_id == 'up' else 'up'
                    return cam_id
        return None

    def _maybe_infer(self) -> None:
        if not self._active():
            return
        now = time.monotonic()
        if now < self._npu_busy_until:
            return
        if not self._infer_lock.acquire(blocking=False):
            return
        try:
            self._maybe_infer_locked(now)
        finally:
            self._infer_lock.release()

    def _maybe_infer_locked(self, now: float) -> None:
        cam_id = self._pick_cam(now)
        if cam_id is None:
            return

        if not self._backend.ok:
            if now - self._last_unavailable_log > 10.0:
                self._last_unavailable_log = now
                self.get_logger().warn('perception active but RKNN model/runtime unavailable')
            self._publish_empty()
            return

        slot = self._cams[cam_id]
        msg = slot.latest_color
        if msg is None:
            return

        # Refresh rotate flags from params (hot-tunable)
        slot.rotate_180 = bool(
            self.get_parameter(
                'up_rotate_180' if cam_id == 'up' else 'down_rotate_180'
            ).value
        )

        rgb = _imgmsg_to_rgb(msg)
        if rgb is None:
            if now - self._last_decode_warn > 5.0:
                self._last_decode_warn = now
                self.get_logger().warn(f'{cam_id}: color decode failed')
            return

        if slot.rotate_180:
            rgb = np.ascontiguousarray(rgb[::-1, ::-1])

        thresh = float(self.get_parameter('object_thresh').value)
        t0 = time.monotonic()
        try:
            boxes = self._backend.infer(rgb, thresh)
        except Exception as exc:  # noqa: BLE001
            if now - self._last_infer_err > 2.0:
                self._last_infer_err = now
                self.get_logger().error(f'{cam_id}: infer failed: {exc}')
            return

        dt_infer = time.monotonic() - t0
        # Serialize NPU: block next infer for max(infer_dt, tiny floor)
        self._npu_busy_until = time.monotonic() + max(0.01, min(0.2, dt_infer * 0.1))
        slot.last_infer_t = now
        slot.last_used_seq = slot.color_seq

        self._handle_detections(cam_id, boxes)

    def _boxes_to_dets(self, cam_id: str, boxes) -> Tuple[List, List[Detection]]:
        slot = self._cams[cam_id]
        cx_img = slot.img_w * 0.5
        half_hfov = math.radians(float(self.get_parameter('hfov_deg').value) * 0.5)
        scored = []
        dets: List[Detection] = []
        for b in boxes:
            bw = max(1.0, float(b.xmax - b.xmin))
            bh = max(1.0, float(b.ymax - b.ymin))
            width_ratio = bw / max(1.0, float(slot.img_w))
            # Gen1-like range from torso/body width (~0.45 m assumed).
            ang = max(1e-3, width_ratio * half_hfov)
            dist_w = float(np.clip(0.225 / math.tan(ang), 0.40, 4.0))
            # Height cue (partial body fills frame → closer)
            dist_h = float(np.clip(2.0 * (float(slot.img_h) / bh), 0.40, 4.0))
            dist_bbox = 0.65 * dist_w + 0.35 * dist_h

            dist = 0.0
            if slot.latest_depth is not None:
                dist = _depth_median_m(
                    slot.latest_depth,
                    b.xmin,
                    b.ymin,
                    b.xmax,
                    b.ymax,
                    rotate_180=slot.rotate_180,
                )
            if dist <= 0.2:
                dist = dist_bbox
            else:
                # Depth often sparse on clothing; if it disagrees with size, trust size.
                if abs(dist - dist_bbox) > 0.7:
                    dist = 0.35 * dist + 0.65 * dist_bbox
                else:
                    dist = 0.55 * dist + 0.45 * dist_bbox

            area = max(1.0, bw * bh)
            bx = ((b.xmin + b.xmax) * 0.5 - cx_img) / max(1.0, cx_img)
            det = Detection(
                xmin=float(b.xmin),
                ymin=float(b.ymin),
                xmax=float(b.xmax),
                ymax=float(b.ymax),
                bearing=float(bx),
                distance=float(dist),
                confidence=float(b.score),
                area=float(area),
                width_ratio=float(width_ratio),
            )
            dets.append(det)
            scored.append((dist if dist > 0 else 1e6, -area, b, det))
        scored.sort(key=lambda t: (t[0], t[1]))
        return scored, dets

    def _handle_detections(self, cam_id: str, boxes) -> None:
        slot = self._cams[cam_id]
        stamp = self.get_clock().now().to_msg()
        scored, dets = self._boxes_to_dets(cam_id, boxes)

        do_follow = self._follow_en and cam_id == self._follow_cam_id()
        # Auto failover: if probing the other cam and it sees a person while active misses,
        # switch follow cam and re-lock on this view.
        if (
            self._follow_en
            and self._follow_mode() == 'auto'
            and cam_id != self._follow_active_cam
            and dets
        ):
            miss = (
                (time.monotonic() - self._last_follow_hit_t)
                if self._last_follow_hit_t > 0
                else 999.0
            )
            if miss > 0.7 or not self._tracker.locked:
                self._follow_active_cam = cam_id
                self._need_lock = True
                self._tracker.reset()
                do_follow = True
                self.get_logger().info(f'follow auto → switch to cam={cam_id} (miss={miss:.1f}s)')

        if do_follow and dets:
            self._last_follow_hit_t = time.monotonic()
            if self._follow_mode() == 'auto':
                self._follow_active_cam = cam_id

        target_det: Optional[Detection] = None
        coasting = False
        lost = False

        if do_follow:
            self._tracker.set_image_size(slot.img_w, slot.img_h)
            self._tracker.lock_strategy = str(self.get_parameter('lock_strategy').value)
            self._tracker.coast_frames = int(self.get_parameter('coast_frames').value)
            self._tracker.iou_thresh = float(self.get_parameter('assoc_iou_thresh').value)
            self._tracker.maha_thresh = float(self.get_parameter('assoc_maha_thresh').value)
            self._tracker.center_frac = float(self.get_parameter('assoc_center_frac').value)
            self._tracker.min_lock_conf = float(self.get_parameter('min_lock_conf').value)
            self._tracker.min_lock_area_frac = float(self.get_parameter('min_lock_area_frac').value)
            self._tracker.min_lock_width_ratio = float(
                self.get_parameter('min_lock_width_ratio').value
            )

            now_t = time.monotonic()
            dt = self._follow_period()
            if self._last_det_t > 0.0:
                dt = max(0.05, min(0.5, now_t - self._last_det_t))
            self._last_det_t = now_t
            relock_s = float(self.get_parameter('relock_after_lost_s').value)
            holding_lost = False

            if self._need_lock:
                if dets and self._tracker.lock_now(dets):
                    self._need_lock = False
                    self._lost_since = 0.0
                    target_det = self._tracker.last_det
                    self.get_logger().info(
                        f'target locked id={self._tracker.target_id} cam={cam_id} '
                        f'strategy={self._tracker.lock_strategy} dets={len(dets)} '
                        f'd={target_det.distance if target_det else -1:.2f}'
                    )
                elif not dets:
                    lost = True
            elif self._tracker.locked:
                target_det, coasting, lost = self._tracker.update(dets, dt=dt)
                if lost:
                    if self._lost_since <= 0.0:
                        self._lost_since = now_t
                        self.get_logger().warn(
                            f'target lost id={self._tracker.target_id} — hold {relock_s:.1f}s before re-lock'
                        )
                    held = now_t - self._lost_since
                    if held < relock_s:
                        target_det = self._tracker.last_det
                        coasting = True
                        lost = False
                        holding_lost = True
                    elif dets and self._tracker.lock_now(dets):
                        self._lost_since = 0.0
                        lost = False
                        coasting = False
                        target_det = self._tracker.last_det
                        self.get_logger().info(
                            f'target re-locked id={self._tracker.target_id} cam={cam_id} '
                            f'after {held:.1f}s dets={len(dets)}'
                        )
                else:
                    self._lost_since = 0.0
            else:
                if self._lost_since <= 0.0:
                    self._lost_since = now_t
                held = now_t - self._lost_since
                if held >= relock_s and dets and self._tracker.lock_now(dets):
                    self._lost_since = 0.0
                    target_det = self._tracker.last_det
                    self.get_logger().info(
                        f'target locked id={self._tracker.target_id} cam={cam_id} '
                        f'(fresh after {held:.1f}s lost) dets={len(dets)}'
                    )
                else:
                    lost = True
                    if self._tracker.last_det is not None and held < relock_s:
                        target_det = self._tracker.last_det
                        coasting = True
                        lost = False
                        holding_lost = True

            tracks = PersonTracks()
            tracks.stamp = stamp
            tracks.frame_id = slot.frame_id
            target_bbox = target_det.as_bbox() if target_det is not None else None
            for i, (_k, _a, b, det) in enumerate(scored):
                is_tgt = False
                tid = i + 2
                if target_bbox is not None and self._tracker.locked and not lost:
                    if bbox_iou(det.as_bbox(), target_bbox) >= 0.2 or (
                        target_det is not None
                        and abs(det.cx - target_det.cx) < 8
                        and abs(det.cy - target_det.cy) < 8
                    ):
                        is_tgt = True
                        tid = self._tracker.target_id
                pt = PersonTrack()
                pt.track_id = tid
                pt.x = float(det.bearing)
                pt.y = float(det.width_ratio)  # gen1-style size cue for follow
                pt.z = float(det.distance)
                pt.distance = float(det.distance)
                pt.confidence = float(det.confidence)
                pt.is_primary = i == 0
                pt.is_target = bool(is_tgt)
                tracks.tracks.append(pt)

            if (
                coasting
                and target_det is not None
                and not any(t.is_target for t in tracks.tracks)
            ):
                pt = PersonTrack()
                pt.track_id = self._tracker.target_id
                pt.x = float(target_det.bearing)
                pt.y = float(target_det.width_ratio)
                pt.z = float(target_det.distance)
                pt.distance = float(target_det.distance)
                # Hold-lost: keep follow session on same target (≥3s). Brief coast: low conf.
                pt.confidence = 0.36 if holding_lost else 0.05
                pt.is_primary = False
                pt.is_target = True
                tracks.tracks.append(pt)

            self._tracks_pub.publish(tracks)
            now = time.monotonic()
            if now - self._last_debug_log > 3.0:
                self._last_debug_log = now
                self.get_logger().info(
                    f'follow dbg cam={cam_id} det={len(scored)} locked={self._tracker.locked} '
                    f'coast={coasting} lost={lost or self._tracker.lost} '
                    f'tid={self._tracker.target_id} '
                    f'd={target_det.distance if target_det else -1.0:.2f} '
                    f'b={target_det.bearing if target_det else 0.0:.2f}'
                )

        if self._fall_en:
            fall_ids: List[int] = []
            geom_reasons: List[str] = []
            for i, (_k, _a, b, _det) in enumerate(scored):
                ok, reason = passes_fall_geometry(
                    b.keypoints, b.xmin, b.ymin, b.xmax, b.ymax, self._geom
                )
                geom_reasons.append(reason)
                if ok:
                    fall_ids.append(i + 2)
            st = self._fall_by_cam[cam_id]
            st.det_n = len(scored)
            if not scored:
                st.reason = 'no_det'
            elif geom_reasons:
                st.reason = geom_reasons[0]
            self._update_fall_cam(cam_id, fall_ids)
            self._publish_fall_or(stamp)

    def _update_fall_cam(self, cam_id: str, fall_ids: List[int]) -> None:
        st = self._fall_by_cam[cam_id]
        need = max(1, int(self.get_parameter('trigger_consecutive_frames').value))
        clear_need = max(1, int(self.get_parameter('clear_consecutive_frames').value))
        if fall_ids:
            st.clear_consec = 0
            if st.last_fall_ids and set(fall_ids).isdisjoint(st.last_fall_ids):
                st.consec = 0
            st.consec += 1
            st.last_fall_ids = list(fall_ids)
            if st.consec >= need:
                st.last_fall_time = time.time()
                if not st.fallen:
                    st.fallen = True
                    self.get_logger().warn(
                        f'fall confirmed cam={cam_id} after {st.consec} frames ids={fall_ids}'
                    )
        else:
            st.consec = 0
            st.last_fall_ids = []
            if st.fallen:
                st.clear_consec += 1
                if st.clear_consec >= clear_need:
                    st.fallen = False
                    st.clear_consec = 0
                    self.get_logger().info(
                        f'fall cleared cam={cam_id} after {clear_need} non-fall frames '
                        f'(reason={st.reason})'
                    )
            else:
                st.clear_consec = 0

    def _publish_fall_or(self, stamp) -> None:
        """OR merge: either cam fallen → output fallen."""
        up = self._fall_by_cam['up']
        down = self._fall_by_cam['down']
        fallen = bool(up.fallen or down.fallen)
        consec = max(up.consec, down.consec)
        need = max(1, int(self.get_parameter('trigger_consecutive_frames').value))
        clear_need = max(1, int(self.get_parameter('clear_consecutive_frames').value))
        clear_c = max(up.clear_consec, down.clear_consec)
        reasons = []
        if up.fallen or up.consec > 0:
            reasons.append(f'up:{up.reason}')
        if down.fallen or down.consec > 0:
            reasons.append(f'down:{down.reason}')
        reason = ','.join(reasons) if reasons else (up.reason or down.reason or 'none')

        msg = FallStatus()
        msg.stamp = stamp
        msg.is_fallen = fallen
        msg.confidence = 0.9 if fallen else (0.4 if consec > 0 else 0.1)
        msg.source = 'yolov8n-pose+geometry|dual-or'
        if fallen:
            msg.detail = (
                f'fallen cams={("up" if up.fallen else "")}'
                f'{("+" if up.fallen and down.fallen else "")}'
                f'{("down" if down.fallen else "")} '
                f'clear={clear_c}/{clear_need} reason={reason}'
            )
        else:
            msg.detail = (
                f'consec={consec}/{need} '
                f'det_up={up.det_n} det_down={down.det_n} reason={reason}'
            )
        self._fall_pub.publish(msg)

        now = time.monotonic()
        if now - self._last_debug_log > 3.0:
            self._last_debug_log = now
            self.get_logger().info(
                f'fall dbg OR fallen={fallen} '
                f'up(c={up.consec},f={up.fallen},d={up.det_n}) '
                f'down(c={down.consec},f={down.fallen},d={down.det_n}) '
                f'reason={reason}'
            )

    def _tick_clear(self) -> None:
        clear_s = float(self.get_parameter('clear_timeout_seconds').value)
        now = time.time()
        changed = False
        for cam_id, st in self._fall_by_cam.items():
            if st.fallen and st.last_fall_time > 0 and (now - st.last_fall_time) >= clear_s:
                st.fallen = False
                st.consec = 0
                st.clear_consec = 0
                changed = True
                self.get_logger().info(f'fall cam={cam_id} cleared by timeout ({clear_s:.0f}s)')
        if changed and self._fall_en:
            self._publish_fall_or(self.get_clock().now().to_msg())

    def _publish_empty(self) -> None:
        stamp = self.get_clock().now().to_msg()
        if self._follow_en:
            t = PersonTracks()
            t.stamp = stamp
            t.frame_id = self._cams[self._follow_cam_id()].frame_id
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
    # Image callbacks must not stall while RKNN runs, or RGB/tracks drop to ~1 Hz
    # and follow's fresh_timeout brakes forever.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
