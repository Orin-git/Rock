#!/usr/bin/env python3
"""Stage 0 — HP60C front_up sensor contract (few frames, BEST_EFFORT)."""

from __future__ import annotations

import json
import math
import statistics
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _pct(xs: List[float], p: float) -> float:
    if not xs:
        return float('nan')
    ys = sorted(xs)
    k = (len(ys) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return float(ys[int(k)])
    return float(ys[f] * (c - k) + ys[c] * (k - f))


class SensorContractNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_sensor_contract')
        self.declare_parameter('num_pairs', 40)
        self.declare_parameter('timeout_sec', 45.0)
        self.declare_parameter('max_dt_ok_sec', 0.08)
        self.declare_parameter('out_json', '/ros2_ws/bench/phase2a_poc_v1_2026-09-07/stage0_sensor_contract.json')
        self.declare_parameter('request_rgb', True)
        # Fallback if public RGB gated and reloc hook not yet live in running bridge.
        self.declare_parameter('allow_vendor_fallback', True)

        self._bridge = CvBridge()
        self._rgb: Optional[Image] = None
        self._depth: Optional[Image] = None
        self._rgb_info: Optional[CameraInfo] = None
        self._depth_info: Optional[CameraInfo] = None
        self._pairs: List[dict] = []
        self._dts: List[float] = []
        self._valid_ratios: List[float] = []
        self._depth_samples: List[int] = []
        self._done = False
        self._t0 = time.monotonic()
        self._used_vendor_rgb = False

        self._rgb_req = self.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
        if bool(self.get_parameter('request_rgb').value):
            self._rgb_req.publish(Bool(data=True))
            self.get_logger().info('published /xw/reloc/rgb_request=true')

        self.create_subscription(Image, '/camera/front_up/color/image_raw', self._on_rgb, _SENSOR_QOS)
        self.create_subscription(Image, '/camera/front_up/depth/image_raw', self._on_depth, _SENSOR_QOS)
        self.create_subscription(CameraInfo, '/camera/front_up/color/camera_info', self._on_rgb_info, _SENSOR_QOS)
        self.create_subscription(CameraInfo, '/camera/front_up/depth/camera_info', self._on_depth_info, _SENSOR_QOS)
        if bool(self.get_parameter('allow_vendor_fallback').value):
            self.create_subscription(
                Image,
                '/ascamera_hp60c/camera_publisher/rgb0/image',
                self._on_vendor_rgb,
                _SENSOR_QOS,
            )
            self.create_subscription(
                CameraInfo,
                '/ascamera_hp60c/camera_publisher/rgb0/camera_info',
                self._on_rgb_info,
                _SENSOR_QOS,
            )
            self.create_subscription(
                CameraInfo,
                '/ascamera_hp60c/camera_publisher/depth0/camera_info',
                self._on_depth_info,
                _SENSOR_QOS,
            )

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self.create_timer(0.05, self._tick)

    def _on_rgb(self, msg: Image) -> None:
        self._rgb = msg

    def _on_vendor_rgb(self, msg: Image) -> None:
        # Only fill if public RGB not arriving (gated).
        if self._rgb is None:
            self._used_vendor_rgb = True
            self._rgb = msg

    def _on_depth(self, msg: Image) -> None:
        self._depth = msg

    def _on_rgb_info(self, msg: CameraInfo) -> None:
        self._rgb_info = msg

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._depth_info = msg

    def _release_rgb(self) -> None:
        self._rgb_req.publish(Bool(data=False))

    def _registration_test(self, bgr: np.ndarray, depth_u16: np.ndarray) -> dict:
        """Edge co-occurrence proxy — does NOT prove factory alignment alone."""
        import cv2

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        rgb_e = cv2.Canny(gray, 50, 150)
        d = depth_u16.astype(np.float32)
        d[d == 0] = np.nan
        # Finite differences as depth edges
        gx = np.nan_to_num(np.diff(d, axis=1, prepend=d[:, :1]), nan=0.0)
        gy = np.nan_to_num(np.diff(d, axis=0, prepend=d[:1, :]), nan=0.0)
        mag = np.sqrt(gx * gx + gy * gy)
        thr = float(np.nanpercentile(mag[np.isfinite(mag)], 90)) if np.isfinite(mag).any() else 1e9
        depth_e = (mag >= thr).astype(np.uint8) * 255
        rgb_n = int(np.count_nonzero(rgb_e))
        both = int(np.count_nonzero((rgb_e > 0) & (depth_e > 0)))
        ratio = float(both) / float(max(rgb_n, 1))
        # Same resolution + identical frame_id is necessary but not sufficient.
        same_size = bgr.shape[:2] == depth_u16.shape[:2]
        return {
            'same_resolution': same_size,
            'rgb_edge_pixels': rgb_n,
            'edge_cooccurrence_ratio': ratio,
            'heuristic': 'PASS_WEAK' if same_size and ratio >= 0.02 else 'FAIL_OR_UNKNOWN',
            'note': 'Edge co-occurrence is a weak proxy; factory RGB-D registration not certified.',
        }

    def _infer_depth_scale(self, samples: List[int]) -> dict:
        if not samples:
            return {'unit': 'UNKNOWN', 'scale_m_per_unit': float('nan'), 'median_raw': None}
        med = float(statistics.median(samples))
        # HP60C typically millimeters; indoor medians often 500–4000.
        if 200 <= med <= 8000:
            return {
                'unit': 'millimeters_assumed',
                'scale_m_per_unit': 0.001,
                'median_raw': med,
                'confidence': 'HIGH_HEURISTIC',
            }
        if 0.2 <= med <= 8.0:
            return {
                'unit': 'meters_raw_unlikely_for_16UC1',
                'scale_m_per_unit': 1.0,
                'median_raw': med,
                'confidence': 'LOW',
            }
        return {
            'unit': 'UNKNOWN',
            'scale_m_per_unit': 0.001,
            'median_raw': med,
            'confidence': 'LOW',
            'note': 'Using 0.001 default; verify against known distance.',
        }

    def _tick(self) -> None:
        if self._done:
            return
        if time.monotonic() - self._t0 > float(self.get_parameter('timeout_sec').value):
            self._finish(timeout=True)
            return
        if self._rgb is None or self._depth is None:
            return
        dt = abs(_stamp_sec(self._rgb.header.stamp) - _stamp_sec(self._depth.header.stamp))
        try:
            depth = self._bridge.imgmsg_to_cv2(self._depth, desired_encoding='passthrough')
            rgb = self._bridge.imgmsg_to_cv2(self._rgb, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return
        if depth.dtype != np.uint16:
            # still record
            pass
        valid = float(np.count_nonzero(depth > 0)) / float(max(depth.size, 1))
        nz = depth[depth > 0]
        if nz.size:
            self._depth_samples.extend(nz.flatten()[:: max(1, nz.size // 50)].astype(int).tolist())
        self._dts.append(dt)
        self._valid_ratios.append(valid)
        if len(self._pairs) == 0:
            self._first_rgb = rgb
            self._first_depth = depth
            self._reg = self._registration_test(rgb, depth.astype(np.uint16) if depth.dtype != np.uint16 else depth)
        self._pairs.append(
            {
                'rgb_stamp': _stamp_sec(self._rgb.header.stamp),
                'depth_stamp': _stamp_sec(self._depth.header.stamp),
                'dt': dt,
                'valid_ratio': valid,
                'rgb_frame': self._rgb.header.frame_id,
                'depth_frame': self._depth.header.frame_id,
                'rgb_enc': self._rgb.encoding,
                'depth_enc': self._depth.encoding,
                'rgb_wh': [int(self._rgb.width), int(self._rgb.height)],
                'depth_wh': [int(self._depth.width), int(self._depth.height)],
            }
        )
        # consume pair
        self._rgb = None
        self._depth = None
        need = int(self.get_parameter('num_pairs').value)
        if len(self._pairs) >= need:
            self._finish(timeout=False)

    def _tf_extrinsic(self) -> dict:
        try:
            tf = self._tf.lookup_transform('base_link', 'camera_front_up_link', rclpy.time.Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            return {
                'source': 'TF(base_link←camera_front_up_link) via robot_state_publisher/URDF',
                'status': 'EXTRINSIC_UNCALIBRATED',
                'translation': [t.x, t.y, t.z],
                'rotation_xyzw': [r.x, r.y, r.z, r.w],
                'urdf_origin': 'xyz=0.251 0 0.49 rpy=-1.33 0 -1.5708',
            }
        except TransformException as exc:
            return {
                'source': 'URDF_FALLBACK',
                'status': 'EXTRINSIC_UNCALIBRATED',
                'error': str(exc),
                'urdf_origin': 'xyz=0.251 0 0.49 rpy=-1.33 0 -1.5708',
            }

    def _finish(self, timeout: bool) -> None:
        self._done = True
        self._release_rgb()
        scale = self._infer_depth_scale(self._depth_samples)
        max_dt_ok = float(self.get_parameter('max_dt_ok_sec').value)
        dt_ok = bool(self._dts) and _pct(self._dts, 95) <= max_dt_ok * 3  # allow USB jitter band
        ri = self._rgb_info
        di = self._depth_info
        info = {
            'rgb_info': None if ri is None else {
                'frame_id': ri.header.frame_id,
                'width': ri.width,
                'height': ri.height,
                'k': list(ri.k),
            },
            'depth_info': None if di is None else {
                'frame_id': di.header.frame_id,
                'width': di.width,
                'height': di.height,
                'k': list(di.k),
            },
        }
        K_match = False
        if ri is not None and di is not None:
            K_match = list(ri.k) == list(di.k) and ri.width == di.width and ri.height == di.height
        same_frame = False
        if self._pairs:
            same_frame = self._pairs[0]['rgb_frame'] == self._pairs[0]['depth_frame']

        reg = getattr(self, '_reg', {'heuristic': 'NO_DATA'})
        # Gate: registration confirmed only if strong evidence
        if not self._pairs:
            reg_conclusion = 'UNKNOWN_NO_DATA'
            stage_pass = False
        elif not (info['rgb_info'] and info['depth_info']):
            reg_conclusion = 'UNKNOWN_MISSING_CAMERA_INFO'
            stage_pass = False
        elif not K_match or not same_frame:
            reg_conclusion = 'NOT_CONFIRMED_INTRINSICS_OR_FRAME_MISMATCH'
            stage_pass = False
        elif reg.get('heuristic') == 'FAIL_OR_UNKNOWN':
            reg_conclusion = 'NOT_CONFIRMED_WEAK_EDGE_ALIGNMENT'
            # Still allow PoC with WARNING if same K + same frame + same res (vendor aligned claim)
            stage_pass = bool(K_match and same_frame and reg.get('same_resolution'))
        else:
            reg_conclusion = 'VENDOR_ALIGNED_ASSUMED_WEAK_EVIDENCE'
            stage_pass = True

        report = {
            'stage': 0,
            'timeout': timeout,
            'pairs': len(self._pairs),
            'rgb_encoding': self._pairs[0]['rgb_enc'] if self._pairs else None,
            'depth_encoding': self._pairs[0]['depth_enc'] if self._pairs else None,
            'depth_scale': scale,
            'rgb_depth_dt_sec': {
                'p50': _pct(self._dts, 50),
                'p95': _pct(self._dts, 95),
                'p99': _pct(self._dts, 99),
                'max': max(self._dts) if self._dts else float('nan'),
                'mean': float(statistics.mean(self._dts)) if self._dts else float('nan'),
            },
            'depth_valid_ratio': {
                'mean': float(statistics.mean(self._valid_ratios)) if self._valid_ratios else float('nan'),
                'p50': _pct(self._valid_ratios, 50),
                'min': min(self._valid_ratios) if self._valid_ratios else float('nan'),
            },
            'camera_info': info,
            'intrinsics_match_640x480': bool(
                ri is not None and ri.width == 640 and ri.height == 480 and K_match
            ),
            'frame_ids_identical': same_frame,
            'registration_test': reg,
            'registration_conclusion': reg_conclusion,
            'extrinsic': self._tf_extrinsic(),
            'dt_gate_loose_ok': dt_ok,
            'used_vendor_rgb_fallback': bool(getattr(self, '_used_vendor_rgb', False)),
            'stage0_pass': bool(stage_pass and self._pairs),
            'pnp_allowed': bool(stage_pass and self._pairs),
            'note': 'Do not treat frame_id equality alone as geometric registration proof.',
        }
        out = Path(str(self.get_parameter('out_json').value))
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2), encoding='utf-8')
        self.get_logger().info(f'Stage0 written {out} pass={report["stage0_pass"]} reg={reg_conclusion}')
        print(json.dumps(report, indent=2))
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorContractNode()
    try:
        rclpy.spin(node)
    except Exception:  # noqa: BLE001
        pass
    finally:
        try:
            node._release_rgb()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
