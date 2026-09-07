#!/usr/bin/env python3
"""Standalone Stage 0 sensor contract probe (minimal observer load)."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def pct(xs, p):
    if not xs:
        return float('nan')
    ys = sorted(xs)
    k = (len(ys) - 1) * p / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return float(ys[int(k)])
    return float(ys[f] * (c - k) + ys[c] * (k - f))


def main() -> None:
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )
    rclpy.init()
    n = Node('s0_probe')
    state = {'rgb': None, 'depth': None, 'ri': None, 'di': None}
    n.create_subscription(
        Image, '/ascamera_hp60c/camera_publisher/rgb0/image',
        lambda m: state.update(rgb=m), qos,
    )
    n.create_subscription(
        Image, '/camera/front_up/depth/image_raw',
        lambda m: state.update(depth=m), qos,
    )
    n.create_subscription(
        CameraInfo, '/ascamera_hp60c/camera_publisher/rgb0/camera_info',
        lambda m: state.update(ri=m), qos,
    )
    n.create_subscription(
        CameraInfo, '/camera/front_up/depth/camera_info',
        lambda m: state.update(di=m), qos,
    )
    # also vendor depth info if public missing
    n.create_subscription(
        CameraInfo, '/ascamera_hp60c/camera_publisher/depth0/camera_info',
        lambda m: state.update(di=m) if state['di'] is None else None, qos,
    )

    bridge = CvBridge()
    pairs = []
    dts = []
    valids = []
    samples = []
    reg = None
    t0 = time.time()
    while time.time() - t0 < 30 and len(pairs) < 30:
        rclpy.spin_once(n, timeout_sec=0.05)
        if state['rgb'] is None or state['depth'] is None:
            continue
        rs = state['rgb'].header.stamp.sec + state['rgb'].header.stamp.nanosec * 1e-9
        ds = state['depth'].header.stamp.sec + state['depth'].header.stamp.nanosec * 1e-9
        dt = abs(rs - ds)
        depth = bridge.imgmsg_to_cv2(state['depth'], 'passthrough')
        rgb = bridge.imgmsg_to_cv2(state['rgb'], 'bgr8')
        valid = float(np.count_nonzero(depth > 0)) / float(depth.size)
        nz = depth[depth > 0]
        if nz.size:
            samples.extend(nz.flatten()[:: max(1, nz.size // 40)].astype(int).tolist())
        dts.append(dt)
        valids.append(valid)
        pairs.append(
            {
                'dt': dt,
                'valid': valid,
                'rgb_enc': state['rgb'].encoding,
                'depth_enc': state['depth'].encoding,
                'rgb_f': state['rgb'].header.frame_id,
                'depth_f': state['depth'].header.frame_id,
                'rgb_wh': [state['rgb'].width, state['rgb'].height],
                'depth_wh': [state['depth'].width, state['depth'].height],
            }
        )
        if len(pairs) == 1:
            import cv2

            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
            rgb_e = cv2.Canny(gray, 50, 150)
            d = depth.astype(np.float32)
            d[d == 0] = np.nan
            gx = np.nan_to_num(np.diff(d, axis=1, prepend=d[:, :1]), nan=0.0)
            gy = np.nan_to_num(np.diff(d, axis=0, prepend=d[:1, :]), nan=0.0)
            mag = np.sqrt(gx * gx + gy * gy)
            finite = mag[np.isfinite(mag)]
            thr = float(np.nanpercentile(finite, 90)) if finite.size else 1e9
            depth_e = (mag >= thr).astype(np.uint8) * 255
            co = float(np.count_nonzero((rgb_e > 0) & (depth_e > 0))) / max(
                int(np.count_nonzero(rgb_e)), 1
            )
            reg = {
                'same_resolution': rgb.shape[:2] == depth.shape[:2],
                'edge_cooccurrence_ratio': co,
                'heuristic': 'PASS_WEAK' if co >= 0.02 else 'FAIL_OR_UNKNOWN',
            }
        state['rgb'] = None
        state['depth'] = None

    ri, di = state['ri'], state['di']
    K_match = bool(ri and di and list(ri.k) == list(di.k) and ri.width == di.width)
    med = float(np.median(samples)) if samples else float('nan')
    report = {
        'stage': 0,
        'pairs': len(pairs),
        'rgb_encoding': pairs[0]['rgb_enc'] if pairs else None,
        'depth_encoding': pairs[0]['depth_enc'] if pairs else None,
        'frame_ids': {
            'rgb': pairs[0]['rgb_f'] if pairs else None,
            'depth': pairs[0]['depth_f'] if pairs else None,
            'identical': (pairs[0]['rgb_f'] == pairs[0]['depth_f']) if pairs else None,
        },
        'rgb_depth_dt_sec': {
            'p50': pct(dts, 50),
            'p95': pct(dts, 95),
            'p99': pct(dts, 99),
            'max': max(dts) if dts else None,
            'mean': float(np.mean(dts)) if dts else None,
        },
        'depth_valid_ratio': {
            'mean': float(np.mean(valids)) if valids else None,
            'p50': pct(valids, 50),
            'min': min(valids) if valids else None,
        },
        'depth_scale': {
            'median_raw': med,
            'unit': 'millimeters_assumed' if 200 <= med <= 8000 else 'UNKNOWN',
            'scale_m_per_unit': 0.001,
            'confidence': 'HIGH_HEURISTIC' if 200 <= med <= 8000 else 'LOW',
        },
        'camera_info': {
            'rgb_k': list(ri.k) if ri else None,
            'depth_k': list(di.k) if di else None,
            'wh': [ri.width, ri.height] if ri else None,
            'K_match': K_match,
            'matches_640x480': bool(ri and ri.width == 640 and ri.height == 480),
        },
        'registration_test': reg,
        'extrinsic': {
            'source': 'URDF',
            'status': 'EXTRINSIC_UNCALIBRATED',
            'urdf_origin': 'xyz=0.251 0 0.49 rpy=-1.33 0 -1.5708',
        },
        'used_vendor_rgb_fallback': True,
        'note': 'Do not treat frame_id equality alone as geometric registration proof.',
    }
    if not pairs:
        report['registration_conclusion'] = 'UNKNOWN_NO_DATA'
        report['stage0_pass'] = False
        report['pnp_allowed'] = False
    elif not (ri and di):
        report['registration_conclusion'] = 'UNKNOWN_MISSING_CAMERA_INFO'
        report['stage0_pass'] = False
        report['pnp_allowed'] = False
    elif not K_match or not report['frame_ids']['identical']:
        report['registration_conclusion'] = 'NOT_CONFIRMED_INTRINSICS_OR_FRAME_MISMATCH'
        report['stage0_pass'] = False
        report['pnp_allowed'] = False
    else:
        report['registration_conclusion'] = 'VENDOR_ALIGNED_ASSUMED_WEAK_EVIDENCE'
        report['stage0_pass'] = True
        report['pnp_allowed'] = True

    out = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07/stage0_sensor_contract.json')
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
