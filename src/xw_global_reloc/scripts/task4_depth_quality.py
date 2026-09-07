#!/usr/bin/env python3
"""Task4 — static depth quality + ORB depth coverage (few frames)."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from xw_global_reloc.orb_utils import extract_orb, make_orb
from xw_global_reloc.pairing import ImageRingBuffer, pair_nearest


_SENSOR = QoSProfile(
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


def pct(xs, p):
    if not xs:
        return float('nan')
    ys = sorted(xs)
    k = (len(ys) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return float(ys[int(k)])
    return float(ys[f] * (c - k) + ys[c] * (k - f))


def analyze(bgr, depth_u16, scale=0.001):
    h, w = depth_u16.shape[:2]
    valid_full = depth_u16 > 0
    full_ratio = float(np.count_nonzero(valid_full)) / float(depth_u16.size)
    # center ROI 40%
    y0, y1 = int(h * 0.3), int(h * 0.7)
    x0, x1 = int(w * 0.3), int(w * 0.7)
    roi = depth_u16[y0:y1, x0:x1]
    roi_valid = roi > 0
    roi_ratio = float(np.count_nonzero(roi_valid)) / float(roi.size)
    vals = depth_u16[valid_full].astype(np.float64) * scale
    orb = extract_orb(bgr, make_orb(1000))
    with_d = 0
    for kp in orb.keypoints:
        u, v = int(round(kp.pt[0])), int(round(kp.pt[1]))
        if 0 <= u < w and 0 <= v < h and depth_u16[v, u] > 0:
            with_d += 1
    cov = float(with_d) / float(max(orb.n_features, 1))
    return {
        'full_frame_valid_ratio': full_ratio,
        'center_roi_valid_ratio': roi_ratio,
        'depth_median_m': float(np.median(vals)) if vals.size else None,
        'depth_p10_m': float(np.percentile(vals, 10)) if vals.size else None,
        'depth_p50_m': float(np.percentile(vals, 50)) if vals.size else None,
        'depth_p90_m': float(np.percentile(vals, 90)) if vals.size else None,
        'invalid_ratio': 1.0 - full_ratio,
        'orb_total': orb.n_features,
        'orb_with_valid_depth': with_d,
        'orb_depth_coverage_ratio': cov,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', default='static_unknown')
    ap.add_argument('--expected-m', type=float, default=0.0)
    ap.add_argument('--frames', type=int, default=5)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rclpy.init()
    n = Node('depth_quality_probe')
    bridge = CvBridge()
    rgb_buf = ImageRingBuffer(20)
    depth_buf = ImageRingBuffer(20)
    n.create_subscription(Image, '/camera/front_up/color/image_raw', lambda m: rgb_buf.push(m), _SENSOR)
    n.create_subscription(Image, '/camera/front_up/depth/image_raw', lambda m: depth_buf.push(m), _SENSOR)
    req = n.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
    req.publish(Bool(data=True))

    frames = []
    seen = set()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 20 and len(frames) < args.frames:
        rclpy.spin_once(n, timeout_sec=0.05)
        pair = pair_nearest(rgb_buf, depth_buf, prefer='rgb')
        if pair is None or pair.pair_dt_sec > 0.05:
            continue
        key = round(pair.rgb_t, 3)
        if key in seen:
            continue
        seen.add(key)
        bgr = bridge.imgmsg_to_cv2(pair.rgb, 'bgr8')
        depth = bridge.imgmsg_to_cv2(pair.depth, 'passthrough')
        if depth.dtype != np.uint16:
            continue
        stats = analyze(bgr, depth)
        stats['pair_dt_ms'] = pair.pair_dt_sec * 1000.0
        frames.append(stats)

    req.publish(Bool(data=False))
    # aggregate
    def mean_key(k):
        xs = [f[k] for f in frames if f.get(k) is not None]
        return float(sum(xs) / len(xs)) if xs else None

    report = {
        'label': args.label,
        'expected_m': args.expected_m,
        'n_frames': len(frames),
        'frames': frames,
        'agg': {
            'full_frame_valid_ratio_mean': mean_key('full_frame_valid_ratio'),
            'center_roi_valid_ratio_mean': mean_key('center_roi_valid_ratio'),
            'orb_depth_coverage_ratio_mean': mean_key('orb_depth_coverage_ratio'),
            'orb_total_mean': mean_key('orb_total'),
            'depth_median_m_mean': mean_key('depth_median_m'),
            'pair_dt_ms_mean': mean_key('pair_dt_ms'),
        },
    }
    out = Path(args.out) if args.out else Path(
        f'/ros2_ws/bench/phase2a_poc_v1_2026-09-07/task4_depth_{args.label}.json'
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
