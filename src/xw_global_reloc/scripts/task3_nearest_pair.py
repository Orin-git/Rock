#!/usr/bin/env python3
"""Task3 — nearest-timestamp RGB/Depth pairing stats (40–50 pairs)."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from xw_global_reloc.pairing import ImageRingBuffer, pair_nearest


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


def main() -> None:
    rclpy.init()
    n = Node('nearest_pair_probe')
    rgb_buf = ImageRingBuffer(40)
    depth_buf = ImageRingBuffer(40)
    n.create_subscription(Image, '/camera/front_up/color/image_raw', lambda m: rgb_buf.push(m), _SENSOR)
    n.create_subscription(Image, '/camera/front_up/depth/image_raw', lambda m: depth_buf.push(m), _SENSOR)
    req = n.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
    req.publish(Bool(data=True))
    dts = []
    naive_dts = []
    last_rgb = {'m': None}
    last_depth = {'m': None}

    def on_rgb(m):
        last_rgb['m'] = m
        rgb_buf.push(m)

    def on_depth(m):
        last_depth['m'] = m
        depth_buf.push(m)

    # rebind with naive trackers
    n.destroy_subscription  # noqa: B018 — keep lint quiet
    # recreate cleanly
    n2 = Node('nearest_pair_probe2')
    rgb_buf = ImageRingBuffer(40)
    depth_buf = ImageRingBuffer(40)
    n2.create_subscription(Image, '/camera/front_up/color/image_raw', on_rgb, _SENSOR)
    n2.create_subscription(Image, '/camera/front_up/depth/image_raw', on_depth, _SENSOR)
    req2 = n2.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
    req2.publish(Bool(data=True))

    t0 = time.monotonic()
    while time.monotonic() - t0 < 25 and len(dts) < 50:
        rclpy.spin_once(n2, timeout_sec=0.05)
        pair = pair_nearest(rgb_buf, depth_buf, prefer='rgb')
        if pair is None:
            continue
        # de-dup by rgb stamp
        if dts and abs(pair.pair_dt_sec - dts[-1]) < 1e-9 and len(dts) > 0:
            # still allow; use stamp uniqueness
            pass
        key = round(pair.rgb_t, 3)
        if any(abs(key - round(x, 3)) < 1e-9 for x in getattr(main, '_keys', [])):
            continue
        if not hasattr(main, '_keys'):
            main._keys = []
        main._keys.append(key)
        dts.append(pair.pair_dt_sec * 1000.0)
        if last_rgb['m'] is not None and last_depth['m'] is not None:
            rs = last_rgb['m'].header.stamp.sec + last_rgb['m'].header.stamp.nanosec * 1e-9
            ds = last_depth['m'].header.stamp.sec + last_depth['m'].header.stamp.nanosec * 1e-9
            naive_dts.append(abs(rs - ds) * 1000.0)

    req2.publish(Bool(data=False))
    report = {
        'n_pairs': len(dts),
        'nearest_pair_dt_ms': {
            'p50': pct(dts, 50),
            'p95': pct(dts, 95),
            'p99': pct(dts, 99),
            'max': max(dts) if dts else None,
            'mean': sum(dts) / len(dts) if dts else None,
        },
        'naive_latest_dt_ms': {
            'p50': pct(naive_dts, 50),
            'p95': pct(naive_dts, 95),
            'p99': pct(naive_dts, 99),
            'max': max(naive_dts) if naive_dts else None,
            'mean': sum(naive_dts) / len(naive_dts) if naive_dts else None,
        },
        'stage0_reference_dt_ms': {'p50': 109, 'p95': 307, 'p99': 358, 'max': 364},
        'note': 'Nearest pairing vs naive latest; stamps are ROS now() not hardware capture.',
    }
    out = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07/task3_nearest_pair.json')
    out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    n.destroy_node()
    n2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
