#!/usr/bin/env python3
"""Phase-0 charger barcode check: synthetic (always) or ros2 bag /scan replay.

Usage:
  ros2 run xw_recharge validate_scan --synthetic
  ros2 run xw_recharge validate_scan --bag /path/to/bag
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import List, Optional


def _run_synthetic(threshold: float) -> int:
    from xw_recharge.reflaction_detector import (
        DetectionTracker,
        DetectorParams,
        ReflactionDetector,
        make_synthetic_scan,
    )

    params = DetectorParams(intensity_threshold=threshold)
    det = ReflactionDetector(params)
    tracker = DetectionTracker()
    cases = [
        ('front_0.40', 0.40, 0.0),
        ('front_0.80', 0.80, 0.0),
        ('front_1.20', 1.20, 0.0),
        ('side_y0.12', 0.90, 0.12),
    ]
    ok = 0
    print(f'synthetic barcode check  threshold={threshold}')
    for name, x, y in cases:
        tracker.reset()
        ranges, inten, amin, inc = make_synthetic_scan(charger_x=x, charger_y=y)
        locked = None
        hits = 0
        for _ in range(8):
            h = det.detect(ranges, inten, amin, inc)
            if h:
                hits += 1
            locked = tracker.update(h)
        status = 'PASS' if locked else 'FAIL'
        if locked:
            ok += 1
            print(
                f'  {status} {name:12s}  raw_hits={hits}/8  '
                f'x={locked.x:.3f} y={locked.y:.3f} r={locked.range:.3f}'
            )
        else:
            print(f'  {status} {name:12s}  raw_hits={hits}/8  (no lock)')
    print(f'{ok}/{len(cases)} locked  (need 95% on real bags before enabling FSM)')
    return 0 if ok == len(cases) else 1


def _run_bag(bag: str, topic: str, threshold: float) -> int:
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        print('rosbag2_py not available; cannot replay bag', file=sys.stderr)
        return 2

    from xw_recharge.reflaction_detector import (
        DetectionTracker,
        DetectorParams,
        ReflactionDetector,
        intensity_histogram,
    )

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag, storage_id='sqlite3'),
        ConverterOptions('', ''),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        print(f'topic {topic} not in bag. have: {sorted(types)}', file=sys.stderr)
        return 2
    msg_cls = get_message(types[topic])
    det = ReflactionDetector(DetectorParams(intensity_threshold=threshold))
    tracker = DetectionTracker()
    n = 0
    hits = 0
    locks = 0
    all_int: List[float] = []
    xs: List[float] = []
    ys: List[float] = []
    while reader.has_next():
        tname, data, _stamp = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(data, msg_cls)
        n += 1
        inten = list(msg.intensities)
        all_int.extend(inten)
        h = det.detect(list(msg.ranges), inten, float(msg.angle_min), float(msg.angle_increment))
        if h:
            hits += 1
        locked = tracker.update(h)
        if locked:
            locks += 1
            xs.append(locked.x)
            ys.append(locked.y)
    rate = (hits / n) if n else 0.0
    print(f'bag {bag}  topic={topic}  frames={n}')
    print(f'  raw detect {hits}/{n} = {rate:.1%}')
    print(f'  tracker lock frames {locks}/{n}')
    if xs:
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        std = math.sqrt(
            sum((a - mx) ** 2 + (b - my) ** 2 for a, b in zip(xs, ys)) / len(xs)
        )
        print(f'  lock mean x={mx:.3f} y={my:.3f} std={std:.4f} m')
        print(f'  PASS geometry' if std < 0.03 and rate >= 0.95 else '  need more data / retune threshold')
    print('  intensity histogram (bin_center, count):')
    for c, k in intensity_histogram(all_int):
        print(f'    {c:7.1f}  {k}')
    return 0 if n and rate >= 0.95 else 1


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description='Validate charger reflector barcode on /scan')
    p.add_argument('--synthetic', action='store_true', help='run built-in coded-strip scans')
    p.add_argument('--bag', default='', help='ros2 bag directory')
    p.add_argument('--topic', default='/scan')
    p.add_argument('--threshold', type=float, default=200.0)
    args = p.parse_args(argv)
    if args.bag:
        return _run_bag(args.bag, args.topic, args.threshold)
    return _run_synthetic(args.threshold)


if __name__ == '__main__':
    sys.exit(main())
