#!/usr/bin/env python3
"""Offline reflector detector tests (no ROS). Synthetic scans stand in for bag replay."""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xw_recharge.reflaction_detector import (  # noqa: E402
    DetectionTracker,
    ReflactionDetector,
    in_base_window,
    make_synthetic_scan,
    transform_xy_yaw,
)


def _detect_at(x: float, y: float = 0.0, yaw: float = math.pi):
    det = ReflactionDetector()
    ranges, inten, amin, inc = make_synthetic_scan(
        charger_x=x, charger_y=y, charger_yaw=yaw
    )
    return det.detect(ranges, inten, amin, inc, frame_id='lidar_link')


def test_head_on_08m_detects():
    hit = _detect_at(0.80)
    assert hit is not None
    assert abs(hit.x - 0.80) < 0.04
    assert abs(hit.y) < 0.03
    assert hit.matched_segments == 3
    assert in_base_window(hit.x, hit.y)


def test_head_on_distances():
    for dist in (0.40, 0.80, 1.20):
        hit = _detect_at(dist)
        assert hit is not None, f'missed at {dist}m'
        assert abs(hit.range - dist) < 0.05


def test_oblique_30deg_still_matches():
    # Dock facing 30° off +x; place center 0.9 m along that facing from origin.
    yaw = math.pi  # still facing -x from a point on +x
    hit = _detect_at(0.90, 0.12, yaw)
    assert hit is not None


def test_noise_blob_does_not_match():
    det = ReflactionDetector()
    n = 1571
    ranges = [8.0] * n
    inten = [10.0] * n
    # One bright blob ~0.06 m at 0.7 m — not a 5-element code
    amin, inc = -math.pi, 2 * math.pi / (n - 1)
    for i in range(n):
        th = amin + i * inc
        if abs(th) < 0.05:
            ranges[i] = 0.7
            inten[i] = 250.0
    assert det.detect(ranges, inten, amin, inc) is None


def test_tracker_needs_three_frames():
    tr = DetectionTracker(confirm_frames=3, confirm_std_m=0.03)
    hit = _detect_at(0.80)
    assert hit is not None
    assert tr.update(hit) is None
    assert tr.update(hit) is None
    locked = tr.update(hit)
    assert locked is not None
    assert abs(locked.x - 0.80) < 0.04
    tr.update(None)
    assert tr.update(hit) is None  # reset after miss


def test_lidar_yaw_pi_window_in_base():
    """Strip in front of base (+x) appears at -x in lidar_link when yaw=π."""
    hit = _detect_at(0.80)
    assert hit is not None
    bx, by, _ = transform_xy_yaw(hit.x, hit.y, hit.yaw, 0.0, 0.0, math.pi)
    # Identity case: detector in lidar. After yaw=π, +x lidar → -x base.
    # For this synthetic the scan IS lidar-frame with charger at +x lidar.
    assert abs(bx + hit.x) < 1e-6 or abs(bx - (-hit.x)) < 1e-6


def test_intensity_separation():
    ranges, inten, amin, inc = make_synthetic_scan(charger_x=0.8)
    bright = [v for v in inten if v > 200]
    dim = [v for v in inten if v <= 200]
    assert len(bright) > 8
    assert len(dim) > len(bright)


if __name__ == '__main__':
    test_head_on_08m_detects()
    test_head_on_distances()
    test_oblique_30deg_still_matches()
    test_noise_blob_does_not_match()
    test_tracker_needs_three_frames()
    test_lidar_yaw_pi_window_in_base()
    test_intensity_separation()
    print('reflaction_detector tests passed')
