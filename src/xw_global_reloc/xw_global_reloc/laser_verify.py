"""Laser verification via precomputed 2D distance field."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


@dataclass
class LaserScore:
    accepted: bool
    reason: str
    valid_beams: int
    matched_ratio: float
    mean_dist: float
    p90_dist: float
    laser_score: float
    runtime_sec: float


class DistanceField:
    def __init__(self, grid: OccupancyGrid, occupied_thresh: int = 50) -> None:
        info = grid.info
        self.resolution = float(info.resolution)
        self.origin_x = float(info.origin.position.x)
        self.origin_y = float(info.origin.position.y)
        self.width = int(info.width)
        self.height = int(info.height)
        data = np.asarray(grid.data, dtype=np.int16).reshape(self.height, self.width)
        occ = (data >= occupied_thresh).astype(np.uint8)
        # Free/unknown treated as non-occupied for distance-to-obstacle field.
        self.occ = occ
        self.dist_m = self._distance_transform(occ) * self.resolution
        # Free mask for candidate footprint check (unknown=-1, free < occupied_thresh)
        self.free = (data >= 0) & (data < occupied_thresh)

    @staticmethod
    def _distance_transform(occ: np.ndarray) -> np.ndarray:
        # Pure numpy EDT approximation via OpenCV if available.
        try:
            import cv2

            # distance to nearest zero in (1-occ) → distance to occupied
            inv = (1 - occ).astype(np.uint8)
            return cv2.distanceTransform(inv, cv2.DIST_L2, 3)
        except Exception:  # noqa: BLE001
            # Fallback slow: coarse
            ys, xs = np.where(occ > 0)
            if len(xs) == 0:
                return np.full(occ.shape, 1e3, dtype=np.float32)
            out = np.full(occ.shape, 1e3, dtype=np.float32)
            for y in range(occ.shape[0]):
                for x in range(occ.shape[1]):
                    d = np.sqrt((xs - x) ** 2 + (ys - y) ** 2)
                    out[y, x] = float(np.min(d))
            return out

    def world_to_ixy(self, x: float, y: float) -> Tuple[int, int]:
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        return ix, iy

    def sample_dist(self, x: float, y: float) -> float:
        ix, iy = self.world_to_ixy(x, y)
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return 1e3
        return float(self.dist_m[iy, ix])

    def is_free(self, x: float, y: float) -> bool:
        ix, iy = self.world_to_ixy(x, y)
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return False
        return bool(self.free[iy, ix])


def score_scan_at_pose(
    field: DistanceField,
    scan: LaserScan,
    x: float,
    y: float,
    yaw: float,
    *,
    beam_stride: int = 6,
    match_dist_m: float = 0.25,
    min_valid_beams: int = 20,
    min_laser_score: float = 0.45,
) -> LaserScore:
    t0 = time.monotonic()
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    rmin = float(scan.range_min)
    rmax = float(scan.range_max)
    dists = []
    valid = 0
    matched = 0
    stride = max(1, int(beam_stride))
    # URDF: lidar_link yaw = π vs base_link (see xw_gen2.urdf).
    lyaw = yaw + math.pi
    lc, ls = math.cos(lyaw), math.sin(lyaw)
    for i in range(0, len(ranges), stride):
        r = float(ranges[i])
        if not math.isfinite(r) or r < rmin or r > rmax:
            continue
        ang = float(scan.angle_min) + float(i) * float(scan.angle_increment)
        bx = r * math.cos(ang)
        by = r * math.sin(ang)
        mx = x + lc * bx - ls * by
        my = y + ls * bx + lc * by
        d = field.sample_dist(mx, my)
        dists.append(d)
        valid += 1
        if d <= match_dist_m:
            matched += 1

    if valid < min_valid_beams:
        return LaserScore(
            False, 'few_beams', valid, 0.0, 999.0, 999.0, 0.0, time.monotonic() - t0
        )
    arr = np.asarray(dists, dtype=np.float64)
    mean_d = float(np.mean(arr))
    p90 = float(np.percentile(arr, 90))
    matched_ratio = float(matched) / float(valid)
    # Score: high match ratio, low mean distance
    score = matched_ratio * float(np.clip(1.0 - mean_d / max(match_dist_m * 4.0, 1e-3), 0.0, 1.0))
    ok = score >= min_laser_score and matched_ratio >= min_laser_score * 0.8
    return LaserScore(
        ok,
        'ok' if ok else 'laser_gate',
        valid,
        matched_ratio,
        mean_d,
        p90,
        float(score),
        time.monotonic() - t0,
    )
