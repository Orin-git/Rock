"""Laser verification via precomputed 2D distance field."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

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


@dataclass
class PreparedScan:
    """Precomputed beam endpoints in lidar frame (stride already applied)."""

    bx: np.ndarray  # (N,)
    by: np.ndarray  # (N,)
    n_valid: int


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
        try:
            import cv2

            inv = (1 - occ).astype(np.uint8)
            return cv2.distanceTransform(inv, cv2.DIST_L2, 3)
        except Exception:  # noqa: BLE001
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

    def in_map_mask(self, mx: np.ndarray, my: np.ndarray) -> np.ndarray:
        """True where the endpoint lies inside the occupancy grid."""
        ix = np.floor((mx - self.origin_x) / self.resolution).astype(np.int32)
        iy = np.floor((my - self.origin_y) / self.resolution).astype(np.int32)
        return (ix >= 0) & (iy >= 0) & (ix < self.width) & (iy < self.height)

    def sample_dist_batch(self, mx: np.ndarray, my: np.ndarray) -> np.ndarray:
        """Sample distance field at world points; OOB → 1e3 sentinel.

        Callers that form a mean must drop the sentinel. An endpoint past the
        map border is not a 1000 m mismatch.
        """
        ix = np.floor((mx - self.origin_x) / self.resolution).astype(np.int32)
        iy = np.floor((my - self.origin_y) / self.resolution).astype(np.int32)
        out = np.full(mx.shape, 1e3, dtype=np.float64)
        ok = (ix >= 0) & (iy >= 0) & (ix < self.width) & (iy < self.height)
        if np.any(ok):
            out[ok] = self.dist_m[iy[ok], ix[ok]]
        return out

    def sample_in_map_dists(self, mx: np.ndarray, my: np.ndarray) -> np.ndarray:
        """Distance-to-obstacle for in-map endpoints only."""
        ok = self.in_map_mask(mx, my)
        if not np.any(ok):
            return np.empty(0, dtype=np.float64)
        return self.sample_dist_batch(mx[ok], my[ok])


def prepare_scan(scan: LaserScan, beam_stride: int = 6) -> PreparedScan:
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    rmin = float(scan.range_min)
    rmax = float(scan.range_max)
    stride = max(1, int(beam_stride))
    idx = np.arange(0, len(ranges), stride, dtype=np.int32)
    r = ranges[idx]
    valid = np.isfinite(r) & (r >= rmin) & (r <= rmax)
    idx = idx[valid]
    r = r[valid]
    ang = float(scan.angle_min) + idx.astype(np.float64) * float(scan.angle_increment)
    bx = r * np.cos(ang)
    by = r * np.sin(ang)
    return PreparedScan(bx=bx, by=by, n_valid=int(bx.size))


def _score_from_dists(
    dists: np.ndarray,
    *,
    match_dist_m: float,
    min_valid_beams: int,
    min_laser_score: float,
    runtime_sec: float,
) -> LaserScore:
    valid = int(dists.size)
    if valid < min_valid_beams:
        return LaserScore(False, 'few_beams', valid, 0.0, 999.0, 999.0, 0.0, runtime_sec)
    mean_d = float(np.mean(dists))
    p90 = float(np.percentile(dists, 90))
    matched = int(np.count_nonzero(dists <= match_dist_m))
    matched_ratio = float(matched) / float(valid)
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
        runtime_sec,
    )


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
    min_laser_score: float = 0.38,
    prepared: Optional[PreparedScan] = None,
) -> LaserScore:
    t0 = time.monotonic()
    prep = prepared if prepared is not None else prepare_scan(scan, beam_stride=beam_stride)
    if prep.n_valid < min_valid_beams:
        return LaserScore(
            False, 'few_beams', prep.n_valid, 0.0, 999.0, 999.0, 0.0, time.monotonic() - t0
        )
    # URDF: lidar_link yaw = π vs base_link
    lyaw = yaw + math.pi
    lc, ls = math.cos(lyaw), math.sin(lyaw)
    mx = x + lc * prep.bx - ls * prep.by
    my = y + ls * prep.bx + lc * prep.by
    # Drop out-of-map endpoints. The 1e3 OOB sentinel must not enter the mean.
    dists = field.sample_in_map_dists(mx, my)
    return _score_from_dists(
        dists,
        match_dist_m=match_dist_m,
        min_valid_beams=min_valid_beams,
        min_laser_score=min_laser_score,
        runtime_sec=time.monotonic() - t0,
    )


def score_scan_at_poses(
    field: DistanceField,
    prepared: PreparedScan,
    xs: np.ndarray,
    ys: np.ndarray,
    yaws: np.ndarray,
    *,
    match_dist_m: float = 0.25,
    min_valid_beams: int = 20,
    min_laser_score: float = 0.38,
) -> List[LaserScore]:
    """Vectorized scoring for many SE(2) poses with one prepared scan."""
    t0 = time.monotonic()
    xs = np.asarray(xs, dtype=np.float64).reshape(-1)
    ys = np.asarray(ys, dtype=np.float64).reshape(-1)
    yaws = np.asarray(yaws, dtype=np.float64).reshape(-1)
    n = int(xs.size)
    if n == 0:
        return []
    if prepared.n_valid < min_valid_beams:
        rt = time.monotonic() - t0
        return [
            LaserScore(False, 'few_beams', prepared.n_valid, 0.0, 999.0, 999.0, 0.0, rt)
            for _ in range(n)
        ]
    lyaw = yaws + math.pi
    lc = np.cos(lyaw)
    ls = np.sin(lyaw)
    mx = xs[:, None] + lc[:, None] * prepared.bx[None, :] - ls[:, None] * prepared.by[None, :]
    my = ys[:, None] + ls[:, None] * prepared.bx[None, :] + lc[:, None] * prepared.by[None, :]
    in_map = field.in_map_mask(mx, my)
    dists = field.sample_dist_batch(mx.reshape(-1), my.reshape(-1)).reshape(n, prepared.n_valid)
    rt = time.monotonic() - t0
    out: List[LaserScore] = []
    for i in range(n):
        out.append(
            _score_from_dists(
                dists[i][in_map[i]],
                match_dist_m=match_dist_m,
                min_valid_beams=min_valid_beams,
                min_laser_score=min_laser_score,
                runtime_sec=rt / max(n, 1),
            )
        )
    return out
