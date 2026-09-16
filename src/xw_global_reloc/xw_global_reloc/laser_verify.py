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


# ===========================================================================
# Ray-consistency criterion.
#
# ADDITIVE ONLY. Nothing above this line changes. The endpoint-proximity scorer
# (`_score_from_dists` / `score_scan_at_pose*`) still drives every existing
# caller; this block adds a second, independent primitive.
#
# Why: the scalar above measures "is my endpoint near *some* occupied cell",
# not "should this ray have travelled that far". In a map fragmented into
# hundreds of small components (M1: 419 components, 70.7% of occupied cells in
# components < 4 m) the two diverge, and 59% of its "matches" at the true pose
# were spurious. A pose sitting next to clutter can therefore out-score the
# true pose. This primitive asks the ray question instead:
#
#   march each beam from the pose along its own heading and locate `d_map`,
#   the distance to the FIRST occupied cell on that ray. Then
#     |r - d_map| <= tol   MATCH    the map agrees with the measurement
#     r < d_map - tol      EARLY    something unmapped stopped the beam
#                                   (neutral: expected in a real building)
#     r > d_map + tol      THROUGH  the beam crossed a mapped wall
#                                   (a pose contradiction -- walls are solid)
#
# EARLY being neutral is the whole point: an object that was never mapped must
# not be able to condemn the pose.
# ===========================================================================

# Per-beam classification, as returned in `out_beams` for telemetry/sectors.
BEAM_OOB = 0
BEAM_MATCH = 1
BEAM_EARLY = 2
BEAM_THROUGH = 3
BEAM_GRAZING = 4
BEAM_NOHIT = 5

RAY_STRIDE = 1  # G8: use every beam (stride=6 throws away 5/6 of the evidence)
RAY_TOL_M = 0.25
RAY_STEP_M = 0.025  # G1: half a cell -- see note in `ray_consistency_at_pose`
RAY_LONG_BEAM_M = 1.5
RAY_GRAZING_M = 0.30  # G5
RAY_MIN_ORIGIN_CLEAR_M = 0.15  # G4
RAY_MIN_EVIDENCE = 40  # G7
RAY_MIN_LONG_BEAMS = 40  # G9
RAY_MIN_STRUCTURE_MATCH = 15  # G14


@dataclass
class RayConsistency:
    """Ray-marching verdict for one scan at one pose.

    All counts are over the in-grid evidence set unless stated otherwise.
    `through_ratio` and `long_match_ratio` are the two discriminants; the
    forward one (`long_match_ratio`) is monotone in pose error, the backward
    one (`through_ratio`) is not, so a verdict must use both.
    """

    n_beams: int  # beams after stride, before any exclusion
    n_steps: int  # march length in steps (runtime proxy)
    n_evidence: int  # in-grid beams = denominator for through_ratio
    n_oob: int  # rays that left the grid before reaching anything
    n_match: int
    n_early: int
    n_through: int
    n_grazing: int
    n_nohit: int
    n_long: int  # in-grid beams with r >= long_beam_m
    n_long_match: int
    through_ratio: float  # n_through / n_evidence
    long_match_ratio: float  # n_long_match / n_long
    max_through_run: int  # longest angularly contiguous run of THROUGH (G6)
    origin_blocked: bool  # G4: pose in / hugging an occupied cell
    origin_dist_m: float  # distance from pose to nearest occupied cell
    structure_ok: bool  # G14: enough positive evidence to judge at all
    reason: str
    runtime_sec: float


def _max_true_run(flags: np.ndarray, adjacent: Optional[np.ndarray] = None) -> int:
    """Longest run of consecutive True.

    `adjacent[i]` must be True when beam i is angularly adjacent to beam i+1.
    This matters: `prepare_scan` drops every non-finite return, and the driver
    reports "no echo" as `inf` (measured: 478 of 1800 beams). After a stride of
    6 the surviving beams are spaced anywhere from 1.19 deg to 29.93 deg apart,
    so two beams adjacent in the array can be 25 beam-widths apart on the
    sensor. A "contiguous sector" must be contiguous in angle, not in index.
    """
    if flags.size == 0 or not flags.any():
        return 0
    b = flags.astype(np.int8)
    if adjacent is not None and adjacent.size == flags.size - 1:
        b = b * np.concatenate((adjacent.astype(np.int8), np.ones(1, np.int8)))
    # The guard above tests `flags`; the reduction below runs on `b`. They
    # differ because the multiply drops a flagged beam whenever the NEXT
    # prepared beam is angularly far (a gap left by a dropped `inf` return).
    # If that happens to every flagged beam, `b` is all zeros, `d` has no +1
    # and no -1, starts/ends are empty, and `.max()` raises
    #   ValueError: zero-size array to reduction operation maximum which has
    #   no identity
    # The trailing `np.ones(1)` above spares the LAST array element, so the
    # crash needs the final beam to be unflagged as well. Found 2026-09-16 by
    # the wide pose search (raycal_wide.py); T1-T9 did not cover it.
    if not b.any():
        return 0
    d = np.diff(np.concatenate((np.zeros(1, np.int8), b, np.zeros(1, np.int8))))
    starts = np.flatnonzero(d == 1)
    ends = np.flatnonzero(d == -1)
    return int((ends - starts).max())


def ray_consistency_at_pose(
    field: DistanceField,
    scan: Optional[LaserScan],
    x: float,
    y: float,
    yaw: float,
    *,
    beam_stride: int = RAY_STRIDE,
    tol_m: float = RAY_TOL_M,
    step_m: float = RAY_STEP_M,
    max_range_m: Optional[float] = None,
    long_beam_m: float = RAY_LONG_BEAM_M,
    grazing_m: float = RAY_GRAZING_M,
    min_origin_clear_m: float = RAY_MIN_ORIGIN_CLEAR_M,
    min_structure_match: int = RAY_MIN_STRUCTURE_MATCH,
    run_gap_factor: float = 1.5,
    step_mode: str = 'uniform',
    prepared: Optional[PreparedScan] = None,
    out_beams: Optional[List[np.ndarray]] = None,
) -> RayConsistency:
    """March every beam along its own heading and classify it.

    Stride defaults to 1 rather than the legacy 6: at 2 Hz the full 1374-beam
    sweep costs ~13 ms, so there is no reason to discard 5/6 of the evidence.

    `step_m` defaults to half a cell. At one whole cell (5 cm) the march steps
    over walls it should have found: measured against an exact DDA over the
    four calibration poses, 5 cm reported "no wall here" for 9-16 beams per
    pose that a finer march resolved; 2.5 cm roughly halves that. See
    `step_mode` for why the sampled step is Euclidean rather than dominant-axis.
    """
    t0 = time.monotonic()
    res = float(field.resolution)

    # ---- G4: the pose itself must not sit in or hug an occupied cell. ------
    # An origin inside an occupied cell makes step 1 the "first hit", so
    # d_map ~ 0.05 and every beam reads THROUGH. With 133 single-cell
    # components in this map that is not a remote possibility, and the failure
    # is total (through_ratio ~ 1.0), so the frame abstains instead.
    oix = int(math.floor((x - field.origin_x) / res))
    oiy = int(math.floor((y - field.origin_y) / res))
    if not (0 <= oix < field.width and 0 <= oiy < field.height):
        return RayConsistency(
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0, True,
            float('inf'), False, 'origin_oob', time.monotonic() - t0,
        )
    origin_dist = float(field.dist_m[oiy, oix])
    if bool(field.occ[oiy, oix]) or origin_dist < min_origin_clear_m:
        return RayConsistency(
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0, True,
            origin_dist, False, 'origin_blocked', time.monotonic() - t0,
        )

    prep = prepared if prepared is not None else prepare_scan(scan, beam_stride=beam_stride)
    n = int(prep.n_valid)
    if n == 0:
        return RayConsistency(
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0, False,
            origin_dist, False, 'few_beams', time.monotonic() - t0,
        )

    # `max_range_m` may be supplied directly so that a caller holding only a
    # `PreparedScan` does not need to keep the message around.
    if max_range_m is not None:
        rmax = float(max_range_m)
    elif scan is not None:
        rmax = float(scan.range_max)
    else:
        raise ValueError('need either scan or max_range_m')
    bx = np.asarray(prep.bx, dtype=np.float64)
    by = np.asarray(prep.by, dtype=np.float64)
    r = np.hypot(bx, by)
    # The lidar-frame beam angle is recoverable exactly from the endpoint, so
    # this is independent of the stride used to build `prep`.
    ang = np.arctan2(by, bx)
    # URDF: lidar_link yaw = pi vs base_link (same convention as score_scan_at_pose)
    th = yaw + math.pi + ang
    dx = np.cos(th)
    dy = np.sin(th)

    # `uniform` is the rule the M3/M4 reference used, and therefore the only
    # one that reproduces those eight numbers exactly. `dominant` advances
    # step_m along whichever axis the ray traverses faster, so the cell index
    # changes by at most one per step and no cell is stepped over. That sounds
    # strictly better and is not: measured against an exact DDA over the four
    # calibration poses it came out 0.0368 mean vs 0.0377 for uniform, i.e. a
    # wash -- better on two poses, worse on one, and with a 0.3775 m worst case
    # on A_claimed against uniform's 0.0249. Kept because it is a real option
    # for a better-calibrated scene, not because it was shown to win.
    #
    # What the measurement DOES support is 2.5 cm over 5 cm: at 5 cm -- one
    # whole cell -- 9 to 16 of the walls the fine march finds are stepped over
    # and reported as "no wall here". 2.5 cm roughly halves that.
    if step_mode == 'dominant':
        axis = np.maximum(np.maximum(np.abs(dx), np.abs(dy)), 1e-9)
        s = step_m / axis
    else:
        s = np.full(n, float(step_m))
    sx = dx * s
    sy = dy * s

    eps = max(1e-3, step_m)
    # G3: a saturated return means "no echo", not "a wall at range_max".
    saturated = r >= (rmax - eps)
    # Searching past r + tol cannot change any classification: a first hit
    # beyond r + tol is an EARLY by definition.
    cap = np.minimum(r + tol_m, rmax)

    steps = int(math.ceil(float(np.max(cap / s)))) + 1

    ks = np.arange(1, steps + 1, dtype=np.float64)[:, None]  # (K, 1)
    tdist = ks * s[None, :]  # (K, N) Euclidean distance travelled
    fx = ((x - field.origin_x) + ks * sx[None, :]) / res
    fy = ((y - field.origin_y) + ks * sy[None, :]) / res
    ix = np.floor(fx).astype(np.int32)
    iy = np.floor(fy).astype(np.int32)
    inb = (ix >= 0) & (iy >= 0) & (ix < field.width) & (iy < field.height)

    # G2: np.floor throughout. `world_to_ixy` uses int() truncation and
    # disagrees with `in_map_mask`'s np.floor by one cell below the origin.
    np.clip(ix, 0, field.width - 1, out=ix)
    np.clip(iy, 0, field.height - 1, out=iy)
    hit = (field.occ[iy, ix] > 0) & inb

    within_cap = tdist <= cap[None, :]
    valid = hit & within_cap
    any_hit = valid.any(axis=0)
    cols = np.arange(n)
    d_map = np.where(any_hit, tdist[valid.argmax(axis=0), cols], np.inf)

    # A ray leaves a convex grid rectangle at most once, so ~inb is monotone.
    # Only the steps a beam actually searches may count: the sample array is
    # rectangular, padded out to the longest march in the frame, and a short
    # beam's padding can walk off the grid without that saying anything about
    # the beam. Reading that as "this ray left the map" silently drops good
    # beams from the denominator.
    oob_any = ((~inb) & within_cap).any(axis=0)
    # G7: only an in-grid ray can carry a MATCH or a THROUGH. A ray that walks
    # off the map proves nothing either way, so it leaves the denominator.
    is_oob = (~any_hit) & oob_any
    is_nohit = (~any_hit) & (~is_oob) & saturated
    is_early = (any_hit & (r < d_map - tol_m)) | ((~any_hit) & (~is_oob) & (~saturated))
    is_match = any_hit & (np.abs(r - d_map) <= tol_m)
    over = any_hit & (r > d_map + tol_m)
    # G5: a near-field first hit means the ray grazed a wall corner or clipped
    # a single-cell map fragment; the geometry is a millisecond-scale artefact,
    # not a pose contradiction. Counted, but not as evidence.
    is_through = over & (d_map >= grazing_m)
    is_grazing = over & (d_map < grazing_m)

    # G6: a lone beam flipping is noise; a contiguous angular sector is signal.
    # Adjacency is taken from the actual angles, not the array index, because
    # dropped beams leave arbitrary gaps (see `_max_true_run`).
    if n >= 2:
        gap_nom = float(np.median(np.diff(ang)))
        adjacent = np.diff(ang) <= max(gap_nom * run_gap_factor, 1e-9)
    else:
        adjacent = np.zeros(0, dtype=bool)
    beam_class = np.full(n, BEAM_OOB, dtype=np.uint8)
    beam_class[is_nohit] = BEAM_NOHIT
    beam_class[is_early] = BEAM_EARLY
    beam_class[is_match] = BEAM_MATCH
    beam_class[is_grazing] = BEAM_GRAZING
    beam_class[is_through] = BEAM_THROUGH

    n_evidence = int(n - int(is_oob.sum()))
    n_through = int(is_through.sum())
    long_arr = (r >= long_beam_m) & (~is_oob)
    n_long = int(long_arr.sum())
    n_long_match = int((long_arr & is_match).sum())
    n_match = int(is_match.sum())

    rc = RayConsistency(
        n_beams=n,
        n_steps=steps,
        n_evidence=n_evidence,
        n_oob=int(is_oob.sum()),
        n_match=n_match,
        n_early=int(is_early.sum()),
        n_through=n_through,
        n_grazing=int(is_grazing.sum()),
        n_nohit=int(is_nohit.sum()),
        n_long=n_long,
        n_long_match=n_long_match,
        through_ratio=float(n_through) / float(max(n_evidence, 1)),
        long_match_ratio=float(n_long_match) / float(max(n_long, 1)),
        max_through_run=_max_true_run(is_through, adjacent),
        origin_blocked=False,
        origin_dist_m=origin_dist,
        # G14: with every occupied cell in this map flattened to "free" by
        # free_thresh=0.25 (M2), a pose in a never-observed area produces no
        # MATCH at all. Judging that LOST would be as wrong as judging it OK;
        # the frame abstains and the telemetry says so.
        structure_ok=n_match >= min_structure_match,
        reason='ok',
        runtime_sec=time.monotonic() - t0,
    )
    if not rc.structure_ok:
        rc.reason = 'no_structure'
    if out_beams is not None:
        out_beams.append(beam_class)
    return rc


def ray_mismatch(
    rc: RayConsistency,
    *,
    max_through_ratio: float,
    min_long_match_ratio: float,
    min_through_run: int = 3,
    min_evidence: int = RAY_MIN_EVIDENCE,
    min_long_beams: int = RAY_MIN_LONG_BEAMS,
) -> Tuple[bool, str]:
    """Two-sided decision rule. Returns (mismatch, reason).

    The two thresholds are keyword-required with no defaults on purpose: they
    may only come from an on-robot multi-frame calibration, never from a single
    measurement or from reasoning about the code.

    Both sides are needed. Measured on a good pose vs one displaced 0.5 m,
    `through_ratio` moves 0.066 -> 0.228 while a 1.63 m displacement only
    reaches 0.316 -- it saturates, so a 0.5 m error is already most of the way
    to a 1.6 m error. `long_match_ratio` is monotone across the same range
    (0.340 -> 0.087 -> 0.107 -> 0.047), so it supplies what `through_ratio`
    loses. Neither alone separates the cases; together they do.
    """
    if rc.origin_blocked:
        return False, 'abstain_' + rc.reason
    if rc.n_evidence < min_evidence:
        return False, 'abstain_few_evidence'
    if not rc.structure_ok:
        return False, 'abstain_no_structure'
    if (
        rc.n_through >= min_through_run
        and rc.max_through_run >= min_through_run
        and rc.through_ratio >= max_through_ratio
    ):
        return True, 'through_walls'
    if rc.n_long >= min_long_beams and rc.long_match_ratio <= min_long_match_ratio:
        return True, 'long_beams_do_not_match'
    return False, 'ok'
