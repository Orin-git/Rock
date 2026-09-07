"""RGB-D geometric verification: PnP + query depth consistency."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from xw_global_reloc.orb_utils import match_orb


@dataclass
class GeometryResult:
    accepted: bool
    reason: str
    inliers: int
    inlier_ratio: float
    reproj_error: float
    depth_consistency: float
    matches: int
    T_query_from_kf: Optional[np.ndarray]
    runtime_sec: float
    visual_score: float
    geometry_score: float


def backproject(
    u: float,
    v: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return np.array([x, y, depth_m], dtype=np.float64)


def depth_at(depth_m: np.ndarray, u: float, v: float) -> float:
    h, w = depth_m.shape[:2]
    x = int(round(u))
    y = int(round(v))
    if x < 0 or y < 0 or x >= w or y >= h:
        return 0.0
    return float(depth_m[y, x])


def verify_pnp_rgbd(
    query_kps,
    query_desc: np.ndarray,
    query_depth_m: np.ndarray,
    kf_kps,
    kf_desc: np.ndarray,
    kf_depth_m: np.ndarray,
    K: np.ndarray,
    *,
    min_matches: int = 20,
    min_pnp_inliers: int = 12,
    min_inlier_ratio: float = 0.35,
    max_reproj_error: float = 4.0,
    min_depth_consistency: float = 0.55,
    depth_rel_tol: float = 0.12,
    depth_abs_tol_m: float = 0.08,
    ratio: float = 0.75,
) -> GeometryResult:
    t0 = time.monotonic()
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])

    raw, good = match_orb(query_desc, kf_desc, ratio=ratio)
    if len(good) < min_matches:
        return GeometryResult(
            False, 'insufficient_matches', 0, 0.0, 999.0, 0.0, len(good),
            None, time.monotonic() - t0, 0.0, 0.0,
        )

    obj_pts: List[np.ndarray] = []
    img_pts: List[np.ndarray] = []
    q_uvs: List[Tuple[float, float]] = []
    for m in good:
        kp_t = kf_kps[m.trainIdx]
        kp_q = query_kps[m.queryIdx]
        du, dv = float(kp_t.pt[0]), float(kp_t.pt[1])
        qu, qv = float(kp_q.pt[0]), float(kp_q.pt[1])
        d = depth_at(kf_depth_m, du, dv)
        if d <= 0.05 or d > 8.0:
            continue
        obj_pts.append(backproject(du, dv, d, fx, fy, cx, cy))
        img_pts.append(np.array([qu, qv], dtype=np.float64))
        q_uvs.append((qu, qv))

    if len(obj_pts) < min_matches:
        return GeometryResult(
            False, 'insufficient_3d', 0, 0.0, 999.0, 0.0, len(obj_pts),
            None, time.monotonic() - t0, 0.0, 0.0,
        )

    obj = np.asarray(obj_pts, dtype=np.float64)
    img = np.asarray(img_pts, dtype=np.float64)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj,
        img,
        K,
        None,
        iterationsCount=200,
        reprojectionError=float(max_reproj_error),
        confidence=0.99,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok or inliers is None:
        return GeometryResult(
            False, 'pnp_failed', 0, 0.0, 999.0, 0.0, len(obj_pts),
            None, time.monotonic() - t0, 0.0, 0.0,
        )

    inliers = np.asarray(inliers).reshape(-1)
    n_inl = int(len(inliers))
    ratio_inl = n_inl / float(max(len(obj_pts), 1))
    if n_inl < min_pnp_inliers or ratio_inl < min_inlier_ratio:
        return GeometryResult(
            False, 'inliers_gate', n_inl, ratio_inl, 999.0, 0.0, len(obj_pts),
            None, time.monotonic() - t0, ratio_inl, 0.0,
        )

    proj, _ = cv2.projectPoints(obj[inliers], rvec, tvec, K, None)
    proj = proj.reshape(-1, 2)
    err = np.linalg.norm(proj - img[inliers], axis=1)
    mean_reproj = float(np.mean(err)) if len(err) else 999.0
    if mean_reproj > max_reproj_error:
        return GeometryResult(
            False, 'reproj_gate', n_inl, ratio_inl, mean_reproj, 0.0, len(obj_pts),
            None, time.monotonic() - t0, ratio_inl, 0.0,
        )

    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)
    # Depth consistency: transform kf 3D → query cam; compare to query depth
    consistent = 0
    checked = 0
    for idx in inliers.tolist():
        p_kf = obj[idx]
        p_q = R @ p_kf + t
        if p_q[2] <= 0.05:
            continue
        qu, qv = q_uvs[idx]
        dq = depth_at(query_depth_m, qu, qv)
        if dq <= 0.05 or dq > 8.0:
            continue
        checked += 1
        tol = max(depth_abs_tol_m, depth_rel_tol * dq)
        if abs(float(p_q[2]) - dq) <= tol:
            consistent += 1
    depth_cons = float(consistent) / float(max(checked, 1))
    if checked < max(6, min_pnp_inliers // 2) or depth_cons < min_depth_consistency:
        return GeometryResult(
            False, 'depth_consistency_gate', n_inl, ratio_inl, mean_reproj, depth_cons,
            len(obj_pts), None, time.monotonic() - t0, ratio_inl, depth_cons,
        )

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    visual = float(min(1.0, ratio_inl))
    geom = float(0.5 * (1.0 - min(mean_reproj / max_reproj_error, 1.0)) + 0.5 * depth_cons)
    return GeometryResult(
        True, 'ok', n_inl, ratio_inl, mean_reproj, depth_cons, len(obj_pts),
        T, time.monotonic() - t0, visual, geom,
    )
