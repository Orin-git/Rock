"""OpenCV ORB helpers — minimal dependency (no DBoW2)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class OrbFrame:
    keypoints: List[cv2.KeyPoint]
    descriptors: Optional[np.ndarray]
    n_features: int


def make_orb(n_features: int = 1000) -> cv2.ORB:
    return cv2.ORB_create(nfeatures=int(n_features))


def extract_orb(bgr_or_gray: np.ndarray, orb: Optional[cv2.ORB] = None) -> OrbFrame:
    if orb is None:
        orb = make_orb()
    if bgr_or_gray.ndim == 3:
        gray = cv2.cvtColor(bgr_or_gray, cv2.COLOR_BGR2GRAY)
    else:
        gray = bgr_or_gray
    kps, desc = orb.detectAndCompute(gray, None)
    kps = list(kps or [])
    return OrbFrame(keypoints=kps, descriptors=desc, n_features=len(kps))


def match_orb(
    desc_q: np.ndarray,
    desc_t: np.ndarray,
    ratio: float = 0.75,
) -> Tuple[List[cv2.DMatch], List[cv2.DMatch]]:
    """Return (raw_knn_best, ratio_test_matches)."""
    if desc_q is None or desc_t is None or len(desc_q) < 2 or len(desc_t) < 2:
        return [], []
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    knn = bf.knnMatch(desc_q, desc_t, k=2)
    raw: List[cv2.DMatch] = []
    good: List[cv2.DMatch] = []
    for pair in knn:
        if len(pair) < 2:
            continue
        m, n = pair[0], pair[1]
        raw.append(m)
        if m.distance < ratio * n.distance:
            good.append(m)
    return raw, good


def visual_difference(desc_a: Optional[np.ndarray], desc_b: Optional[np.ndarray]) -> float:
    """1 - inlier_match_fraction proxy in [0,1]; higher = more different."""
    if desc_a is None or desc_b is None:
        return 1.0
    _, good = match_orb(desc_a, desc_b, ratio=0.8)
    denom = max(min(len(desc_a), len(desc_b)), 1)
    return 1.0 - (len(good) / float(denom))


def pack_keypoints(kps: List[cv2.KeyPoint]) -> np.ndarray:
    """Nx7: x,y,size,angle,response,octave,class_id"""
    if not kps:
        return np.zeros((0, 7), dtype=np.float32)
    rows = []
    for k in kps:
        rows.append([k.pt[0], k.pt[1], k.size, k.angle, k.response, float(k.octave), float(k.class_id)])
    return np.asarray(rows, dtype=np.float32)


def unpack_keypoints(arr: np.ndarray) -> List[cv2.KeyPoint]:
    kps: List[cv2.KeyPoint] = []
    for row in arr:
        kps.append(
            cv2.KeyPoint(
                x=float(row[0]),
                y=float(row[1]),
                size=float(row[2]),
                angle=float(row[3]),
                response=float(row[4]),
                octave=int(row[5]),
                class_id=int(row[6]),
            )
        )
    return kps
