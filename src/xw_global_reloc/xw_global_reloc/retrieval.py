"""Offline visual retrieval — OpenCV ORB only, no final pose."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from xw_global_reloc.orb_utils import extract_orb, match_orb, make_orb


@dataclass
class RetrievalCandidate:
    keyframe_id: str
    raw_matches: int
    ratio_matches: int
    score: float
    rank: int = 0


@dataclass
class RetrievalResult:
    query_features: int
    candidates: List[RetrievalCandidate] = field(default_factory=list)
    runtime_sec: float = 0.0


def score_keyframe(ratio_matches: int, query_features: int) -> float:
    if query_features <= 0:
        return 0.0
    return float(ratio_matches) / float(query_features)


def retrieve_topk(
    query_bgr: np.ndarray,
    keyframes: List[Dict[str, Any]],
    top_k: int = 10,
    ratio: float = 0.75,
    n_features: int = 1000,
) -> RetrievalResult:
    """keyframes items need: id, descriptors (Nx32 uint8)."""
    t0 = time.monotonic()
    orb = make_orb(n_features)
    q = extract_orb(query_bgr, orb)
    scored: List[RetrievalCandidate] = []
    for kf in keyframes:
        desc = kf.get('descriptors')
        if desc is None:
            continue
        raw, good = match_orb(q.descriptors, desc, ratio=ratio)
        sc = score_keyframe(len(good), max(q.n_features, 1))
        scored.append(
            RetrievalCandidate(
                keyframe_id=str(kf['id']),
                raw_matches=len(raw),
                ratio_matches=len(good),
                score=sc,
            )
        )
    scored.sort(key=lambda c: c.score, reverse=True)
    for i, c in enumerate(scored[: max(1, top_k)]):
        c.rank = i + 1
    return RetrievalResult(
        query_features=q.n_features,
        candidates=scored[: max(1, top_k)],
        runtime_sec=time.monotonic() - t0,
    )
