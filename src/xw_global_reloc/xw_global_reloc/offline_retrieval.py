#!/usr/bin/env python3
"""Stage 2 — offline visual retrieval against a saved keyframe DB."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List

import cv2
import numpy as np
import yaml

from xw_global_reloc.orb_utils import unpack_keypoints
from xw_global_reloc.retrieval import retrieve_topk


def load_db(root: Path) -> List[Dict[str, Any]]:
    idx = json.loads((root / 'descriptors' / 'index.json').read_text(encoding='utf-8'))
    out = []
    for item in idx:
        kdir = root / 'keyframes' / item['id']
        desc = np.load(str(kdir / 'descriptors.npy'))
        meta = yaml.safe_load((kdir / 'meta.yaml').read_text(encoding='utf-8'))
        out.append({'id': item['id'], 'descriptors': desc, 'meta': meta})
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--db', required=True)
    ap.add_argument('--query', required=True, help='query RGB image path')
    ap.add_argument('--top-k', type=int, default=10)
    ap.add_argument('--gt-id', default='', help='optional ground-truth keyframe id for Hit@K')
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    root = Path(args.db)
    kfs = load_db(root)
    bgr = cv2.imread(args.query, cv2.IMREAD_COLOR)
    if bgr is None:
        raise SystemExit(f'cannot read {args.query}')
    res = retrieve_topk(bgr, kfs, top_k=args.top_k)
    ranks = {c.keyframe_id: c.rank for c in res.candidates}
    hit = {}
    if args.gt_id:
        r = ranks.get(args.gt_id)
        hit = {
            'gt_id': args.gt_id,
            'rank': r,
            'hit@1': bool(r == 1),
            'hit@5': bool(r is not None and r <= 5),
            'hit@10': bool(r is not None and r <= 10),
        }
    report = {
        'query_features': res.query_features,
        'runtime_sec': res.runtime_sec,
        'candidates': [
            {
                'id': c.keyframe_id,
                'rank': c.rank,
                'raw_matches': c.raw_matches,
                'ratio_matches': c.ratio_matches,
                'score': c.score,
            }
            for c in res.candidates
        ],
        'hit': hit,
        'note': 'Retrieval only — no final pose from visual score',
    }
    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        Path(args.out).write_text(text, encoding='utf-8')


if __name__ == '__main__':
    main()
