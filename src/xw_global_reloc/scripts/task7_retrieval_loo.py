#!/usr/bin/env python3
"""Stage2 leave-one-out / cross-query retrieval evaluation (no Depth)."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from xw_global_reloc.retrieval import retrieve_topk


def load_kfs(root: Path):
    idx = json.loads((root / 'descriptors' / 'index.json').read_text(encoding='utf-8'))
    out = []
    for item in idx:
        kdir = root / 'keyframes' / item['id']
        meta = yaml.safe_load((kdir / 'meta.yaml').read_text(encoding='utf-8'))
        if not meta.get('retrieval_ready', True):
            continue
        desc = np.load(str(kdir / 'descriptors.npy'))
        rgb = cv2.imread(str(kdir / 'rgb.jpg'))
        out.append({'id': item['id'], 'descriptors': desc, 'meta': meta, 'rgb': rgb, 'pose': meta['map_pose']})
    return out


def region_of(pose, cells=1.5):
    return (round(pose['x'] / cells), round(pose['y'] / cells))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--db', required=True)
    ap.add_argument('--out', default='/ros2_ws/bench/phase2a_poc_v1_2026-09-07/task7_retrieval.json')
    ap.add_argument('--top-k', type=int, default=10)
    args = ap.parse_args()
    root = Path(args.db)
    kfs = load_kfs(root)
    if len(kfs) < 3:
        report = {'error': 'need>=3 retrieval_ready keyframes', 'n': len(kfs)}
        Path(args.out).write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        return

    trials = []
    hit1 = hit5 = hit10 = 0
    for i, q in enumerate(kfs):
        pool = [k for k in kfs if k['id'] != q['id']]
        res = retrieve_topk(q['rgb'], pool, top_k=args.top_k)
        gt_region = region_of(q['pose'])
        # GT hit if top candidate same spatial cell as query
        ranks_same = []
        for c in res.candidates:
            kp = next(k for k in pool if k['id'] == c.keyframe_id)
            if region_of(kp['pose']) == gt_region:
                ranks_same.append(c.rank)
        best = min(ranks_same) if ranks_same else None
        h1 = best == 1
        h5 = best is not None and best <= 5
        h10 = best is not None and best <= 10
        hit1 += int(h1)
        hit5 += int(h5)
        hit10 += int(h10)
        top = res.candidates[0] if res.candidates else None
        top5 = res.candidates[:5]
        margin = 0.0
        if len(res.candidates) >= 2:
            margin = res.candidates[0].score - res.candidates[1].score
        trials.append(
            {
                'query_id': q['id'],
                'gt_region': gt_region,
                'gt_pose': q['pose'],
                'top1': None if top is None else {'id': top.keyframe_id, 'score': top.score, 'matches': top.ratio_matches},
                'top5': [{'id': c.keyframe_id, 'score': c.score, 'matches': c.ratio_matches} for c in top5],
                'same_region_best_rank': best,
                'hit@1': h1,
                'hit@5': h5,
                'hit@10': h10,
                'score_margin': margin,
                'query_features': res.query_features,
            }
        )
    n = len(trials)
    report = {
        'n_queries': n,
        'Hit@1': hit1 / n if n else 0,
        'Hit@5': hit5 / n if n else 0,
        'Hit@10': hit10 / n if n else 0,
        'counts': {'hit1': hit1, 'hit5': hit5, 'hit10': hit10},
        'trials': trials,
        'note': 'Leave-one-out; GT=same spatial cell (~1.5m). Depth unused.',
    }
    Path(args.out).write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k: report[k] for k in report if k != 'trials'}, indent=2))
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
