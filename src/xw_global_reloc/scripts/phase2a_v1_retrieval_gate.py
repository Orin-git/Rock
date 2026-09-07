#!/usr/bin/env python3
"""Phase2A V1 Visual Retrieval Gate — multi-region leave-one-out.

No NPU. Depth unused. Hit = correct region_id in Top-K.
"""

from __future__ import annotations

import argparse
import json
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
        rgb = cv2.imread(str(kdir / 'rgb.jpg'))
        if rgb is None:
            continue
        out.append(
            {
                'id': item['id'],
                'descriptors': np.load(str(kdir / 'descriptors.npy')),
                'meta': meta,
                'rgb': rgb,
                'pose': meta['map_pose'],
                'region_id': meta.get('region_id') or item.get('region_id') or 'unknown',
                'region_class': meta.get('region_class', ''),
                'location_id': meta.get('location_id', ''),
            }
        )
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--db', default='/ros2_ws/maps/vp/visual')
    ap.add_argument(
        '--out',
        default='/ros2_ws/bench/phase2a_poc_v1_2026-09-07/retrieval_gate_v1.json',
    )
    ap.add_argument('--top-k', type=int, default=5)
    args = ap.parse_args()
    kfs = load_kfs(Path(args.db))
    if len(kfs) < 10:
        report = {'error': 'need>=10 retrieval_ready keyframes', 'n': len(kfs)}
        Path(args.out).write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        return

    trials = []
    hit1 = hit3 = hit5 = 0
    for q in kfs:
        pool = [k for k in kfs if k['id'] != q['id']]
        res = retrieve_topk(q['rgb'], pool, top_k=args.top_k)
        ranks = []
        topk = []
        for c in res.candidates:
            kf = next(k for k in pool if k['id'] == c.keyframe_id)
            topk.append(
                {
                    'id': c.keyframe_id,
                    'region_id': kf['region_id'],
                    'location_id': kf['location_id'],
                    'rank': c.rank,
                    'visual_score': c.score,
                    'match_count': c.ratio_matches,
                    'raw_matches': c.raw_matches,
                    'pose': kf['pose'],
                }
            )
            if kf['region_id'] == q['region_id']:
                ranks.append(c.rank)
        best = min(ranks) if ranks else None
        h1 = best == 1
        h3 = best is not None and best <= 3
        h5 = best is not None and best <= 5
        hit1 += int(h1)
        hit3 += int(h3)
        hit5 += int(h5)
        margin = 0.0
        if len(res.candidates) >= 2:
            margin = res.candidates[0].score - res.candidates[1].score
        trials.append(
            {
                'query_id': q['id'],
                'gt_region': q['region_id'],
                'gt_class': q['region_class'],
                'gt_location': q['location_id'],
                'gt_pose': q['pose'],
                'query_features': res.query_features,
                'runtime_sec': res.runtime_sec,
                'topk': topk,
                'same_region_best_rank': best,
                'hit@1': h1,
                'hit@3': h3,
                'hit@5': h5,
                'top1_top2_margin': margin,
            }
        )

    n = len(trials)
    hit5_rate = hit5 / n if n else 0.0
    # Proceed to laser E2E only if correct region stably enters Top5.
    proceed = hit5_rate >= 0.60 and n >= 20
    bottleneck = None
    if n < 20:
        bottleneck = 'insufficient_queries'
        proceed = False
    elif hit5_rate < 0.60:
        bottleneck = 'retrieval_hit@5'
        proceed = False
    report = {
        'n_queries': n,
        'n_db': len(kfs),
        'regions': sorted({k['region_id'] for k in kfs}),
        'Hit@1': hit1 / n if n else 0.0,
        'Hit@3': hit3 / n if n else 0.0,
        'Hit@5': hit5_rate,
        'counts': {'hit1': hit1, 'hit3': hit3, 'hit5': hit5},
        'proceed_laser_e2e': proceed,
        'bottleneck': bottleneck,
        'npu_descriptor': False,
        'note': 'LOO; GT=same region_id. Depth unused. No NPU.',
        'trials': trials,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=2), encoding='utf-8')
    summary = {k: report[k] for k in report if k != 'trials'}
    print(json.dumps(summary, indent=2))
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
