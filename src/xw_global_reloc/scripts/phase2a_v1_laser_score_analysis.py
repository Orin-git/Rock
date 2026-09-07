#!/usr/bin/env python3
"""Offline laser-score distribution, threshold sweep, Top3 vs Top5, scene dig.

Uses laser_dryrun_debug_cluster.json (+ optional track_b hard-negatives).
Does not change ORB / Depth / search geometry / production defaults.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from xw_global_reloc.pose_cluster_accept import (
    ClusterMember,
    absolute_gate_member,
    decide_pose_clusters,
    decision_to_dict,
)

BENCH = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07')
IN_DEBUG = BENCH / 'laser_dryrun_debug_cluster.json'
IN_TRACKB = BENCH / 'track_b_visual_laser.json'
OUT = BENCH / 'laser_score_threshold_analysis.json'

CLUSTER_XY_M = 0.25
CLUSTER_YAW_RAD = math.radians(6.0)
CLUSTER_MIN_MARGIN = 0.03
MAX_TRANS = 1.2
MAX_YAW = math.radians(35.0)
FA_XY = 1.0
FA_YAW = 0.52
POC_XY = 0.25
POC_YAW = math.radians(5.0)
# Label refined pose as near-GT / correct place hypothesis
CORRECT_XY = 0.50
CORRECT_YAW = 0.35

THRESHOLDS = [0.30, 0.35, 0.38, 0.40, 0.42, 0.45, 0.48, 0.50]
UNKNOWN_FOCUS = {
    'kf_000013': 'corridor_wp2',
    'kf_000019': 'similar_corridor_wp3',
    'kf_000032': 'open_wp6',
}


def yaw_err(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def pct(arr: List[float], p: float) -> Optional[float]:
    if not arr:
        return None
    return float(np.percentile(arr, p))


def summarize(arr: List[float]) -> Dict[str, Any]:
    if not arr:
        return {'n': 0}
    a = np.asarray(arr, dtype=float)
    return {
        'n': int(a.size),
        'min': float(np.min(a)),
        'p10': float(np.percentile(a, 10)),
        'p50': float(np.percentile(a, 50)),
        'p90': float(np.percentile(a, 90)),
        'p95': float(np.percentile(a, 95)),
        'max': float(np.max(a)),
        'mean': float(np.mean(a)),
    }


def is_correct_pose(xy_err: float, yaw_e: float) -> bool:
    return xy_err <= CORRECT_XY and yaw_e <= CORRECT_YAW


def decide_at_threshold(trial: dict, thr: float, top_k: int) -> Dict[str, Any]:
    gt = (float(trial['gt_x']), float(trial['gt_y']), float(trial['gt_yaw']))
    members: List[ClusterMember] = []
    for r in (trial.get('visual_topk') or [])[:top_k]:
        seed = tuple(r['seed'])
        refined = tuple(r['laser_refined'])
        score = float(r['laser_score'])
        ok, reason, dx, dy, dyaw = absolute_gate_member(
            refined=refined,
            seed=seed,
            laser_score=score,
            min_laser_score=thr,
            max_refine_trans_m=MAX_TRANS,
            max_refine_yaw_rad=MAX_YAW,
            free_space=None if r.get('laser_reason') != 'occupied' else False,
            valid_beams=r.get('valid_beams'),
            min_valid_beams=20,
            legacy_reason='',
        )
        members.append(
            ClusterMember(
                keyframe_id=str(r['id']),
                refined=refined,
                seed=seed,
                laser_score=score,
                visual_rank=int(r.get('visual_rank') or 0),
                visual_score=float(r.get('visual_score') or 0.0),
                visual_region=str(r.get('visual_region') or ''),
                dx=dx,
                dy=dy,
                dyaw=dyaw,
                valid_beams=r.get('valid_beams'),
                matched_ratio=r.get('matched_ratio'),
                mean_dist=r.get('mean_dist'),
                p90_dist=r.get('p90_dist'),
                absolute_ok=ok,
                absolute_reason=reason,
            )
        )
    dec = decide_pose_clusters(
        members,
        cluster_xy_m=CLUSTER_XY_M,
        cluster_yaw_rad=CLUSTER_YAW_RAD,
        cluster_min_score_margin=CLUSTER_MIN_MARGIN,
    )
    refined = None
    xy = None
    ye = None
    fa = False
    poc = False
    if dec.best_cluster is not None:
        refined = list(dec.best_cluster.center)
        xy = math.hypot(refined[0] - gt[0], refined[1] - gt[1])
        ye = yaw_err(refined[2], gt[2])
        if dec.status == 'ACCEPT':
            if xy > FA_XY or ye > FA_YAW:
                fa = True
            if xy <= POC_XY and ye <= POC_YAW:
                poc = True
    return {
        'decision': dec.status,
        'reason': dec.reason,
        'false_accept': fa,
        'poc_quality_ok': poc,
        'position_error': xy,
        'yaw_error': ye,
        'cluster': decision_to_dict(dec),
    }


def main() -> None:
    debug = json.loads(IN_DEBUG.read_text(encoding='utf-8'))
    trials = debug.get('trials') or []

    correct_scores: List[float] = []
    wrong_scores: List[float] = []
    correct_rows: List[dict] = []
    wrong_rows: List[dict] = []
    by_scene: Dict[str, Dict[str, List[float]]] = {}

    unknown_detail = []

    for t in trials:
        gt_region = t.get('gt_region')
        scene = t.get('gt_class') or gt_region
        by_scene.setdefault(scene, {'correct': [], 'wrong': []})
        for r in t.get('visual_topk') or []:
            xy = float(r['position_error'])
            ye = float(r['yaw_error'])
            sc = float(r['laser_score'])
            row = {
                'query_id': t['query_id'],
                'gt_region': gt_region,
                'gt_class': t.get('gt_class'),
                'cand_id': r['id'],
                'visual_rank': r['visual_rank'],
                'visual_region': r['visual_region'],
                'visual_score': r['visual_score'],
                'seed': r['seed'],
                'refined': r['laser_refined'],
                'xy_err': xy,
                'yaw_err': ye,
                'laser_score': sc,
                'valid_beams': r.get('valid_beams'),
                'matched_ratio': r.get('matched_ratio'),
                'mean_dist': r.get('mean_dist'),
                'p90_dist': r.get('p90_dist'),
                'same_region': r['visual_region'] == gt_region,
            }
            if is_correct_pose(xy, ye):
                correct_scores.append(sc)
                correct_rows.append(row)
                by_scene[scene]['correct'].append(sc)
            else:
                wrong_scores.append(sc)
                wrong_rows.append(row)
                by_scene[scene]['wrong'].append(sc)

        if t['query_id'] in UNKNOWN_FOCUS or t.get('decision') == 'UNKNOWN':
            # best same-region / nearest-to-GT candidate
            rows = t.get('visual_topk') or []
            same = [r for r in rows if r['visual_region'] == gt_region]
            near = sorted(rows, key=lambda r: r['position_error'])
            pick = same[0] if same else (near[0] if near else None)
            # Prefer same-region with highest laser score
            if same:
                pick = max(same, key=lambda r: r['laser_score'])
            unknown_detail.append(
                {
                    'query_id': t['query_id'],
                    'gt_region': gt_region,
                    'gt_class': t.get('gt_class'),
                    'decision': t.get('decision'),
                    'best_same_or_nearest': None
                    if pick is None
                    else {
                        'id': pick['id'],
                        'visual_rank': pick['visual_rank'],
                        'visual_region': pick['visual_region'],
                        'visual_score': pick['visual_score'],
                        'seed': pick['seed'],
                        'refined': pick['laser_refined'],
                        'xy_err': pick['position_error'],
                        'yaw_err': pick['yaw_error'],
                        'yaw_err_deg': math.degrees(pick['yaw_error']),
                        'laser_score': pick['laser_score'],
                        'valid_beams': pick.get('valid_beams'),
                        'matched_ratio': pick.get('matched_ratio'),
                        'mean_dist': pick.get('mean_dist'),
                        'p90_dist': pick.get('p90_dist'),
                        'laser_reason': pick.get('laser_reason'),
                    },
                    'topk_scores': [
                        {
                            'id': r['id'],
                            'region': r['visual_region'],
                            'rank': r['visual_rank'],
                            'score': r['laser_score'],
                            'xy': r['position_error'],
                            'matched': r.get('matched_ratio'),
                            'mean': r.get('mean_dist'),
                            'p90': r.get('p90_dist'),
                            'beams': r.get('valid_beams'),
                        }
                        for r in rows
                    ],
                }
            )

    # Hard-negatives from track_b
    hard_neg = {}
    if IN_TRACKB.is_file():
        tb = json.loads(IN_TRACKB.read_text(encoding='utf-8'))
        for key in ('hard_negative_wrong_seed', 'hard_negative_shift_seed'):
            hn = tb.get(key) or {}
            hard_neg[key] = {
                'laser_ok': hn.get('laser_ok'),
                'laser_score': hn.get('laser_score'),
                'reason': hn.get('reason'),
                'pos_err_to_gt': hn.get('pos_err_to_gt'),
                'refined': hn.get('refined'),
                'seed': hn.get('seed'),
            }
            sc = hn.get('laser_score')
            if sc is not None:
                # treat as wrong unless very close
                pe = hn.get('pos_err_to_gt')
                if pe is None or pe > CORRECT_XY:
                    wrong_scores.append(float(sc))

    # Separation analysis
    csum = summarize(correct_scores)
    wsum = summarize(wrong_scores)
    sep = {
        'correct_max': csum.get('max'),
        'wrong_p95': wsum.get('p95'),
        'wrong_max': wsum.get('max'),
        'correct_p10': csum.get('p10'),
        'overlap': None,
        'safe_interval': None,
        'recommend_lower_threshold': False,
        'note': '',
    }
    if correct_scores and wrong_scores:
        # Overlap if wrong P95 >= correct P10 or wrong max >= correct min survivors near thr
        sep['overlap'] = float(wsum['p95']) >= float(csum['p10'])
        # Safe interval: thr in (wrong_p95, correct_p10] or at least FA=0 on sweep
        lo = float(wsum['p95'])
        hi = float(csum['p10'])
        if hi > lo:
            sep['safe_interval'] = [lo, hi]
            sep['note'] = 'correct P10 above wrong P95 → possible FA-safe thr in between'
        else:
            sep['safe_interval'] = None
            sep['note'] = 'correct/wrong score distributions overlap at P10/P95 — do not lower thr by guess'

    # Threshold + TopK sweeps
    thr_rows = []
    for thr in THRESHOLDS:
        for top_k in (3, 5):
            decisions = [decide_at_threshold(t, thr, top_k) for t in trials]
            n = len(decisions)
            fa = sum(1 for d in decisions if d['false_accept'])
            acc = sum(1 for d in decisions if d['decision'] == 'ACCEPT')
            unk = sum(1 for d in decisions if d['decision'] == 'UNKNOWN')
            correct = sum(1 for d in decisions if d['decision'] == 'ACCEPT' and not d['false_accept'])
            poc = sum(1 for d in decisions if d.get('poc_quality_ok'))
            thr_rows.append(
                {
                    'threshold': thr,
                    'top_k': top_k,
                    'ACCEPT': acc,
                    'UNKNOWN': unk,
                    'False_Accept': fa,
                    'Correct_Accept': correct,
                    'Correct_Accept_rate': correct / n if n else 0.0,
                    'poc_quality_accepts': poc,
                    'meets_debug_gate': fa == 0 and correct >= 8,
                }
            )

    # Recommend: FA=0 first, then max Correct Accept among FA=0, prefer thr<=0.45 for "lower"
    fa0 = [r for r in thr_rows if r['False_Accept'] == 0 and r['top_k'] == 5]
    best = None
    if fa0:
        best = sorted(fa0, key=lambda r: (-r['Correct_Accept'], -r['threshold']))[0]
    rec = {
        'recommend_change': False,
        'recommended_threshold': 0.45,
        'recommended_top_k_for_eval': 5,
        'reason': 'keep 0.45 until FA-safe ≥8/10 exists below 0.45',
    }
    if best and best['Correct_Accept'] >= 8 and best['threshold'] < 0.45:
        if not sep['overlap']:
            rec = {
                'recommend_change': True,
                'recommended_threshold': best['threshold'],
                'recommended_top_k_for_eval': 5,
                'reason': (
                    f"FA=0 and Correct Accept={best['Correct_Accept']}/10 at thr={best['threshold']}; "
                    'distributions support lowering'
                ),
            }
        else:
            rec = {
                'recommend_change': False,
                'recommended_threshold': 0.45,
                'recommended_top_k_for_eval': 5,
                'reason': (
                    f"thr={best['threshold']} reaches {best['Correct_Accept']}/10 FA=0 on this set, "
                    'but correct/wrong score overlap → do not lower; need score normalization'
                ),
                'candidate_threshold_on_this_set_only': best['threshold'],
            }
    elif best:
        rec = {
            'recommend_change': False,
            'recommended_threshold': 0.45,
            'recommended_top_k_for_eval': 5,
            'best_fa0_on_set': best,
            'reason': (
                f"best FA=0 thr on set is {best['threshold']} with Correct={best['Correct_Accept']}/10 "
                '(need ≥8 to justify lowering)'
            ),
        }

    # Scene diagnostics
    scene_report = {}
    for scene, packs in by_scene.items():
        scene_report[scene] = {
            'correct': summarize(packs['correct']),
            'wrong': summarize(packs['wrong']),
        }

    # Low-score cause proxies for UNKNOWN focus
    low_score_causes = []
    for u in unknown_detail:
        b = u.get('best_same_or_nearest') or {}
        cause = []
        beams = b.get('valid_beams')
        matched = b.get('matched_ratio')
        mean_d = b.get('mean_dist')
        p90 = b.get('p90_dist')
        if beams is not None and beams < 40:
            cause.append('few_usable_beams')
        if matched is not None and matched < 0.5:
            cause.append('low_matched_ratio')
        if mean_d is not None and mean_d > 0.4:
            cause.append('high_mean_endpoint_dist')
        if p90 is not None and p90 > 0.8:
            cause.append('high_p90_endpoint_dist')
        if u.get('gt_class') == 'open' or 'open' in str(u.get('gt_region')):
            cause.append('open_area_sparse_walls')
        if 'corridor' in str(u.get('gt_region')) or u.get('gt_class') in ('corridor', 'visual_similar'):
            cause.append('corridor_or_similar_structure')
        # score formula dilution: matched_ratio * clip(1 - mean/(4*0.25))
        if matched is not None and mean_d is not None:
            dil = float(np.clip(1.0 - mean_d / 1.0, 0.0, 1.0))
            cause.append(f'score_components matched={matched:.3f} dist_term={dil:.3f}')
        low_score_causes.append(
            {
                'query_id': u['query_id'],
                'gt_region': u['gt_region'],
                'hypotheses': cause,
                'laser_score': b.get('laser_score'),
                'matched_ratio': matched,
                'mean_dist': mean_d,
                'p90_dist': p90,
                'valid_beams': beams,
            }
        )

    # Top3 vs Top5 at 0.45
    top_cmp = {
        'at_0.45': {
            r['top_k']: r
            for r in thr_rows
            if abs(r['threshold'] - 0.45) < 1e-9
        }
    }

    report = {
        'source_debug': str(IN_DEBUG),
        'labeling': {
            'correct_pose': f'xy<={CORRECT_XY}m and yaw<={CORRECT_YAW}rad vs GT',
            'wrong_pose': 'otherwise (includes wrong region / far refine)',
        },
        'distribution': {
            'correct_cluster': csum,
            'wrong_cluster': wsum,
            'separation': sep,
        },
        'by_scene': scene_report,
        'unknown_focus_detail': unknown_detail,
        'low_score_scene_hypotheses': low_score_causes,
        'hard_negatives_track_b': hard_neg,
        'threshold_sweep': thr_rows,
        'top3_vs_top5': top_cmp,
        'recommendation': rec,
        'actual_topk_in_debug': 5,
    }
    OUT.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(
        {
            'correct': csum,
            'wrong': wsum,
            'separation': sep,
            'recommendation': rec,
            'sweep_fa0_top5': [r for r in thr_rows if r['False_Accept'] == 0 and r['top_k'] == 5],
            'unknown': [
                {
                    'q': u['query_id'],
                    'score': (u.get('best_same_or_nearest') or {}).get('laser_score'),
                    'matched': (u.get('best_same_or_nearest') or {}).get('matched_ratio'),
                    'mean': (u.get('best_same_or_nearest') or {}).get('mean_dist'),
                    'xy': (u.get('best_same_or_nearest') or {}).get('xy_err'),
                }
                for u in unknown_detail
            ],
            'out': str(OUT),
        },
        indent=2,
    ))


if __name__ == '__main__':
    main()
