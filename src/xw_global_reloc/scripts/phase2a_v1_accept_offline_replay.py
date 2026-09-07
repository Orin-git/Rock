#!/usr/bin/env python3
"""Offline replay of laser_dryrun_debug.json under pose-cluster ACCEPT policy.

No robot motion. Uses stored seed/refined/laser_score from prior dry-run.
Missing fields: valid_beams / matched_ratio / mean/P90 — not required for
absolute score+refine gates or SE(2) clustering; noted in output.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from xw_global_reloc.pose_cluster_accept import (
    ClusterMember,
    absolute_gate_member,
    decide_pose_clusters,
    decision_to_dict,
)

BENCH = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07')
IN_PATH = BENCH / 'laser_dryrun_debug.json'
OUT_PATH = BENCH / 'laser_accept_offline_replay.json'

CLUSTER_XY_M = 0.25
CLUSTER_YAW_RAD = math.radians(6.0)
CLUSTER_MIN_MARGIN = 0.03
MIN_LASER = 0.45
MAX_TRANS = 1.2
MAX_YAW = math.radians(35.0)
FA_XY = 1.0
FA_YAW = 0.52
POC_XY = 0.25
POC_YAW = math.radians(5.0)


def yaw_err(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def replay_trial(t: dict) -> dict:
    members = []
    for r in t.get('visual_topk') or []:
        seed = tuple(r['seed'])
        refined = tuple(r['laser_refined'])
        score = float(r['laser_score'])
        legacy = str(r.get('laser_reason') or '')
        abs_ok, abs_reason, dx, dy, dyaw = absolute_gate_member(
            refined=refined,
            seed=seed,
            laser_score=score,
            min_laser_score=MIN_LASER,
            max_refine_trans_m=MAX_TRANS,
            max_refine_yaw_rad=MAX_YAW,
            free_space=None if legacy != 'occupied' else False,
            valid_beams=r.get('valid_beams'),
            min_valid_beams=20,
            legacy_reason=legacy,
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
                absolute_ok=abs_ok,
                absolute_reason=abs_reason,
            )
        )
    dec = decide_pose_clusters(
        members,
        cluster_xy_m=CLUSTER_XY_M,
        cluster_yaw_rad=CLUSTER_YAW_RAD,
        cluster_min_score_margin=CLUSTER_MIN_MARGIN,
    )
    gt = (float(t['gt_x']), float(t['gt_y']), float(t['gt_yaw']))
    refined = None
    xy = None
    yaw = None
    lscore = None
    support = 0
    if dec.best_cluster is not None:
        refined = list(dec.best_cluster.center)
        xy = math.hypot(refined[0] - gt[0], refined[1] - gt[1])
        yaw = yaw_err(refined[2], gt[2])
        lscore = dec.best_cluster.best_laser_score
        support = dec.best_cluster.support_count
    fa = False
    poc_ok = False
    if dec.status == 'ACCEPT' and refined is not None:
        if xy > FA_XY or yaw > FA_YAW:
            fa = True
        if xy <= POC_XY and yaw <= POC_YAW:
            poc_ok = True
    return {
        'query_id': t.get('query_id'),
        'gt_region': t.get('gt_region'),
        'gt': list(gt),
        'old_decision': t.get('decision'),
        'old_reason': t.get('reason'),
        'new_decision': dec.status,
        'new_reason': dec.reason,
        'refined_pose': refined,
        'position_error': xy,
        'yaw_error': yaw,
        'laser_score': lscore,
        'cluster_count': len(dec.clusters),
        'cluster_support': support,
        'best_cluster_score': None if dec.best_cluster is None else dec.best_cluster.best_laser_score,
        'second_cluster_score': None
        if dec.second_cluster is None
        else dec.second_cluster.best_laser_score,
        'cluster_margin': dec.cluster_margin,
        'false_accept': fa,
        'poc_quality_ok': poc_ok,
        'cluster_accept': decision_to_dict(dec),
    }


def main() -> None:
    raw = json.loads(IN_PATH.read_text(encoding='utf-8'))
    trials = [replay_trial(t) for t in raw.get('trials') or []]
    n = len(trials)
    fa = sum(1 for t in trials if t['false_accept'])
    acc = sum(1 for t in trials if t['new_decision'] == 'ACCEPT')
    unk = sum(1 for t in trials if t['new_decision'] == 'UNKNOWN')
    correct = sum(1 for t in trials if t['new_decision'] == 'ACCEPT' and not t['false_accept'])
    poc = sum(1 for t in trials if t.get('poc_quality_ok'))
    flip = sum(1 for t in trials if t['old_decision'] != t['new_decision'])
    report = {
        'source': str(IN_PATH),
        'accept_policy': 'pose_cluster_v1',
        'missing_fields_note': (
            'valid_beams/matched_ratio/mean/P90 absent in prior debug JSON; '
            'absolute gate uses laser_score + refine dx/dy/dyaw + legacy reason. '
            'Sufficient for SE(2) clustering.'
        ),
        'params': {
            'cluster_xy_m': CLUSTER_XY_M,
            'cluster_yaw_rad': CLUSTER_YAW_RAD,
            'cluster_min_score_margin': CLUSTER_MIN_MARGIN,
            'min_laser_score': MIN_LASER,
        },
        'stats': {
            'n': n,
            'old_ACCEPT': sum(1 for t in trials if t['old_decision'] == 'ACCEPT'),
            'old_UNKNOWN': sum(1 for t in trials if t['old_decision'] == 'UNKNOWN'),
            'new_ACCEPT': acc,
            'new_UNKNOWN': unk,
            'Correct_Accept': correct,
            'False_Accept': fa,
            'poc_quality_accepts': poc,
            'decision_flips': flip,
            'Correct_Accept_rate': correct / n if n else 0.0,
            'UNKNOWN_rate': unk / n if n else 0.0,
        },
        'trials': trials,
    }
    OUT_PATH.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report['stats'], indent=2))
    for t in trials:
        print(
            json.dumps(
                {
                    'q': t['query_id'],
                    'old': t['old_decision'],
                    'new': t['new_decision'],
                    'xy': None if t['position_error'] is None else round(t['position_error'], 3),
                    'yaw_deg': None
                    if t['yaw_error'] is None
                    else round(math.degrees(t['yaw_error']), 2),
                    'support': t['cluster_support'],
                    'n_cl': t['cluster_count'],
                    'fa': t['false_accept'],
                    'poc': t['poc_quality_ok'],
                }
            )
        )
    print(f'wrote {OUT_PATH}')


if __name__ == '__main__':
    main()
