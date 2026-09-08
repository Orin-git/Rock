#!/usr/bin/env python3
"""Phase2C-C4A.4 offline laser regression.

Does not change threshold, weights, or the production scorer.
Replays recorded scan/map/candidate with:
  - current shared scorer (out-of-map endpoints dropped from the mean)
  - pre-fix scorer (1e3 OOB sentinel kept in the mean)
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_verify import (
    DistanceField,
    _score_from_dists,
    prepare_scan,
    score_scan_at_pose,
)
from xw_phase2c.charger_prior import load_charger_waypoint
from xw_phase2c.laser_prior_verify import verify_prior_in_window, verify_pose_with_laser

BENCH = Path('/ros2_ws/bench')
OUT = Path('/ros2_ws/bench/phase2c_c4a4_p1_safety_2026-09-08')
MAP_YAML = Path('/ros2_ws/maps/vp.yaml')
KF = Path('/ros2_ws/maps/vp/visual/keyframes')
FORMAL = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07/laser_dryrun_formal30_score038_fast.json')
C4A3 = Path('/ros2_ws/bench/phase2c_c4a3_charger_laser_2026-09-08')
THR = 0.38
MATCH = 0.25
MIN_BEAMS = 20
FA_XY = 1.0
FA_YAW = 0.52
CHARGER = (1.8663955491712294, -0.05958837147746455, -3.1286646850836126)
PRE = (1.7993268507431137, 0.01590625307226233, -2.8414570739649156)
WRONG = (-8.93, 1.60, 0.0)


def yaw_err(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def load_map_yaml(yaml_path: Path) -> OccupancyGrid:
    meta = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    import cv2

    pgm = cv2.imread(str(yaml_path.parent / meta['image']), cv2.IMREAD_UNCHANGED)
    if pgm is None:
        raise FileNotFoundError(meta['image'])
    if pgm.ndim == 3:
        pgm = cv2.cvtColor(pgm, cv2.COLOR_BGR2GRAY)
    img_u8 = np.flipud(np.asarray(pgm, dtype=np.uint8))
    h, w = img_u8.shape[:2]
    grid = OccupancyGrid()
    grid.info.resolution = float(meta['resolution'])
    grid.info.width = int(w)
    grid.info.height = int(h)
    grid.info.origin.position.x = float(meta['origin'][0])
    grid.info.origin.position.y = float(meta['origin'][1])
    negate = int(meta.get('negate', 0))
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.25))
    pix = img_u8.astype(np.float64)
    occ = (pix / 255.0) if negate else ((255.0 - pix) / 255.0)
    data = np.full(occ.shape, -1, dtype=np.int8)
    data[occ > occ_t] = 100
    data[occ < free_t] = 0
    grid.data = data.reshape(-1).astype(np.int8).tolist()
    return grid


def scan_from_npz(path: Path) -> LaserScan:
    z = np.load(str(path), allow_pickle=True)
    scan = LaserScan()
    scan.ranges = z['ranges'].astype(np.float32).tolist()
    scan.angle_min = float(z['angle_min'])
    scan.angle_max = float(z['angle_max'])
    scan.angle_increment = float(z['angle_increment'])
    scan.range_min = float(z['range_min'])
    scan.range_max = float(z['range_max'])
    return scan


def scan_from_json(path: Path) -> LaserScan:
    d = json.loads(path.read_text(encoding='utf-8'))
    scan = LaserScan()
    scan.header.frame_id = str(d.get('header', {}).get('frame_id') or 'lidar_link')
    scan.angle_min = float(d['angle_min'])
    scan.angle_max = float(d['angle_max'])
    scan.angle_increment = float(d['angle_increment'])
    scan.range_min = float(d['range_min'])
    scan.range_max = float(d['range_max'])
    scan.ranges = [float(r) for r in d['ranges']]
    return scan


def project(prep, x: float, y: float, yaw: float):
    lyaw = yaw + math.pi
    lc, ls = math.cos(lyaw), math.sin(lyaw)
    mx = x + lc * prep.bx - ls * prep.by
    my = y + ls * prep.bx + lc * prep.by
    return mx, my


def score_old(field: DistanceField, scan: LaserScan, x: float, y: float, yaw: float):
    """Pre-fix: 1e3 OOB sentinel stays in the mean."""
    prep = prepare_scan(scan, beam_stride=6)
    if prep.n_valid < MIN_BEAMS:
        return {
            'accepted': False,
            'reason': 'few_beams',
            'laser_score': 0.0,
            'matched_ratio': 0.0,
            'mean_dist': 999.0,
            'valid_beams': prep.n_valid,
            'in_map_beams': 0,
            'out_of_map_beams': 0,
        }
    mx, my = project(prep, x, y, yaw)
    dists = field.sample_dist_batch(mx, my)
    sc = _score_from_dists(
        dists,
        match_dist_m=MATCH,
        min_valid_beams=MIN_BEAMS,
        min_laser_score=THR,
        runtime_sec=0.0,
    )
    in_map = field.in_map_mask(mx, my)
    return {
        'accepted': bool(sc.accepted),
        'reason': sc.reason,
        'laser_score': float(sc.laser_score),
        'matched_ratio': float(sc.matched_ratio),
        'mean_dist': float(sc.mean_dist),
        'valid_beams': int(sc.valid_beams),
        'in_map_beams': int(np.count_nonzero(in_map)),
        'out_of_map_beams': int(np.count_nonzero(~in_map)),
    }


def account(field: DistanceField, scan: LaserScan, x: float, y: float, yaw: float) -> dict:
    """Beam ledger for the frozen new rule. Does not change the scorer."""
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    total = int(ranges.size)
    rmin = float(scan.range_min)
    rmax = float(scan.range_max)
    nan_inf = int(np.count_nonzero(~np.isfinite(ranges)))
    stride = 6
    idx = np.arange(0, total, stride, dtype=np.int32)
    r = ranges[idx]
    finite = np.isfinite(r)
    in_range = finite & (r >= rmin) & (r <= rmax)
    valid_n = int(np.count_nonzero(in_range))
    invalid_stride = int(idx.size - valid_n)
    prep = prepare_scan(scan, beam_stride=stride)
    mx, my = project(prep, x, y, yaw)
    in_map = field.in_map_mask(mx, my) if prep.n_valid else np.zeros(0, dtype=bool)
    in_map_n = int(np.count_nonzero(in_map))
    oob_n = int(prep.n_valid - in_map_n)
    new = score_scan_at_pose(
        field, scan, x, y, yaw, beam_stride=stride, match_dist_m=MATCH,
        min_valid_beams=MIN_BEAMS, min_laser_score=THR, prepared=prep,
    )
    matched = 0
    if in_map_n:
        dists = field.sample_dist_batch(mx[in_map], my[in_map])
        matched = int(np.count_nonzero(dists <= MATCH))
    old = score_old(field, scan, x, y, yaw)
    decision_new = 'ACCEPT' if new.accepted else 'REJECT'
    decision_old = 'ACCEPT' if old['accepted'] else 'REJECT'
    return {
        'pose': {'x': x, 'y': y, 'yaw': yaw},
        'total_beams': total,
        'nan_inf_beams': nan_inf,
        'stride': stride,
        'stride_candidates': int(idx.size),
        'valid_beams': valid_n,
        'invalid_or_nonfinite_after_stride': invalid_stride,
        'in_map_beams': in_map_n,
        'out_of_map_beams': oob_n,
        'matched_beams': matched,
        'matched_ratio': float(new.matched_ratio),
        'mean_dist': float(new.mean_dist),
        'laser_score': float(new.laser_score),
        'reason': new.reason,
        'coverage_gate': 'in_map_beams >= 20',
        'coverage_pass': in_map_n >= MIN_BEAMS,
        'matched_ratio_gate': THR * 0.8,
        'decision_new': decision_new,
        'old_score': old['laser_score'],
        'old_mean_dist': old['mean_dist'],
        'old_accepted': old['accepted'],
        'decision_old': decision_old,
        'crossed_up': (not old['accepted']) and bool(new.accepted),
    }


def far(pose, gt) -> bool:
    return math.hypot(pose[0] - gt[0], pose[1] - gt[1]) > FA_XY or yaw_err(pose[2], gt[2]) > FA_YAW


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    grid = load_map_yaml(MAP_YAML)
    field = DistanceField(grid)
    wp = load_charger_waypoint('/ros2_ws/maps', 'vp')
    assert wp is not None

    # --- C4A3 captured dock scan: raw vs refine ---
    dock = scan_from_json(C4A3 / 'scan.json')
    raw = account(field, dock, *CHARGER)
    pre = account(field, dock, *PRE)
    wrong = account(field, dock, *WRONG)
    win = verify_prior_in_window(CHARGER, dock, grid, min_score=THR, field=field)
    exact = verify_pose_with_laser(CHARGER, dock, grid, min_score=THR, field=field)
    refined_pose = win.get('pose') or {}
    seed = win.get('seed_pose') or {}
    delta = None
    if win.get('refined') and seed and refined_pose:
        delta = {
            'dx': float(refined_pose['x']) - float(seed['x']),
            'dy': float(refined_pose['y']) - float(seed['y']),
            'dyaw': float(math.atan2(
                math.sin(float(refined_pose['yaw']) - float(seed['yaw'])),
                math.cos(float(refined_pose['yaw']) - float(seed['yaw'])),
            )),
        }
    raw_refine = {
        'note': (
            'C4A3 capture scan: stored waypoint raw < 0.38, local 0.35m/20deg window '
            'finds a peak >= 0.38. Live BOOT_A P1 score 0.451 used the stored waypoint '
            'itself (stage candidate == waypoint, no refine). Different scan, not a hidden refine.'
        ),
        'stored_waypoint': CHARGER,
        'raw': raw,
        'exact_verify': {
            'ok': exact.get('ok'),
            'laser_score': exact.get('laser_score'),
            'refined': exact.get('refined', False),
        },
        'window': {
            'ok': win.get('ok'),
            'refined': win.get('refined'),
            'exact_laser_score': win.get('exact_laser_score'),
            'laser_score': win.get('laser_score'),
            'pose': refined_pose,
            'delta': delta,
        },
        'pre_scramble': pre,
        'wrong_corridor': wrong,
        'live_p1_recorded': {
            'file': 'phase2c_c4a3_charger_laser_2026-09-08/trial_BOOT_A_p1.json',
            'laser_score': 0.4514124363448962,
            'candidate_pose': {
                'x': CHARGER[0], 'y': CHARGER[1], 'yaw': CHARGER[2],
            },
            'refined': False,
            'source': 'stage pose equals stored charger waypoint; window not applied',
        },
    }

    # --- Formal30 recorded candidates ---
    formal = json.loads(FORMAL.read_text(encoding='utf-8'))
    rows = []
    crossings = []
    class_counts = {}
    for trial in formal['trials']:
        qid = trial['query_id']
        scan = scan_from_npz(KF / qid / 'scan.npz')
        gt = (float(trial['gt_x']), float(trial['gt_y']), float(trial['gt_yaw']))
        klass = str(trial.get('gt_class') or trial.get('gt_location'))
        class_counts[klass] = class_counts.get(klass, 0) + 1
        for cand in trial.get('visual_topk') or []:
            pose = cand.get('laser_refined') or cand.get('seed')
            if not pose:
                continue
            rec = account(field, scan, float(pose[0]), float(pose[1]), float(pose[2]))
            pos_err = float(cand.get('position_error') or 0.0)
            yerr = float(cand.get('yaw_error') or 0.0)
            wrong_cand = pos_err > FA_XY or yerr > FA_YAW
            row = {
                'set': 'formal30',
                'query_id': qid,
                'gt_class': klass,
                'gt_location': trial.get('gt_location'),
                'candidate_id': cand.get('id'),
                'recorded_score': cand.get('laser_score'),
                'recorded_ok': cand.get('laser_ok'),
                'position_error': pos_err,
                'yaw_error': yerr,
                'wrong_candidate': wrong_cand,
                'old_score': rec['old_score'],
                'new_score': rec['laser_score'],
                'decision_old': rec['decision_old'],
                'decision_new': rec['decision_new'],
                'out_of_map_beams': rec['out_of_map_beams'],
                'in_map_beams': rec['in_map_beams'],
                'matched_ratio': rec['matched_ratio'],
                'mean_dist': rec['mean_dist'],
            }
            rows.append(row)
            if rec['crossed_up']:
                crossings.append(row)

        # charger waypoint on this recorded scan
        ch = account(field, scan, *wp)
        ch_wrong = far(wp, gt)
        rows.append({
            'set': 'formal30_charger_waypoint',
            'query_id': qid,
            'gt_class': klass,
            'gt_location': trial.get('gt_location'),
            'candidate_id': 'stored_charger',
            'recorded_score': None,
            'recorded_ok': None,
            'position_error': math.hypot(wp[0] - gt[0], wp[1] - gt[1]),
            'yaw_error': yaw_err(wp[2], gt[2]),
            'wrong_candidate': ch_wrong,
            'old_score': ch['old_score'],
            'new_score': ch['laser_score'],
            'decision_old': ch['decision_old'],
            'decision_new': ch['decision_new'],
            'out_of_map_beams': ch['out_of_map_beams'],
            'in_map_beams': ch['in_map_beams'],
            'matched_ratio': ch['matched_ratio'],
            'mean_dist': ch['mean_dist'],
        })
        if ch['crossed_up'] and ch_wrong:
            crossings.append(rows[-1])

    # charger-room keyframes vs stored charger and vs wrong corridor
    charger_ds = []
    for item in json.loads(Path('/ros2_ws/maps/vp/visual/descriptors/index.json').read_text()):
        if item.get('region_id') != 'charger_room':
            continue
        scan = scan_from_npz(KF / item['id'] / 'scan.npz')
        pose = item['pose']
        gt = (float(pose['x']), float(pose['y']), float(pose['yaw']))
        for name, p in (('stored_charger', wp), ('own_pose', gt), ('wrong_corridor', WRONG)):
            rec = account(field, scan, *p)
            rec.update({
                'set': 'charger_dataset',
                'query_id': item['id'],
                'candidate': name,
                'gt': gt,
                'xy_to_candidate': math.hypot(p[0] - gt[0], p[1] - gt[1]),
            })
            charger_ds.append(rec)

    fa_new = [
        r for r in rows
        if r['wrong_candidate'] and r['decision_new'] == 'ACCEPT'
    ]
    crossed_wrong = [
        r for r in rows
        if r['wrong_candidate'] and r['decision_old'] == 'REJECT' and r['decision_new'] == 'ACCEPT'
    ]
    score_up = [
        r for r in rows
        if r['old_score'] < THR and r['new_score'] >= THR
    ]
    score_up_wrong = [r for r in score_up if r['wrong_candidate']]
    score_changed = [
        r for r in rows
        if abs(float(r['new_score']) - float(r['old_score'])) > 1e-9
    ]
    # samples by class for the report
    sample_keep = []
    seen = set()
    for r in rows:
        if r['set'] != 'formal30':
            continue
        key = r['gt_class']
        if key in seen:
            continue
        seen.add(key)
        sample_keep.append(r)

    summary = {
        'threshold': THR,
        'scorer': 'xw_global_reloc.laser_verify (shared; P1 window is extra, not used for this replay)',
        'formal30_trials': len(formal['trials']),
        'formal30_recorded_candidates': sum(1 for r in rows if r['set'] == 'formal30'),
        'formal30_classes': class_counts,
        'score_changed': len(score_changed),
        'score_crossed_0_38': len(score_up),
        'score_crossed_0_38_wrong': len(score_up_wrong),
        'score_crossed_0_38_wrong_rows': score_up_wrong[:20],
        'crossed_up_any': len(crossings),
        'crossed_up_wrong': len(crossed_wrong),
        'preexisting_far_accept_unchanged': len(fa_new),
        'false_accept_new': len(score_up_wrong),
        'false_accept_new_rows': fa_new[:20],
        'crossed_wrong_rows': crossed_wrong[:20],
        'class_samples': sample_keep,
        'gate': 'PASS' if not score_up_wrong else 'FAIL',
        'note': (
            'False Accept for this fix = wrong candidate whose score rose from <0.38 to >=0.38 '
            'after dropping out-of-map rays. Formal30 already had some far laser_ok rows; '
            'those scores are unchanged (old==new) and are not new accepts from this fix.'
        ),
    }
    payload = {
        'scoring_rule': {
            'enter_mean': [
                'finite scan ranges',
                'range_min <= range <= range_max',
                'beam_stride=6 survivors',
                'projected endpoint inside occupancy grid',
            ],
            'excluded_from_mean': [
                'NaN / Inf',
                'invalid range',
                'projected out-of-map rays (1e3 sentinel never enters mean)',
            ],
            'coverage_gate': 'in_map_beams >= 20 else reason=few_beams score=0',
            'score': 'matched_ratio * clip(1 - mean_dist/(0.25*4), 0, 1)',
            'accept': 'laser_score >= 0.38 AND matched_ratio >= 0.304',
            'threshold': 0.38,
        },
        'raw_vs_refine': raw_refine,
        'regression': summary,
        'charger_dataset': [
            {
                'query_id': r['query_id'],
                'candidate': r['candidate'],
                'laser_score': r['laser_score'],
                'old_score': r['old_score'],
                'decision_new': r['decision_new'],
                'decision_old': r['decision_old'],
                'in_map_beams': r['in_map_beams'],
                'out_of_map_beams': r['out_of_map_beams'],
                'matched_ratio': r['matched_ratio'],
                'mean_dist': r['mean_dist'],
                'xy_to_candidate': r['xy_to_candidate'],
            }
            for r in charger_ds
        ],
    }
    # P1 window on one recorded scan per class (not a global search).
    window_by_class = []
    seen_cls = set()
    for trial in formal['trials']:
        klass = str(trial.get('gt_class'))
        if klass in seen_cls:
            continue
        seen_cls.add(klass)
        scan = scan_from_npz(KF / trial['query_id'] / 'scan.npz')
        w = verify_prior_in_window(wp, scan, grid, min_score=THR, field=field)
        window_by_class.append({
            'query_id': trial['query_id'],
            'gt_class': klass,
            'gt_location': trial.get('gt_location'),
            'gt': [trial['gt_x'], trial['gt_y'], trial['gt_yaw']],
            'exact_score': w.get('exact_laser_score', w.get('laser_score')),
            'ok': w.get('ok'),
            'refined': w.get('refined'),
            'laser_score': w.get('laser_score'),
            'pose': w.get('pose'),
        })
    payload['p1_window_by_class'] = window_by_class
    payload['score_crossed_up_rows'] = [
        {k: r[k] for k in (
            'set', 'query_id', 'gt_class', 'candidate_id', 'old_score', 'new_score',
            'wrong_candidate', 'position_error',
        )}
        for r in score_up
    ]
    (OUT / 'offline_replay.json').write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')

    print(json.dumps({
        'gate': summary['gate'],
        'false_accept_new': summary['false_accept_new'],
        'score_crossed_0_38': summary['score_crossed_0_38'],
        'score_crossed_0_38_wrong': summary['score_crossed_0_38_wrong'],
        'crossed_up_wrong': summary['crossed_up_wrong'],
        'raw_score': raw['laser_score'],
        'window_score': win.get('laser_score'),
        'window_refined': win.get('refined'),
        'pre_score': pre['laser_score'],
        'wrong_score': wrong['laser_score'],
        'p1_window_by_class': [
            {k: w[k] for k in ('gt_class', 'gt_location', 'ok', 'laser_score', 'exact_score')}
            for w in window_by_class
        ],
    }, indent=2))
    return 0 if summary['gate'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
