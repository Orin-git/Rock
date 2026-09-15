"""Phase2D Candidate validation + safe promote to next Active version.

C1 produced vp_visual_v1.1 from v1.0. C2+ bumps Active → next minor
(e.g. v1.1 → v1.2). Never mutates prior version trees in-place.
Laser gate frozen at 0.38. Reuses Phase2A retrieve_topk / laser dry-run policy.
"""

from __future__ import annotations

import json
import math
import shutil
import tempfile
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

import cv2
import numpy as np
import yaml

from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files
from xw_global_reloc.orb_utils import visual_difference
from xw_global_reloc.phase2d.config_loader import load_phase2d_config
from xw_global_reloc.phase2d.coverage_model import CoverageModel, FrameRef, build_coverage_model
from xw_global_reloc.phase2d.coverage_report import build_coverage_dict
from xw_global_reloc.phase2d.spatial import (
    parse_spatial_cell,
    spatial_cell_id,
    yaw_bin_index,
    yaw_delta_deg,
)
from xw_global_reloc.phase2d.version_store import (
    atomic_set_pointer,
    file_sha256,
    inventory_keyframe_hashes,
    load_visual_db_from_root,
    next_visual_version,
    read_pointer_version,
    validate_version_for_activate,
    version_dir,
    visual_root,
)
from xw_global_reloc.retrieval import retrieve_topk

MIN_LASER_SCORE = 0.38
FA_XY_M = 1.0
FA_YAW_RAD = 0.52
VERSION_V10 = 'vp_visual_v1.0'
VERSION_V11 = 'vp_visual_v1.1'


@dataclass
class CandRecord:
    id: str
    path: Path
    meta: Dict[str, Any]
    status: str = 'PENDING'  # VERIFIED / REJECTED_*
    reasons: List[str] = field(default_factory=list)
    value: Dict[str, Any] = field(default_factory=dict)
    promoted_id: Optional[str] = None

    @property
    def x(self) -> float:
        return float(self.meta.get('x') if self.meta.get('x') is not None else self.meta.get('map_pose', {}).get('x', 0.0))

    @property
    def y(self) -> float:
        return float(self.meta.get('y') if self.meta.get('y') is not None else self.meta.get('map_pose', {}).get('y', 0.0))

    @property
    def yaw(self) -> float:
        return float(self.meta.get('yaw') if self.meta.get('yaw') is not None else self.meta.get('map_pose', {}).get('yaw', 0.0))


def _pose_of(meta: Dict[str, Any]) -> Dict[str, float]:
    if isinstance(meta.get('map_pose'), dict):
        mp = meta['map_pose']
        return {'x': float(mp['x']), 'y': float(mp['y']), 'yaw': float(mp['yaw'])}
    return {
        'x': float(meta.get('x', 0.0)),
        'y': float(meta.get('y', 0.0)),
        'yaw': float(meta.get('yaw', 0.0)),
    }


def _load_db_frames(root: Path, *, require_scan: bool = False) -> List[Dict[str, Any]]:
    idx = json.loads((root / 'descriptors' / 'index.json').read_text(encoding='utf-8'))
    out: List[Dict[str, Any]] = []
    for item in idx:
        kid = str(item['id'])
        kdir = root / 'keyframes' / kid
        meta_p = kdir / 'meta.yaml'
        if not meta_p.is_file():
            continue
        meta = yaml.safe_load(meta_p.read_text(encoding='utf-8')) or {}
        if not meta.get('retrieval_ready', True):
            continue
        rgb = cv2.imread(str(kdir / 'rgb.jpg'))
        desc_p = kdir / 'descriptors.npy'
        if rgb is None or not desc_p.is_file():
            continue
        scan_p = kdir / 'scan.npz'
        if require_scan and not scan_p.is_file():
            continue
        pose = _pose_of(meta)
        region = str(meta.get('region_id') or meta.get('spatial_cell') or kid)
        out.append(
            {
                'id': kid,
                'descriptors': np.load(str(desc_p)),
                'meta': meta,
                'rgb': rgb,
                'scan': scan_p if scan_p.is_file() else None,
                'pose': pose,
                'region_id': region,
                'region_class': str(meta.get('region_class') or meta.get('source') or ''),
                'location_id': str(meta.get('location_id') or meta.get('spatial_cell') or kid),
                'spatial_cell': str(meta.get('spatial_cell') or spatial_cell_id(pose['x'], pose['y'], 1.0)),
                'yaw_bin': int(meta['yaw_bin']) if meta.get('yaw_bin') is not None else yaw_bin_index(pose['yaw'], 8),
                'dir': kdir,
            }
        )
    return out


def load_candidates(cand_root: Path) -> List[CandRecord]:
    idx_p = cand_root / 'index.json'
    ids: List[str] = []
    if idx_p.is_file():
        idx = json.loads(idx_p.read_text(encoding='utf-8'))
        for item in idx:
            ids.append(str(item['id'] if isinstance(item, dict) else item))
    else:
        ids = [p.name for p in sorted((cand_root / 'keyframes').glob('cand_*'))]
    out: List[CandRecord] = []
    for kid in ids:
        kdir = cand_root / 'keyframes' / kid
        meta_p = kdir / 'meta.yaml'
        if not meta_p.is_file():
            out.append(CandRecord(kid, kdir, {}, 'REJECTED_INVALID', ['missing_meta']))
            continue
        meta = yaml.safe_load(meta_p.read_text(encoding='utf-8')) or {}
        out.append(CandRecord(kid, kdir, meta))
    return out


def precheck_candidate(
    rec: CandRecord,
    *,
    map_name: str,
    map_hash: str,
) -> None:
    m = rec.meta
    reasons: List[str] = []
    if str(m.get('lifecycle') or '') != 'CANDIDATE':
        reasons.append('lifecycle')
    if str(m.get('map_name') or '') != map_name:
        reasons.append('map_name')
    if str(m.get('map_hash') or '') != map_hash:
        reasons.append('map_hash')
    if not bool(m.get('laser_verified')):
        reasons.append('laser_verified')
    try:
        if float(m.get('laser_score') or 0.0) < MIN_LASER_SCORE:
            reasons.append('laser_score')
    except (TypeError, ValueError):
        reasons.append('laser_score')
    # Pose gate history: cov + loc status + phase2c recorded at capture
    pose_ok = all(
        k in m for k in ('amcl_cov_xy', 'amcl_cov_yaw', 'localization_status', 'phase2c_state')
    )
    if not pose_ok:
        reasons.append('pose_gate_history')
    iq = m.get('image_quality')
    if not isinstance(iq, dict) or not all(k in iq for k in ('sharpness', 'brightness', 'orb_features')):
        reasons.append('image_gate_history')
    for fname in ('descriptors.npy', 'keypoints.npy', 'rgb.jpg'):
        if not (rec.path / fname).is_file():
            reasons.append(f'missing_{fname}')
    if m.get('spatial_cell') in (None, ''):
        reasons.append('spatial_cell')
    if m.get('yaw_bin') is None:
        reasons.append('yaw_bin')
    if not str(m.get('source') or ''):
        reasons.append('source')
    if not str(m.get('build_session_id') or ''):
        reasons.append('build_session_id')
    # file integrity
    try:
        if (rec.path / 'descriptors.npy').is_file():
            d = np.load(str(rec.path / 'descriptors.npy'))
            if d is None or len(d) < 10:
                reasons.append('descriptors_corrupt')
        rgb = cv2.imread(str(rec.path / 'rgb.jpg'))
        if rgb is None:
            reasons.append('rgb_corrupt')
    except Exception:  # noqa: BLE001
        reasons.append('file_corrupt')

    if reasons:
        rec.status = 'REJECTED_INVALID'
        rec.reasons = reasons


def batch_dedup_candidates(
    recs: Sequence[CandRecord],
    cfg: Dict[str, Any],
) -> None:
    """Among still-PENDING candidates, drop near-duplicates keeping higher quality."""
    dd = dict(cfg.get('dedup') or {})
    max_xy = float(dd.get('max_xy_m', 0.35))
    max_yaw = float(dd.get('max_yaw_deg', 15.0))
    max_vdiff = float(dd.get('max_visual_diff_for_duplicate', 0.25))

    pending = [r for r in recs if r.status == 'PENDING']

    def quality(r: CandRecord) -> Tuple[float, float, float]:
        iq = r.meta.get('image_quality') or {}
        return (
            float(r.meta.get('laser_score') or 0.0),
            float(iq.get('sharpness') or 0.0),
            float(iq.get('orb_features') or 0.0),
        )

    # Prefer higher laser / sharpness; process best-first so losers get rejected
    pending.sort(key=quality, reverse=True)
    kept: List[CandRecord] = []
    descs: Dict[str, np.ndarray] = {}
    for r in pending:
        desc = np.load(str(r.path / 'descriptors.npy'))
        descs[r.id] = desc
        dup_of = None
        for k in kept:
            dist = math.hypot(r.x - k.x, r.y - k.y)
            dyaw = yaw_delta_deg(r.yaw, k.yaw)
            if dist > max_xy or dyaw > max_yaw:
                continue
            vdiff = visual_difference(desc, descs[k.id])
            if vdiff <= max_vdiff:
                dup_of = k.id
                break
        if dup_of:
            r.status = 'REJECTED_DUPLICATE'
            r.reasons = [f'dup_of:{dup_of}']
        else:
            kept.append(r)


def _active_cells_yaw(active_root: Path, cell_size: float = 1.0, yaw_bins: int = 8) -> Tuple[Set[str], Set[Tuple[str, int]]]:
    cells: Set[str] = set()
    yaw_occ: Set[Tuple[str, int]] = set()
    for fr in _load_db_frames(active_root):
        pose = fr['pose']
        cell = spatial_cell_id(pose['x'], pose['y'], cell_size)
        yb = yaw_bin_index(pose['yaw'], yaw_bins)
        cells.add(cell)
        yaw_occ.add((cell, yb))
    return cells, yaw_occ


def assess_value(
    rec: CandRecord,
    *,
    active_cells: Set[str],
    active_yaw: Set[Tuple[str, int]],
    active_descs_near: List[Tuple[str, np.ndarray, float, float, float]],
    cfg: Dict[str, Any],
) -> None:
    cell = str(rec.meta.get('spatial_cell') or '')
    yb = int(rec.meta.get('yaw_bin'))
    new_cell = cell not in active_cells
    new_yaw = (cell, yb) not in active_yaw
    desc = np.load(str(rec.path / 'descriptors.npy'))
    dd = dict(cfg.get('dedup') or {})
    max_xy = float(dd.get('max_xy_m', 0.35)) * 3.0
    max_yaw = float(dd.get('max_yaw_deg', 15.0)) * 2.0
    min_novel = float(cfg.get('capture', {}).get('min_visual_diff', 0.30))
    nearest_vdiff = 1.0
    for _id, ad, ax, ay, ayaw in active_descs_near:
        if math.hypot(rec.x - ax, rec.y - ay) > max_xy:
            continue
        if yaw_delta_deg(rec.yaw, ayaw) > max_yaw:
            continue
        nearest_vdiff = min(nearest_vdiff, visual_difference(desc, ad))
    novel = nearest_vdiff >= min_novel or nearest_vdiff == 1.0
    rec.value = {
        'new_spatial_cell': new_cell,
        'new_yaw_bin': new_yaw,
        'visual_novel': novel,
        'nearest_visual_diff': float(nearest_vdiff),
        'laser_score': float(rec.meta.get('laser_score') or 0.0),
    }
    if not new_cell and not new_yaw and not novel:
        rec.status = 'REJECTED_NO_VALUE'
        rec.reasons = ['no_coverage_no_novelty']


def build_temp_eval_db(
    active_root: Path,
    candidates: Sequence[CandRecord],
    dest: Path,
) -> Path:
    if dest.exists():
        shutil.rmtree(dest)
    shutil.copytree(active_root, dest, symlinks=False)
    idx_path = dest / 'descriptors' / 'index.json'
    idx = json.loads(idx_path.read_text(encoding='utf-8'))
    existing = {str(i['id']) for i in idx}
    for rec in candidates:
        if rec.status != 'PENDING':
            continue
        # Keep cand_ id in temp eval only (retrieval scripts load by index; Reloc loader skips cand_)
        kid = rec.id
        if kid in existing:
            continue
        kdir = dest / 'keyframes' / kid
        shutil.copytree(rec.path, kdir)
        # Normalize meta for retrieval GT
        meta = dict(rec.meta)
        meta['map_pose'] = {'x': rec.x, 'y': rec.y, 'yaw': rec.yaw}
        meta['region_id'] = str(meta.get('spatial_cell') or kid)
        meta['region_class'] = 'auto_patrol_candidate'
        meta['location_id'] = f"{meta.get('spatial_cell')}_yb{meta.get('yaw_bin')}"
        meta['retrieval_ready'] = True
        meta['lifecycle'] = 'EVAL_CANDIDATE'
        (kdir / 'meta.yaml').write_text(yaml.safe_dump(meta, sort_keys=False), encoding='utf-8')
        idx.append(
            {
                'id': kid,
                'pose': meta['map_pose'],
                'region_id': meta['region_id'],
                'location_id': meta['location_id'],
                'spatial_cell': meta.get('spatial_cell'),
                'yaw_bin': meta.get('yaw_bin'),
                'lifecycle': 'EVAL_CANDIDATE',
            }
        )
        existing.add(kid)
    idx_path.write_text(json.dumps(idx, indent=2) + '\n', encoding='utf-8')
    man = yaml.safe_load((dest / 'manifest.yaml').read_text(encoding='utf-8')) or {}
    man['version'] = 'temp_eval'
    man['lifecycle'] = 'EVAL'
    man['keyframe_count'] = len(idx)
    (dest / 'manifest.yaml').write_text(yaml.safe_dump(man, sort_keys=False), encoding='utf-8')
    return dest


def retrieval_loo_metrics(frames: Sequence[Dict[str, Any]], top_k: int = 5) -> Dict[str, Any]:
    """LOO Hit@K by region_id (Phase2A convention)."""
    trials = []
    hit1 = hit3 = hit5 = 0
    for q in frames:
        pool = [k for k in frames if k['id'] != q['id']]
        if not pool:
            continue
        res = retrieve_topk(q['rgb'], pool, top_k=top_k)
        ranks = []
        topk = []
        for c in res.candidates:
            kf = next(k for k in pool if k['id'] == c.keyframe_id)
            topk.append(
                {
                    'id': c.keyframe_id,
                    'region_id': kf['region_id'],
                    'spatial_cell': kf.get('spatial_cell'),
                    'rank': c.rank,
                    'visual_score': c.score,
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
        trials.append(
            {
                'query_id': q['id'],
                'gt_region': q['region_id'],
                'same_region_best_rank': best,
                'hit@1': h1,
                'hit@3': h3,
                'hit@5': h5,
                'topk': topk,
            }
        )
    n = len(trials)
    return {
        'n_queries': n,
        'n_db': len(frames),
        'Hit@1': hit1 / n if n else 0.0,
        'Hit@3': hit3 / n if n else 0.0,
        'Hit@5': hit5 / n if n else 0.0,
        'counts': {'hit1': hit1, 'hit3': hit3, 'hit5': hit5},
        'trials': trials,
    }


def candidate_loo_check(
    cand: CandRecord,
    eval_frames: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    """Leave-one-out: can neighbors support this candidate's cell without self?"""
    q = next((f for f in eval_frames if f['id'] == cand.id), None)
    if q is None:
        return {'ok': False, 'reason': 'not_in_eval'}
    pool = [k for k in eval_frames if k['id'] != cand.id]
    res = retrieve_topk(q['rgb'], pool, top_k=5)
    cell = str(cand.meta.get('spatial_cell') or '')
    try:
        cx, cy = parse_spatial_cell(cell)
    except ValueError:
        cx = cy = None
    neighbor_hits = []
    self_only_risk = True
    for c in res.candidates:
        kf = next(k for k in pool if k['id'] == c.keyframe_id)
        sc = str(kf.get('spatial_cell') or '')
        near = sc == cell
        if cx is not None and sc.startswith('cell_'):
            try:
                ox, oy = parse_spatial_cell(sc)
                near = near or (abs(ox - cx) + abs(oy - cy) <= 1)
            except ValueError:
                pass
        neighbor_hits.append(
            {
                'id': kf['id'],
                'spatial_cell': sc,
                'rank': c.rank,
                'visual_score': c.score,
                'near_cell': near,
                'pose_dist_m': math.hypot(kf['pose']['x'] - q['pose']['x'], kf['pose']['y'] - q['pose']['y']),
            }
        )
        if near:
            self_only_risk = False
    return {
        'ok': not self_only_risk,
        'self_only_risk': self_only_risk,
        'topk': neighbor_hits,
        'reason': 'self_only' if self_only_risk else 'neighbor_support',
    }


def confusion_analysis(
    frames: Sequence[Dict[str, Any]],
    focus_classes: Sequence[str] = ('corridor', 'visual_similar', 'doorway', 'open'),
    top_k: int = 5,
) -> Dict[str, Any]:
    """Similar-place confusion: Top-K cells far from GT but high visual score."""
    cases = []
    risky = 0
    for q in frames:
        cls = str(q.get('region_class') or '')
        if focus_classes and cls and not any(f in cls or f in str(q.get('region_id') or '') for f in focus_classes):
            # Also include all queries for legacy coverage when class empty (candidates)
            if cls not in ('', 'auto_patrol_candidate') and 'corridor' not in str(q.get('region_id')):
                if not any(f in str(q.get('region_id') or '') for f in focus_classes):
                    continue
        pool = [k for k in frames if k['id'] != q['id']]
        res = retrieve_topk(q['rgb'], pool, top_k=top_k)
        topk = []
        confused = False
        for c in res.candidates:
            kf = next(k for k in pool if k['id'] == c.keyframe_id)
            dist = math.hypot(kf['pose']['x'] - q['pose']['x'], kf['pose']['y'] - q['pose']['y'])
            same_region = kf['region_id'] == q['region_id']
            row = {
                'id': kf['id'],
                'region_id': kf['region_id'],
                'spatial_cell': kf.get('spatial_cell'),
                'rank': c.rank,
                'visual_score': c.score,
                'distance_m': dist,
                'same_region': same_region,
            }
            topk.append(row)
            # Dangerous if far (>2m) but ranked in Top-3 with high score
            if (not same_region) and dist > 2.0 and c.rank <= 3 and c.score >= 0.15:
                confused = True
        if confused:
            risky += 1
        cases.append(
            {
                'query_id': q['id'],
                'correct_cell': q.get('spatial_cell'),
                'gt_region': q['region_id'],
                'gt_class': q.get('region_class'),
                'topk': topk,
                'visual_confusion': confused,
            }
        )
    return {
        'n_cases': len(cases),
        'visual_confusion_cases': risky,
        'note': 'Visual similarity allowed; Laser Gate must reject wrong poses (see E4).',
        'cases': cases,
    }


def _load_map_field(map_yaml: Path):
    from nav_msgs.msg import OccupancyGrid
    from xw_global_reloc.laser_verify import DistanceField

    meta = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    img = map_yaml.parent / meta['image']
    pgm = cv2.imread(str(img), cv2.IMREAD_UNCHANGED)
    if pgm is None:
        raise FileNotFoundError(img)
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
    return DistanceField(grid)


def _load_scan(path: Path):
    from sensor_msgs.msg import LaserScan

    z = np.load(str(path), allow_pickle=True)
    scan = LaserScan()
    scan.ranges = z['ranges'].astype(np.float32).tolist()
    scan.angle_min = float(z['angle_min'])
    scan.angle_max = float(z['angle_max'])
    scan.angle_increment = float(z['angle_increment'])
    scan.range_min = float(z['range_min'])
    scan.range_max = float(z['range_max'])
    return scan


def visual_laser_fa_validation(
    db_root: Path,
    map_yaml: Path,
    *,
    query_ids: Optional[Sequence[str]] = None,
    top_k: int = 5,
    max_queries: int = 30,
) -> Dict[str, Any]:
    """Formal-style Visual→TopK→Laser FA check. Threshold frozen 0.38."""
    from xw_global_reloc.laser_refine import refine_candidate_with_laser
    from xw_global_reloc.laser_verify import prepare_scan
    from xw_global_reloc.pose_cluster_accept import (
        ClusterMember,
        absolute_gate_member,
        decide_pose_clusters,
    )
    from xw_global_reloc.transforms import Pose2D

    frames = _load_db_frames(db_root, require_scan=True)
    # Prefer legacy Formal30 locations when present
    by_loc: Dict[str, List[Dict[str, Any]]] = {}
    for k in frames:
        if str(k['id']).startswith('cand_'):
            continue
        by_loc.setdefault(k['location_id'] or k['id'], []).append(k)
    if query_ids:
        queries = [k for k in frames if k['id'] in set(query_ids)]
    else:
        # formal-ish: up to 10 locs × 3
        loc_ids = sorted(by_loc.keys())
        prefer = [
            'doorway',
            'corridor_wp2',
            'similar',
            'room_wp5',
            'open',
            'charger',
        ]

        def rank(lid: str) -> tuple:
            pri = 99
            for i, key in enumerate(prefer):
                if key in lid:
                    pri = i
                    break
            return (pri, lid)

        loc_ids = sorted(loc_ids, key=rank)[:10]
        queries = []
        for lid in loc_ids:
            queries.extend(by_loc[lid][:3])
        queries = queries[:max_queries]

    field = _load_map_field(map_yaml)
    trials = []
    fa = 0
    for q in queries:
        pool = [k for k in frames if k['id'] != q['id']]
        retr = retrieve_topk(q['rgb'], pool, top_k=top_k)
        scan = _load_scan(q['scan'])
        prepared = prepare_scan(scan, beam_stride=6)
        gt = Pose2D(q['pose']['x'], q['pose']['y'], q['pose']['yaw'])
        members: List[ClusterMember] = []
        rows = []
        for c in retr.candidates:
            kf = next(k for k in pool if k['id'] == c.keyframe_id)
            seed = Pose2D(kf['pose']['x'], kf['pose']['y'], kf['pose']['yaw'])
            ref = refine_candidate_with_laser(
                field,
                scan,
                seed,
                coarse_xy_m=1.0,
                coarse_yaw_rad=math.radians(30),
                coarse_xy_step=0.10,
                coarse_yaw_step=math.radians(3.0),
                fine_xy_m=0.20,
                fine_yaw_rad=math.radians(5.0),
                fine_xy_step=0.05,
                fine_yaw_step=math.radians(1.0),
                beam_stride=6,
                match_dist_m=0.25,
                min_valid_beams=20,
                min_laser_score=MIN_LASER_SCORE,
                min_margin=0.03,
                max_refine_trans_m=1.2,
                max_refine_yaw_rad=math.radians(35.0),
                top_n_coarse=5,
                reject_local_grid_margin=False,
                prepared=prepared,
            )
            refined_t = ref.refined.as_tuple()
            seed_t = seed.as_tuple()
            abs_ok, abs_reason, dx, dy, dyaw = absolute_gate_member(
                refined=refined_t,
                seed=seed_t,
                laser_score=ref.top1_score,
                min_laser_score=MIN_LASER_SCORE,
                max_refine_trans_m=1.2,
                max_refine_yaw_rad=math.radians(35.0),
                free_space=field.is_free(ref.refined.x, ref.refined.y),
                valid_beams=ref.score.valid_beams,
                min_valid_beams=20,
                legacy_reason=ref.reason,
            )
            if not ref.accepted and ref.reason != 'ambiguous_margin':
                abs_ok = False
                abs_reason = ref.reason
            rows.append(
                {
                    'id': c.keyframe_id,
                    'visual_rank': c.rank,
                    'visual_score': c.score,
                    'laser_score': ref.top1_score,
                    'laser_ok': abs_ok,
                    'laser_reason': abs_reason,
                    'region_id': kf['region_id'],
                    'is_candidate': str(c.keyframe_id).startswith('cand_'),
                }
            )
            members.append(
                ClusterMember(
                    keyframe_id=c.keyframe_id,
                    refined=refined_t,
                    seed=seed_t,
                    laser_score=float(ref.top1_score),
                    visual_rank=int(c.rank),
                    visual_score=float(c.score),
                    visual_region=str(kf['region_id']),
                    dx=dx,
                    dy=dy,
                    dyaw=dyaw,
                    valid_beams=ref.score.valid_beams,
                    matched_ratio=ref.score.matched_ratio,
                    mean_dist=ref.score.mean_dist,
                    p90_dist=ref.score.p90_dist,
                    absolute_ok=abs_ok,
                    absolute_reason=abs_reason,
                )
            )
        dec = decide_pose_clusters(
            members,
            cluster_xy_m=0.25,
            cluster_yaw_rad=math.radians(6.0),
            cluster_min_score_margin=0.03,
        )
        decision = dec.status
        refined_pose = None
        pos_err = None
        yerr = None
        false_accept = False
        if dec.best_cluster is not None:
            refined_pose = list(dec.best_cluster.center)
            pos_err = math.hypot(refined_pose[0] - gt.x, refined_pose[1] - gt.y)
            yerr = abs(math.atan2(math.sin(refined_pose[2] - gt.yaw), math.cos(refined_pose[2] - gt.yaw)))
            if decision == 'ACCEPT' and (pos_err > FA_XY_M or yerr > FA_YAW_RAD):
                false_accept = True
                fa += 1
        trials.append(
            {
                'query_id': q['id'],
                'gt_region': q['region_id'],
                'decision': decision,
                'false_accept': false_accept,
                'position_error': pos_err,
                'yaw_error': yerr,
                'visual_topk': rows,
            }
        )
        if false_accept:
            break  # hard stop

    return {
        'n_queries': len(trials),
        'false_accept_count': fa,
        'False_Accept': fa,
        'ACCEPT': sum(1 for t in trials if t['decision'] == 'ACCEPT'),
        'UNKNOWN': sum(1 for t in trials if t['decision'] == 'UNKNOWN'),
        'min_laser_score': MIN_LASER_SCORE,
        'stopped_on_fa': fa > 0,
        'trials': trials,
    }


def coverage_snapshot_from_db(db_root: Path, cfg: Dict[str, Any]) -> Dict[str, Any]:
    """Compute coverage stats from a version tree (active-style layout)."""
    frames = _load_db_frames(db_root)
    cell_size = float((cfg.get('coverage') or {}).get('cell_size_m', 1.0))
    yaw_bins = int((cfg.get('coverage') or {}).get('yaw_bins', 8))
    cells: Set[str] = set()
    yaw_occ: Set[Tuple[str, int]] = set()
    for fr in frames:
        pose = fr['pose']
        cell = spatial_cell_id(pose['x'], pose['y'], cell_size)
        yb = yaw_bin_index(pose['yaw'], yaw_bins)
        cells.add(cell)
        yaw_occ.add((cell, yb))
    return {
        'frames': len(frames),
        'cells': len(cells),
        'yaw_occupancy': len(yaw_occ),
        'yaw_coverage_ratio': len(yaw_occ) / max(len(cells) * yaw_bins, 1),
        'cell_ids': sorted(cells),
    }


def _candidate_already_decided(meta: Dict[str, Any]) -> Optional[str]:
    """Return skip status if candidate was already promoted/rejected."""
    if meta.get('promoted_as') or meta.get('version_promoted_to'):
        return 'ALREADY_PROMOTED'
    val = meta.get('validation')
    if isinstance(val, dict):
        st = str(val.get('status') or '')
        if st == 'PROMOTED':
            return 'ALREADY_PROMOTED'
        if st == 'VERIFIED' and (meta.get('promoted_as') or meta.get('version_promoted_to')):
            return 'ALREADY_PROMOTED'
        if st.startswith('REJECTED'):
            return st
    return None


def filter_candidates_for_promote(
    recs: Sequence[CandRecord],
    *,
    build_session_id: Optional[str] = None,
) -> List[CandRecord]:
    """Keep only fresh PENDING candidates (optionally session-scoped)."""
    kept: List[CandRecord] = []
    for r in recs:
        decided = _candidate_already_decided(r.meta)
        if decided:
            r.status = decided
            continue
        if build_session_id and str(r.meta.get('build_session_id') or '') != str(build_session_id):
            r.status = 'SKIPPED_OTHER_SESSION'
            continue
        kept.append(r)
    return kept


def build_promoted_version(
    *,
    vroot: Path,
    active_root: Path,
    verified: Sequence[CandRecord],
    map_hash: str,
    validation_report: Dict[str, Any],
    cfg: Dict[str, Any],
    previous_version: str,
    target_version: str,
    phase: str = 'Phase2D-C2',
) -> Path:
    dest = version_dir(vroot, target_version)
    if dest.exists():
        shutil.rmtree(dest)
    shutil.copytree(active_root, dest, symlinks=False)

    idx_path = dest / 'descriptors' / 'index.json'
    idx = json.loads(idx_path.read_text(encoding='utf-8'))
    nums = []
    for item in idx:
        kid = str(item['id'])
        if kid.startswith('kf_'):
            try:
                nums.append(int(kid.split('_')[1]))
            except ValueError:
                pass
    next_n = max(nums) + 1 if nums else 1

    promoted = []
    for rec in verified:
        new_id = f'kf_{next_n:06d}'
        next_n += 1
        kdir = dest / 'keyframes' / new_id
        shutil.copytree(rec.path, kdir)
        meta = dict(rec.meta)
        meta['keyframe_id'] = new_id
        meta['lifecycle'] = 'ACTIVE'
        meta['map_pose'] = {'x': rec.x, 'y': rec.y, 'yaw': rec.yaw}
        meta['region_id'] = str(meta.get('spatial_cell') or new_id)
        meta['region_class'] = 'auto_patrol'
        meta['location_id'] = f"{meta.get('spatial_cell')}_yb{meta.get('yaw_bin')}"
        meta['retrieval_ready'] = True
        meta['geometry_ready'] = (kdir / 'depth.png').is_file()
        meta['promoted_from'] = rec.id
        meta['validation'] = {'status': 'PROMOTED', 'phase': phase}
        meta['version_promoted_to'] = target_version
        (kdir / 'meta.yaml').write_text(yaml.safe_dump(meta, sort_keys=False), encoding='utf-8')
        idx.append(
            {
                'id': new_id,
                'pose': meta['map_pose'],
                'region_id': meta['region_id'],
                'location_id': meta['location_id'],
                'region_class': meta['region_class'],
                'spatial_cell': meta.get('spatial_cell'),
                'yaw_bin': meta.get('yaw_bin'),
                'promoted_from': rec.id,
            }
        )
        rec.promoted_id = new_id
        promoted.append({'candidate_id': rec.id, 'active_id': new_id})
        try:
            cm = dict(rec.meta)
            cm['validation'] = {'status': 'PROMOTED', 'phase': phase}
            cm['version_promoted_to'] = target_version
            cm['promoted_as'] = new_id
            (rec.path / 'meta.yaml').write_text(yaml.safe_dump(cm, sort_keys=False), encoding='utf-8')
        except Exception:  # noqa: BLE001
            pass

    idx_path.write_text(json.dumps(idx, indent=2) + '\n', encoding='utf-8')
    cov = coverage_snapshot_from_db(dest, cfg)
    man = yaml.safe_load((dest / 'manifest.yaml').read_text(encoding='utf-8')) or {}
    man.update(
        {
            'schema_version': 3,
            'version': target_version,
            'previous_version': previous_version,
            'map_name': str(man.get('map_name') or 'vp'),
            'map_hash': map_hash,
            'keyframe_count': len(idx),
            'candidate_promoted_count': len(promoted),
            'spatial_cells': cov['cells'],
            'yaw_coverage_ratio': cov['yaw_coverage_ratio'],
            'appearance_count': len(idx),
            'validation_status': 'PASSED',
            'false_accept_count': 0,
            'lifecycle': 'ACTIVE',
            'production_ready': True,
            'created_at': float(time.time()),
            'source': {
                'previous_version': previous_version,
                'promoted_candidates': [p['candidate_id'] for p in promoted],
                'phase': phase,
            },
        }
    )
    (dest / 'manifest.yaml').write_text(yaml.safe_dump(man, sort_keys=False), encoding='utf-8')
    (dest / 'coverage.json').write_text(
        json.dumps({'summary': cov, 'promoted': promoted}, indent=2) + '\n', encoding='utf-8'
    )
    validation_report = dict(validation_report)
    validation_report.update(
        {
            'status': 'PASSED',
            'version': target_version,
            'false_accept_count': 0,
            'promoted': promoted,
            'keyframe_count': len(idx),
        }
    )
    (dest / 'validation_report.json').write_text(
        json.dumps(validation_report, indent=2, default=str) + '\n', encoding='utf-8'
    )
    return dest


def build_v11_version(
    *,
    vroot: Path,
    active_root: Path,
    verified: Sequence[CandRecord],
    map_hash: str,
    validation_report: Dict[str, Any],
    cfg: Dict[str, Any],
) -> Path:
    """C1 compatibility wrapper → vp_visual_v1.1 from v1.0 Active."""
    return build_promoted_version(
        vroot=vroot,
        active_root=active_root,
        verified=verified,
        map_hash=map_hash,
        validation_report=validation_report,
        cfg=cfg,
        previous_version=VERSION_V10,
        target_version=VERSION_V11,
        phase='Phase2D-C1',
    )


def mark_rejected_on_disk(recs: Sequence[CandRecord]) -> None:
    for rec in recs:
        if rec.status.startswith('REJECTED') and rec.path.is_dir():
            try:
                m = dict(rec.meta)
                m['validation'] = {'status': rec.status, 'reasons': rec.reasons}
                (rec.path / 'meta.yaml').write_text(yaml.safe_dump(m, sort_keys=False), encoding='utf-8')
            except Exception:  # noqa: BLE001
                pass


def run_c1(
    *,
    maps_dir: Path,
    map_name: str = 'vp',
    work_dir: Optional[Path] = None,
    run_fa: bool = True,
    promote: bool = True,
    reload: bool = True,
    rollback_test: bool = True,
    build_session_id: Optional[str] = None,
    target_version: Optional[str] = None,
    require_active_version: Optional[str] = None,
    phase: str = 'Phase2D-C2',
    stop_check: Optional[Any] = None,
) -> Dict[str, Any]:
    """Validate Candidates and optionally promote to next Active version.

    C1 used require_active_version=vp_visual_v1.0 → vp_visual_v1.1.
    C2 AUTO_BUILD uses current Active (e.g. v1.1) → next (v1.2).
    """
    cfg = load_phase2d_config()
    cfg['maps_dir'] = str(maps_dir)
    cfg['map_name'] = map_name
    vroot = visual_root(maps_dir, map_name)
    active_root, active_ver, src = __import__(
        'xw_global_reloc.phase2d.version_store', fromlist=['resolve_active_root']
    ).resolve_active_root(maps_dir, map_name)
    if require_active_version and active_ver != require_active_version:
        return {
            'ok': False,
            'error': f'expected active {require_active_version}, got {active_ver}',
            'pointer': read_pointer_version(vroot),
        }
    if src == 'missing' or not active_ver or active_ver in ('missing', 'legacy_production_root'):
        return {
            'ok': False,
            'error': f'active_version_unusable:{active_ver}:{src}',
            'pointer': read_pointer_version(vroot),
        }

    try:
        new_version = target_version or next_visual_version(active_ver)
    except ValueError as exc:
        return {'ok': False, 'error': str(exc), 'active_version': active_ver}

    if version_dir(vroot, new_version).exists() and promote:
        # Never overwrite an existing version directory in AUTO_BUILD.
        return {
            'ok': False,
            'error': f'target_version_exists:{new_version}',
            'active_version': active_ver,
            'target_version': new_version,
        }

    y, p = resolve_map_files(maps_dir, map_name)
    mhash = map_pair_hash(y, p)
    cand_root = vroot / 'candidate'
    all_recs = load_candidates(cand_root)
    recs = filter_candidates_for_promote(all_recs, build_session_id=build_session_id)
    report: Dict[str, Any] = {
        'phase': phase,
        'started_at': time.time(),
        'active_version': active_ver,
        'target_version': new_version,
        'previous_version': active_ver,
        'map_hash': mhash,
        'candidates_total_on_disk': len(all_recs),
        'candidates_total': len(recs),
        'build_session_id': build_session_id,
        'skipped': {
            'already_promoted': sum(1 for r in all_recs if r.status == 'ALREADY_PROMOTED'),
            'other_session': sum(1 for r in all_recs if r.status == 'SKIPPED_OTHER_SESSION'),
            'already_rejected': sum(1 for r in all_recs if str(r.status).startswith('REJECTED')),
        },
    }

    if stop_check and stop_check():
        report['ok'] = False
        report['validation_pass'] = False
        report['aborted'] = True
        return report

    # --- Precheck ---
    for r in recs:
        precheck_candidate(r, map_name=map_name, map_hash=mhash)

    # --- Batch dedup ---
    batch_dedup_candidates(recs, cfg)

    # --- Value vs Active ---
    active_cells, active_yaw = _active_cells_yaw(active_root)
    active_frames = _load_db_frames(active_root)
    near_descs = [
        (f['id'], f['descriptors'], f['pose']['x'], f['pose']['y'], f['pose']['yaw']) for f in active_frames
    ]
    for r in recs:
        if r.status == 'PENDING':
            assess_value(
                r,
                active_cells=active_cells,
                active_yaw=active_yaw,
                active_descs_near=near_descs,
                cfg=cfg,
            )

    pending = [r for r in recs if r.status == 'PENDING']
    work = Path(work_dir) if work_dir else Path(tempfile.mkdtemp(prefix='phase2d_c2_'))
    work.mkdir(parents=True, exist_ok=True)
    eval_db = work / 'temp_eval_db'
    build_temp_eval_db(active_root, pending, eval_db)

    # Snapshot previous Active hashes for safety
    prev_inv = inventory_keyframe_hashes(active_root)
    report['previous_inventory'] = {
        'keyframe_count': prev_inv['keyframe_count'],
        'index_sha256': prev_inv['index_sha256'],
        'manifest_sha256': prev_inv['manifest_sha256'],
    }
    # C1 report key retained
    report['v1.0_inventory'] = report['previous_inventory'] if active_ver == VERSION_V10 else None

    baseline_frames = _load_db_frames(active_root)
    tentative_frames = _load_db_frames(eval_db)

    # E1 retrieval
    e1_before = retrieval_loo_metrics(baseline_frames)
    e1_after = retrieval_loo_metrics(tentative_frames)
    legacy_after_pool = [f for f in tentative_frames]
    legacy_queries = [f for f in tentative_frames if not str(f['id']).startswith('cand_')]
    hit1 = hit3 = hit5 = 0
    legacy_trials = []
    for q in legacy_queries:
        pool = [k for k in legacy_after_pool if k['id'] != q['id']]
        res = retrieve_topk(q['rgb'], pool, top_k=5)
        ranks = [c.rank for c in res.candidates if next(k for k in pool if k['id'] == c.keyframe_id)['region_id'] == q['region_id']]
        best = min(ranks) if ranks else None
        h1 = best == 1
        h3 = best is not None and best <= 3
        h5 = best is not None and best <= 5
        hit1 += int(h1)
        hit3 += int(h3)
        hit5 += int(h5)
        legacy_trials.append({'query_id': q['id'], 'best': best, 'hit@1': h1, 'hit@3': h3, 'hit@5': h5})
    nleg = len(legacy_trials)
    e1_legacy_after = {
        'n_queries': nleg,
        'Hit@1': hit1 / nleg if nleg else 0.0,
        'Hit@3': hit3 / nleg if nleg else 0.0,
        'Hit@5': hit5 / nleg if nleg else 0.0,
    }
    report['E1_retrieval'] = {
        'baseline_active': {k: e1_before[k] for k in e1_before if k != 'trials'},
        'baseline_v1.0': {k: e1_before[k] for k in e1_before if k != 'trials'},
        'tentative_full': {k: e1_after[k] for k in e1_after if k != 'trials'},
        'legacy_queries_on_tentative': e1_legacy_after,
    }

    reg_ok = True
    for metric in ('Hit@3', 'Hit@5'):
        before = float(e1_before[metric])
        after = float(e1_legacy_after[metric])
        if after + 1e-9 < before - 0.05:
            reg_ok = False
    report['legacy_regression_ok'] = reg_ok

    if stop_check and stop_check():
        report['ok'] = False
        report['validation_pass'] = False
        report['aborted'] = True
        return report

    # E2 LOO on pending candidates
    e2 = {}
    for r in pending:
        chk = candidate_loo_check(r, tentative_frames)
        e2[r.id] = {k: chk[k] for k in chk if k != 'topk'}
        e2[r.id]['topk'] = chk.get('topk', [])[:5]
        if chk.get('self_only_risk'):
            if not r.value.get('new_spatial_cell'):
                r.status = 'REJECTED_CONFUSION'
                r.reasons = ['loo_self_only_no_new_cell']
            else:
                r.reasons.append('loo_self_only_risk_new_cell')
                r.value['loo_risk'] = True
    report['E2_loo'] = e2

    pending = [r for r in recs if r.status == 'PENDING']

    # E3 confusion
    e3 = confusion_analysis(tentative_frames)
    report['E3_confusion'] = {
        'n_cases': e3['n_cases'],
        'visual_confusion_cases': e3['visual_confusion_cases'],
        'note': e3['note'],
        'sample': e3['cases'][:8],
    }
    dangerous_ids: Set[str] = set()
    for case in e3['cases']:
        if not case.get('visual_confusion'):
            continue
        for row in case['topk'][:3]:
            if row.get('distance_m', 0) > 2.0 and not row.get('same_region') and str(row['id']).startswith('cand_'):
                dangerous_ids.add(str(row['id']))
    for r in pending:
        if r.id in dangerous_ids:
            r.value['confusion_flag'] = True

    # E4 Visual+Laser FA
    if run_fa:
        e4_base = visual_laser_fa_validation(active_root, maps_dir / f'{map_name}.yaml')
        e4_tent = visual_laser_fa_validation(eval_db, maps_dir / f'{map_name}.yaml')
    else:
        e4_base = {'false_accept_count': 0, 'skipped': True}
        e4_tent = {'false_accept_count': 0, 'skipped': True}
    report['E4_visual_laser'] = {
        'baseline': {k: e4_base[k] for k in e4_base if k != 'trials'},
        'tentative': {k: e4_tent[k] for k in e4_tent if k != 'trials'},
    }
    fa_count = int(e4_tent.get('false_accept_count') or 0)
    fa_ok = fa_count == 0
    report['false_accept_count'] = fa_count
    if not fa_ok:
        for r in pending:
            r.status = 'REJECTED_SAFETY'
            r.reasons = ['false_accept_nonzero']
        pending = []

    for r in list(pending):
        if r.status != 'PENDING':
            continue
        if r.value.get('confusion_flag') and not (
            r.value.get('new_spatial_cell') or r.value.get('new_yaw_bin')
        ):
            r.status = 'REJECTED_CONFUSION'
            r.reasons = ['confusion_without_coverage_value']
            continue
        if not (r.value.get('new_spatial_cell') or r.value.get('new_yaw_bin') or r.value.get('visual_novel')):
            r.status = 'REJECTED_NO_VALUE'
            r.reasons = ['no_value_after_gates']
            continue
        r.status = 'VERIFIED'
        r.reasons = ['passed_c1_gates']

    verified = [r for r in recs if r.status == 'VERIFIED']
    # Persist VERIFIED on disk before promote (PROMOTED overwrites after success)
    for r in verified:
        try:
            cm = dict(r.meta)
            cm['validation'] = {'status': 'VERIFIED', 'phase': phase, 'reasons': r.reasons}
            (r.path / 'meta.yaml').write_text(yaml.safe_dump(cm, sort_keys=False), encoding='utf-8')
            r.meta = cm
        except Exception:  # noqa: BLE001
            pass
    mark_rejected_on_disk(recs)

    cov_before = coverage_snapshot_from_db(active_root, cfg)
    report['candidate_results'] = [
        {
            'id': r.id,
            'status': r.status,
            'reasons': r.reasons,
            'value': r.value,
            'spatial_cell': r.meta.get('spatial_cell'),
            'yaw_bin': r.meta.get('yaw_bin'),
            'laser_score': r.meta.get('laser_score'),
            'build_session_id': r.meta.get('build_session_id'),
        }
        for r in recs
    ]
    report['counts'] = {
        'total': len(recs),
        'verified': len(verified),
        'rejected_duplicate': sum(1 for r in recs if r.status == 'REJECTED_DUPLICATE'),
        'rejected_invalid': sum(1 for r in recs if r.status == 'REJECTED_INVALID'),
        'rejected_confusion': sum(1 for r in recs if r.status == 'REJECTED_CONFUSION'),
        'rejected_no_value': sum(1 for r in recs if r.status == 'REJECTED_NO_VALUE'),
        'rejected_safety': sum(1 for r in recs if r.status == 'REJECTED_SAFETY'),
    }
    report['Hit@1'] = e1_legacy_after.get('Hit@1')
    report['Hit@3'] = e1_legacy_after.get('Hit@3')
    report['Hit@5'] = e1_legacy_after.get('Hit@5')

    promote_gate = {
        'false_accept_0': fa_ok,
        'map_hash_match': True,
        'legacy_regression_ok': reg_ok,
        'coverage_increase': False,
        'verified_nonempty': len(verified) > 0,
        'batch_dedup': True,
        'loo': True,
        'confusion': True,
    }

    proj_cells = set(active_cells)
    proj_yaw = set(active_yaw)
    for r in verified:
        cell = str(r.meta.get('spatial_cell'))
        yb = int(r.meta.get('yaw_bin'))
        proj_cells.add(cell)
        proj_yaw.add((cell, yb))
    coverage_up = len(proj_cells) > len(active_cells) or len(proj_yaw) > len(active_yaw)
    promote_gate['coverage_increase'] = coverage_up
    report['promote_gate'] = promote_gate
    report['coverage_before'] = cov_before
    report['coverage_v1.0'] = cov_before
    report['coverage_projected'] = {
        'cells': len(proj_cells),
        'yaw_occupancy': len(proj_yaw),
        'frames': cov_before['frames'] + len(verified),
    }

    can_promote = all(
        [
            promote_gate['false_accept_0'],
            promote_gate['map_hash_match'],
            promote_gate['legacy_regression_ok'],
            promote_gate['coverage_increase'],
            promote_gate['verified_nonempty'],
        ]
    )
    report['validation_pass'] = can_promote and fa_ok and reg_ok

    if stop_check and stop_check():
        report['ok'] = False
        report['validation_pass'] = False
        report['aborted'] = True
        report['promote'] = {'skipped': True, 'reason': 'aborted_before_promote'}
        return report

    promote_result: Dict[str, Any] = {'skipped': True}
    reload_result: Dict[str, Any] = {}
    rollback_result: Dict[str, Any] = {}

    if promote and can_promote and verified:
        built = build_promoted_version(
            vroot=vroot,
            active_root=active_root,
            verified=verified,
            map_hash=mhash,
            validation_report=report,
            cfg=cfg,
            previous_version=active_ver,
            target_version=new_version,
            phase=phase,
        )
        ok, reason = validate_version_for_activate(
            vroot, new_version, maps_dir=maps_dir, map_name=map_name, require_map_hash_match=True
        )
        promote_result = {
            'built': str(built),
            'validate': reason,
            'ok': ok,
            'promoted_count': len(verified),
            'promoted_ids': [{'cand': r.id, 'kf': r.promoted_id} for r in verified],
            'previous_version': active_ver,
            'new_version': new_version,
        }
        prev_after = inventory_keyframe_hashes(active_root)
        promote_result['previous_unchanged'] = (
            prev_after['index_sha256'] == prev_inv['index_sha256']
            and prev_after['keyframe_count'] == prev_inv['keyframe_count']
        )
        promote_result['v1.0_unchanged'] = promote_result['previous_unchanged']
        if ok and promote_result['previous_unchanged']:
            if reload:
                from xw_global_reloc.phase2d.set_active_cli import set_active_version

                reload_result = set_active_version(
                    maps_dir=maps_dir,
                    map_name=map_name,
                    version=new_version,
                    reload=True,
                    timeout=60.0,
                )
                promote_result['set_active'] = reload_result
                if reload_result.get('status') != 'OK':
                    promote_result['commit'] = False
                    report['validation_pass'] = False
                    report['failed_stage'] = 'RELOAD'
                else:
                    promote_result['commit'] = True
                    ptr = read_pointer_version(vroot)
                    loaded = load_visual_db_from_root((vroot / 'current_active_version').resolve())
                    promote_result['post_reload'] = {
                        'pointer': ptr,
                        'loaded_version': loaded.version,
                        'loaded_count': len(loaded.keyframes),
                        'map_hash': loaded.db_hash,
                        'error': loaded.error,
                        'candidate_ids_in_memory': [
                            k['id'] for k in loaded.keyframes if str(k['id']).startswith('cand_')
                        ],
                    }
                    if rollback_test:
                        rb = set_active_version(
                            maps_dir=maps_dir,
                            map_name=map_name,
                            version=active_ver,
                            reload=True,
                            timeout=60.0,
                        )
                        fwd = set_active_version(
                            maps_dir=maps_dir,
                            map_name=map_name,
                            version=new_version,
                            reload=True,
                            timeout=60.0,
                        )
                        rollback_result = {
                            f'to_{active_ver}': rb,
                            f'back_to_{new_version}': fwd,
                            'final_pointer': read_pointer_version(vroot),
                            'ok': rb.get('status') in ('OK', 'POINTER_OK')
                            and fwd.get('status') in ('OK', 'POINTER_OK'),
                        }
            else:
                atomic_set_pointer(vroot, new_version)
                promote_result['commit'] = True
                promote_result['set_active'] = {'status': 'POINTER_OK', 'reload': False}
        else:
            promote_result['commit'] = False
            report['validation_pass'] = False
            report['failed_stage'] = 'PROMOTE'
    elif not can_promote:
        promote_result = {'skipped': True, 'reason': 'promote_gate_failed', 'gate': promote_gate}
        report['failed_stage'] = 'VALIDATION'

    cov_new = None
    if (version_dir(vroot, new_version)).is_dir():
        cov_new = coverage_snapshot_from_db(version_dir(vroot, new_version), cfg)

    report.update(
        {
            'ok': bool(report.get('validation_pass')) and bool(promote_result.get('commit', not promote)),
            'promote': promote_result,
            'reload': reload_result,
            'rollback': rollback_result,
            'coverage_after': cov_new,
            'coverage_v1.1': cov_new if new_version == VERSION_V11 else None,
            f'coverage_{new_version}': cov_new,
            'finished_at': time.time(),
            'work_dir': str(work),
            'pointer_final': read_pointer_version(vroot),
            'new_version': new_version,
            'old_version': active_ver,
        }
    )

    out_json = work / 'c2_validation_report.json'
    slim = dict(report)
    out_json.write_text(json.dumps(slim, indent=2, default=str) + '\n', encoding='utf-8')
    rep_dir = cand_root / 'reports'
    rep_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d')
    (rep_dir / f'c2_validation_promote_{stamp}.json').write_text(
        json.dumps(slim, indent=2, default=str) + '\n', encoding='utf-8'
    )
    return report


# Back-compat alias
run_validate_promote = run_c1
