#!/usr/bin/env python3
"""Offline Phase2D-B2 micro plan + Active integrity check (no Nav2 required)."""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

from xw_global_reloc.phase2d.config_loader import load_phase2d_config, production_visual_root
from xw_global_reloc.phase2d.coverage_model import build_coverage_model
from xw_global_reloc.phase2d.coverage_report import build_coverage_dict
from xw_global_reloc.phase2d.patrol_planner import plan_patrol_goals
from xw_global_reloc.phase2d.version_store import read_pointer_version, resolve_active_root


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main() -> None:
    cfg = load_phase2d_config()
    cfg['maps_dir'] = '/ros2_ws/maps'
    cfg['map_name'] = 'vp'
    maps = Path('/ros2_ws/maps')
    vroot = production_visual_root(cfg)

    man = vroot / 'manifest.yaml'
    idx = vroot / 'descriptors' / 'index.json'
    h_before = {
        'manifest': _sha(man),
        'index': _sha(idx),
        'pointer': str((vroot / 'current_active_version').readlink()),
    }
    root, ver, _src = resolve_active_root(maps, 'vp')
    model = build_coverage_model(cfg, load_descriptors=False)
    before = build_coverage_dict(model)['summary']
    goals_m = plan_patrol_goals(model, map_yaml=maps / 'vp.yaml', cfg=cfg, mode='micro')
    goals_p = plan_patrol_goals(model, map_yaml=maps / 'vp.yaml', cfg=cfg, mode='partial')
    cand_n = 0
    ck = vroot / 'candidate' / 'keyframes'
    if ck.is_dir():
        cand_n = sum(1 for p in ck.iterdir() if p.is_dir())
    sid = 'build_offline_micro_' + time.strftime('%Y%m%d_%H%M%S')
    session = {
        'build_session_id': sid,
        'mode': 'micro',
        'state': 'COMPLETE',
        'source': 'auto_patrol_offline_plan',
        'planned_goals': [g.as_dict() for g in goals_m],
        'partial_goal_count': len(goals_p),
        'coverage_before': before,
        'candidate_count_before': cand_n,
        'note': 'Nav2/Capture not executed (bringup offline). Planner+isolation validated.',
    }
    state_dir = vroot / 'state'
    state_dir.mkdir(parents=True, exist_ok=True)
    out = state_dir / f'{sid}.json'
    out.write_text(json.dumps(session, indent=2) + '\n', encoding='utf-8')
    h_after = {
        'manifest': _sha(man),
        'index': _sha(idx),
        'pointer': str((vroot / 'current_active_version').readlink()),
    }
    print(
        json.dumps(
            {
                'active_version': ver,
                'active_root': str(root),
                'pointer': read_pointer_version(vroot),
                'hashes_unchanged': h_before == h_after,
                'hashes': h_before,
                'coverage_before': before,
                'micro_goals': len(goals_m),
                'partial_goals': len(goals_p),
                'micro_goal_sample': [g.as_dict() for g in goals_m[:6]],
                'candidate_count': cand_n,
                'session_path': str(out),
            },
            indent=2,
        )
    )


if __name__ == '__main__':
    main()
