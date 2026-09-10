#!/usr/bin/env python3
"""CLI: Phase2D Candidate validation + safe promote (C1/C2)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from xw_global_reloc.phase2d.c1_validate_promote import VERSION_V10, run_c1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='Phase2D validate + promote next Active version')
    p.add_argument('--maps-dir', default='/ros2_ws/maps')
    p.add_argument('--map-name', default='vp')
    p.add_argument('--work-dir', default='/ros2_ws/bench/phase2d_c2_2026-09-10')
    p.add_argument('--build-session-id', default='', help='Only validate this session Candidates')
    p.add_argument('--target-version', default='', help='Override next version id')
    p.add_argument('--require-active', default='', help='Fail unless Active equals this version')
    p.add_argument('--c1-legacy', action='store_true', help='Require Active=v1.0 (original C1)')
    p.add_argument('--no-fa', action='store_true', help='Skip Visual+Laser FA (debug only)')
    p.add_argument('--no-promote', action='store_true')
    p.add_argument('--no-reload', action='store_true')
    p.add_argument('--no-rollback-test', action='store_true')
    args = p.parse_args(argv)
    work = Path(args.work_dir)
    work.mkdir(parents=True, exist_ok=True)
    require = args.require_active or (VERSION_V10 if args.c1_legacy else None)
    report = run_c1(
        maps_dir=Path(args.maps_dir),
        map_name=args.map_name,
        work_dir=work,
        run_fa=not args.no_fa,
        promote=not args.no_promote,
        reload=not args.no_reload,
        rollback_test=not args.no_rollback_test,
        build_session_id=args.build_session_id or None,
        target_version=args.target_version or None,
        require_active_version=require or None,
        phase='Phase2D-C1' if args.c1_legacy else 'Phase2D-C2',
    )
    summary = {
        'validation_pass': report.get('validation_pass'),
        'ok': report.get('ok'),
        'error': report.get('error'),
        'old_version': report.get('old_version') or report.get('active_version'),
        'new_version': report.get('new_version') or report.get('target_version'),
        'counts': report.get('counts'),
        'false_accept_count': report.get('false_accept_count'),
        'E1': report.get('E1_retrieval'),
        'E4': report.get('E4_visual_laser'),
        'promote_gate': report.get('promote_gate'),
        'promote': {
            k: report.get('promote', {}).get(k)
            for k in (
                'ok',
                'commit',
                'promoted_count',
                'promoted_ids',
                'previous_unchanged',
                'v1.0_unchanged',
                'post_reload',
                'new_version',
                'previous_version',
            )
        },
        'rollback': report.get('rollback'),
        'coverage_before': report.get('coverage_before') or report.get('coverage_v1.0'),
        'coverage_after': report.get('coverage_after'),
        'pointer_final': report.get('pointer_final'),
        'failed_stage': report.get('failed_stage'),
    }
    print(json.dumps(summary, indent=2, default=str))
    if report.get('error'):
        return 2
    return 0 if report.get('validation_pass') else 1


if __name__ == '__main__':
    raise SystemExit(main())
