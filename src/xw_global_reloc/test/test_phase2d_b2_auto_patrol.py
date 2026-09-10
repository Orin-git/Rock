#!/usr/bin/env python3
"""Phase2D-B2 tests: patrol planner, Candidate-only, Active unchanged, goal limits."""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict, List, Set, Tuple

import numpy as np
import yaml

_SRC = Path(__file__).resolve().parents[1]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from xw_global_reloc.phase2d.config_loader import load_phase2d_config  # noqa: E402
from xw_global_reloc.phase2d.coverage_model import build_coverage_model  # noqa: E402
from xw_global_reloc.phase2d.patrol_planner import (  # noqa: E402
    erode_free,
    load_free_mask,
    plan_patrol_goals,
    world_to_map,
)
from xw_global_reloc.phase2d.version_store import read_pointer_version, resolve_active_root  # noqa: E402

PROD_MAPS = Path('/ros2_ws/maps')
if not PROD_MAPS.is_dir():
    PROD_MAPS = Path('/home/radxa/ros2_ws/maps')

LEGACY_MANIFEST = 'ced28edc60c8530c3265865e9683dabd8b3067e8bce9b4d7fa0e3fe89bfbf19a'
LEGACY_INDEX = '445b27cdc0fd424664c58d550c403dae8b62fc84e8a595a1860e1d8096284eb7'


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


@unittest.skipUnless((PROD_MAPS / 'vp.yaml').is_file(), 'vp map missing')
class TestPatrolPlannerLiveMap(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = load_phase2d_config()
        self.cfg['maps_dir'] = str(PROD_MAPS)
        self.cfg['map_name'] = 'vp'
        self.map_yaml = PROD_MAPS / 'vp.yaml'
        self.model = build_coverage_model(self.cfg, load_descriptors=False)

    def test_free_mask_and_clearance(self) -> None:
        free, info = load_free_mask(self.map_yaml)
        self.assertTrue(free.any())
        safe = erode_free(free, 2)
        self.assertLessEqual(int(safe.sum()), int(free.sum()))
        # eroded cells must be subset of free
        self.assertFalse(bool(np.any(safe & (~free))))

    def test_micro_goal_limits_and_free(self) -> None:
        goals = plan_patrol_goals(
            self.model, map_yaml=self.map_yaml, cfg=self.cfg, mode='micro'
        )
        self.assertGreater(len(goals), 0)
        self.assertLessEqual(len(goals), int(self.cfg['patrol']['micro_max_goals']))
        free, info = load_free_mask(self.map_yaml)
        k = max(
            int(self.cfg['patrol'].get('free_kernel_cells', 2)),
            int(math.ceil(float(self.cfg['patrol'].get('goal_clearance_m', 0.45)) / float(info['resolution']))),
        )
        safe = erode_free(free, k)
        cells: Set[Tuple[int, int]] = set()
        for g in goals:
            ix, iy = world_to_map(g.x, g.y, info)
            self.assertTrue(safe[iy, ix], msg=f'goal not clear: {g}')
            cells.add((int(g.spatial_cell.split('_')[1]), int(g.spatial_cell.split('_')[2])))
        # adjacent-ish: micro should not scatter across entire map
        self.assertLessEqual(len(cells), int(self.cfg['patrol']['micro_max_goals']))
        yaw_per: Dict[str, int] = {}
        for g in goals:
            yaw_per[g.spatial_cell] = yaw_per.get(g.spatial_cell, 0) + 1
        for c, n in yaw_per.items():
            self.assertLessEqual(n, int(self.cfg['patrol']['max_yaw_targets_per_cell']), msg=c)

    def test_no_frontier_unknown_goals(self) -> None:
        """Goals only on eroded free; never unknown."""
        free, info = load_free_mask(self.map_yaml)
        safe = erode_free(free, 2)
        goals = plan_patrol_goals(
            self.model, map_yaml=self.map_yaml, cfg=self.cfg, mode='partial'
        )
        for g in goals:
            ix, iy = world_to_map(g.x, g.y, info)
            self.assertTrue(bool(safe[iy, ix]))

    def test_partial_larger_than_micro(self) -> None:
        micro = plan_patrol_goals(self.model, map_yaml=self.map_yaml, cfg=self.cfg, mode='micro')
        partial = plan_patrol_goals(self.model, map_yaml=self.map_yaml, cfg=self.cfg, mode='partial')
        self.assertLessEqual(len(micro), len(partial))
        self.assertLessEqual(len(partial), int(self.cfg['patrol']['max_total_goals']))


@unittest.skipUnless((PROD_MAPS / 'vp' / 'visual' / 'current_active_version').exists(), 'active pointer missing')
class TestActiveUnchangedAndCandidateOnly(unittest.TestCase):
    def test_active_hashes_and_pointer(self) -> None:
        vroot = PROD_MAPS / 'vp' / 'visual'
        self.assertEqual(read_pointer_version(vroot), 'vp_visual_v1.0')
        root, ver, src = resolve_active_root(PROD_MAPS, 'vp')
        self.assertEqual(ver, 'vp_visual_v1.0')
        man = root / 'manifest.yaml'
        idx = root / 'descriptors' / 'index.json'
        # Prefer production flat copies if still present (B1 kept originals)
        flat_man = vroot / 'manifest.yaml'
        flat_idx = vroot / 'descriptors' / 'index.json'
        for label, p, expect in (
            ('flat_manifest', flat_man, LEGACY_MANIFEST),
            ('flat_index', flat_idx, LEGACY_INDEX),
            ('active_manifest', man, None),
            ('active_index', idx, None),
        ):
            if not p.is_file():
                continue
            h = _sha(p)
            if expect:
                self.assertEqual(h, expect, msg=label)
            self.assertTrue(len(h) == 64)

        # Candidate path isolation
        cand = vroot / 'candidate'
        self.assertTrue(cand.is_dir())
        versions = vroot / 'versions' / 'vp_visual_v1.0'
        # No candidate keyframes under versions
        if (cand / 'keyframes').is_dir():
            for kid in (cand / 'keyframes').iterdir():
                self.assertFalse(str(kid).startswith(str(versions)))

    def test_planner_does_not_mutate_active(self) -> None:
        cfg = load_phase2d_config()
        cfg['maps_dir'] = str(PROD_MAPS)
        cfg['map_name'] = 'vp'
        vroot = PROD_MAPS / 'vp' / 'visual'
        before_m = _sha(vroot / 'manifest.yaml') if (vroot / 'manifest.yaml').is_file() else ''
        before_i = _sha(vroot / 'descriptors' / 'index.json') if (vroot / 'descriptors' / 'index.json').is_file() else ''
        before_ptr = (vroot / 'current_active_version').readlink()
        model = build_coverage_model(cfg, load_descriptors=False)
        _ = plan_patrol_goals(model, map_yaml=PROD_MAPS / 'vp.yaml', cfg=cfg, mode='micro')
        after_m = _sha(vroot / 'manifest.yaml') if (vroot / 'manifest.yaml').is_file() else ''
        after_i = _sha(vroot / 'descriptors' / 'index.json') if (vroot / 'descriptors' / 'index.json').is_file() else ''
        after_ptr = (vroot / 'current_active_version').readlink()
        self.assertEqual(before_m, after_m)
        self.assertEqual(before_i, after_i)
        self.assertEqual(before_ptr, after_ptr)


class TestBuildSessionTallyLogic(unittest.TestCase):
    def test_states_and_tally_buckets(self) -> None:
        from xw_global_reloc.phase2d.build_orchestrator_node import STATES, BuildSession

        for s in (
            'IDLE', 'PRECHECK', 'PLANNING', 'PATROLLING', 'COLLECTING',
            'COMPLETE', 'PAUSED', 'FAILED', 'ABORTED',
        ):
            self.assertIn(s, STATES)
        sess = BuildSession(build_session_id='t', mode='micro')
        d = sess.to_dict()
        self.assertEqual(d['mode'], 'micro')
        self.assertIn('coverage_before', d)


class TestCandidateWriterIsolation(unittest.TestCase):
    def test_refuse_production_keyframes(self) -> None:
        from xw_global_reloc.phase2d.candidate_writer import CandidateWriter

        tmp = Path(tempfile.mkdtemp(prefix='b2cand_'))
        try:
            maps = tmp
            (maps / 'vp' / 'visual' / 'keyframes').mkdir(parents=True)
            (maps / 'vp' / 'visual' / 'candidate').mkdir(parents=True)
            cfg = {
                'maps_dir': str(maps),
                'map_name': 'vp',
                'candidate_root': '',
            }
            w = CandidateWriter(cfg)
            w.ensure_dirs()
            with self.assertRaises(RuntimeError):
                w.assert_not_production(maps / 'vp' / 'visual' / 'keyframes')
            # candidate root itself OK
            w.assert_not_production(w.root)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    unittest.main(verbosity=2)
