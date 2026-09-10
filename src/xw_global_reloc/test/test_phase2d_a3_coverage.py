#!/usr/bin/env python3
"""Phase2D-A3 unit tests: spatial/yaw, policy, dedup, quota, coverage report."""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Optional

import numpy as np

_SRC = Path(__file__).resolve().parents[1]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from xw_global_reloc.phase2d.capture_policy import (  # noqa: E402
    CAPTURE_NEW_CELL,
    CAPTURE_NEW_YAW,
    CAPTURE_TRANSLATION,
    CAPTURE_VISUAL_NOVELTY,
    SKIP_CELL_YAW_QUOTA,
    SKIP_DUPLICATE,
    evaluate_capture_policy,
)
from xw_global_reloc.phase2d.config_loader import load_phase2d_config, legacy_production_paths  # noqa: E402
from xw_global_reloc.phase2d.coverage_model import CoverageModel, FrameRef, build_coverage_model  # noqa: E402
from xw_global_reloc.phase2d.coverage_report import build_coverage_dict, write_coverage_report  # noqa: E402
from xw_global_reloc.phase2d.dedup import evaluate_dedup  # noqa: E402
from xw_global_reloc.phase2d.spatial import (  # noqa: E402
    parse_spatial_cell,
    spatial_cell_id,
    wrap_yaw,
    yaw_bin_index,
)
from xw_global_reloc.phase2d.candidate_writer import production_index_ids  # noqa: E402


def _cfg(tmp: Optional[Path] = None) -> dict:
    cfg = load_phase2d_config()
    for maps in ('/ros2_ws/maps', '/home/radxa/ros2_ws/maps'):
        if Path(maps).is_dir():
            cfg['maps_dir'] = maps
            break
    if tmp is not None:
        cfg['candidate_root'] = str(tmp / 'candidate')
    return cfg


def _desc(seed: int, n: int = 200) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, size=(n, 32), dtype=np.uint8)


def _add(model: CoverageModel, kid: str, x: float, y: float, yaw: float, life: str, desc=None) -> FrameRef:
    cell, yb = model.assign(x, y, yaw)
    ref = FrameRef(
        keyframe_id=kid,
        lifecycle=life,
        x=x,
        y=y,
        yaw=yaw,
        spatial_cell=cell,
        yaw_bin=yb,
        descriptors=desc if desc is not None else _desc(hash(kid) % 10000),
        timestamp=1.0,
    )
    model.add_frame(ref)
    return ref


class TestSpatialYaw(unittest.TestCase):
    def test_cell_positive_negative(self) -> None:
        self.assertEqual(spatial_cell_id(1.2, -0.3, 1.0), 'cell_1_-1')
        self.assertEqual(spatial_cell_id(-0.1, 0.1, 1.0), 'cell_-1_0')
        self.assertEqual(spatial_cell_id(-3.9, 1.1, 1.0), 'cell_-4_1')
        self.assertEqual(parse_spatial_cell('cell_-3_1'), (-3, 1))
        self.assertEqual(parse_spatial_cell('cell_-3_-1'), (-3, -1))
        self.assertEqual(parse_spatial_cell('cell_2_5'), (2, 5))

    def test_yaw_wrap(self) -> None:
        self.assertAlmostEqual(wrap_yaw(0.0), 0.0)
        self.assertAlmostEqual(wrap_yaw(2 * math.pi), 0.0, places=9)
        self.assertAlmostEqual(wrap_yaw(-math.pi / 2), 1.5 * math.pi, places=9)

    def test_eight_yaw_bins(self) -> None:
        # bin width = 45 deg
        self.assertEqual(yaw_bin_index(0.0, 8), 0)
        self.assertEqual(yaw_bin_index(math.radians(44.9), 8), 0)
        self.assertEqual(yaw_bin_index(math.radians(45.0), 8), 1)
        self.assertEqual(yaw_bin_index(math.radians(180.0), 8), 4)
        self.assertEqual(yaw_bin_index(math.radians(359.0), 8), 7)


class TestCapturePolicyAndDedup(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = _cfg()
        self.model = CoverageModel(self.cfg)

    def test_empty_cell_capture(self) -> None:
        d = evaluate_capture_policy(
            self.model, x=10.0, y=10.0, yaw=0.0, query_descriptors=_desc(1), cfg=self.cfg
        )
        self.assertTrue(d.should_capture)
        self.assertEqual(d.reason, CAPTURE_NEW_CELL)

    def test_existing_cell_new_yaw(self) -> None:
        _add(self.model, 'a1', 0.2, 0.2, 0.0, 'ACTIVE')
        # yaw ~90 deg → different bin
        d = evaluate_capture_policy(
            self.model, x=0.25, y=0.25, yaw=math.radians(90), query_descriptors=_desc(2), cfg=self.cfg
        )
        self.assertTrue(d.should_capture)
        self.assertEqual(d.reason, CAPTURE_NEW_YAW)

    def test_near_same_yaw_similar_duplicate(self) -> None:
        desc = _desc(42)
        _add(self.model, 'a1', 0.0, 0.0, 0.0, 'ACTIVE', desc=desc)
        # identical descriptors → visual_difference ~0
        d = evaluate_capture_policy(
            self.model, x=0.05, y=0.05, yaw=math.radians(5), query_descriptors=desc, cfg=self.cfg
        )
        self.assertFalse(d.should_capture)
        self.assertEqual(d.reason, SKIP_DUPLICATE)

    def test_near_same_yaw_visual_novelty(self) -> None:
        _add(self.model, 'a1', 0.0, 0.0, 0.0, 'ACTIVE', desc=_desc(1))
        # very different descriptors
        d = evaluate_capture_policy(
            self.model, x=0.05, y=0.05, yaw=0.0, query_descriptors=_desc(999), cfg=self.cfg
        )
        self.assertTrue(d.should_capture)
        self.assertEqual(d.reason, CAPTURE_VISUAL_NOVELTY)
        self.assertTrue(d.possible_new_appearance)

    def test_translation_capture(self) -> None:
        # Same cell (1m), already has yaw bin 0, similar appearance blocked → use translation
        # Place active at 0.1, query at 0.9 same cell → dist 0.8 >= 0.75
        desc = _desc(7)
        _add(self.model, 'a1', 0.1, 0.1, 0.0, 'ACTIVE', desc=desc)
        # Use same desc so not novelty; same yaw bin; translation triggers
        # But dedup: dist 0.8 > max_xy 0.35 so not duplicate
        d = evaluate_capture_policy(
            self.model, x=0.9, y=0.1, yaw=0.0, query_descriptors=desc, cfg=self.cfg
        )
        self.assertTrue(d.should_capture)
        self.assertEqual(d.reason, CAPTURE_TRANSLATION)

    def test_candidate_participates_dedup(self) -> None:
        desc = _desc(55)
        _add(self.model, 'c1', 1.0, 1.0, 0.0, 'CANDIDATE', desc=desc)
        d = evaluate_dedup(
            self.model, x=1.05, y=1.02, yaw=0.02, query_descriptors=desc, cfg=self.cfg
        )
        self.assertTrue(d.is_duplicate)
        self.assertEqual(d.nearest_keyframe_id, 'c1')

    def test_quota_blocks(self) -> None:
        self.cfg['candidate_limits']['max_per_cell_yaw'] = 2
        for i in range(2):
            _add(self.model, f'a{i}', 0.1, 0.1, 0.0, 'ACTIVE', desc=_desc(i + 10))
        d = evaluate_capture_policy(
            self.model, x=0.12, y=0.12, yaw=0.0, query_descriptors=_desc(999), cfg=self.cfg
        )
        self.assertFalse(d.should_capture)
        self.assertEqual(d.reason, SKIP_CELL_YAW_QUOTA)


class TestLegacyCoverageReadonly(unittest.TestCase):
    def test_legacy_active_readonly_coverage(self) -> None:
        cfg = _cfg()
        paths = legacy_production_paths(cfg)
        meta = paths['keyframes'] / 'kf_000001' / 'meta.yaml'
        before = meta.read_bytes() if meta.is_file() else b''
        model = build_coverage_model(cfg, load_descriptors=False)
        base = model.baseline_stats()
        self.assertEqual(base['active_frames'], 37)
        self.assertEqual(base['active_occupied_cells'], 9)
        after = meta.read_bytes() if meta.is_file() else b''
        self.assertEqual(before, after)

    def test_coverage_report_stats(self) -> None:
        cfg = _cfg()
        tmp = Path(tempfile.mkdtemp(prefix='covrep_'))
        try:
            cfg['candidate_root'] = str(tmp / 'candidate')
            model = build_coverage_model(cfg, load_descriptors=False)
            out = write_coverage_report(cfg, out_dir=tmp / 'reports', model=model)
            data = json.loads(Path(out['coverage_json']).read_text(encoding='utf-8'))
            self.assertEqual(data['summary']['active_frames'], 37)
            self.assertEqual(data['summary']['active_occupied_cells'], 9)
            self.assertTrue(Path(out['coverage_md']).is_file())
            md = Path(out['coverage_md']).read_text(encoding='utf-8')
            self.assertIn('Active occupied cells', md)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_candidate_not_in_production_index(self) -> None:
        cfg = _cfg()
        ids = production_index_ids(cfg)
        self.assertEqual(len(ids), 37)
        self.assertTrue(all(i.startswith('kf_') for i in ids))
        manifest = legacy_production_paths(cfg)['manifest']
        index = legacy_production_paths(cfg)['index']
        self.assertEqual(
            hashlib.sha256(manifest.read_bytes()).hexdigest(),
            'ced28edc60c8530c3265865e9683dabd8b3067e8bce9b4d7fa0e3fe89bfbf19a',
        )
        self.assertEqual(
            hashlib.sha256(index.read_bytes()).hexdigest(),
            '445b27cdc0fd424664c58d550c403dae8b62fc84e8a595a1860e1d8096284eb7',
        )


if __name__ == '__main__':
    unittest.main()
