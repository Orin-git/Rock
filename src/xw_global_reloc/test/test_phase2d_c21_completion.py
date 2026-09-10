"""Phase2D-C2.1 coverage completion gate tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from xw_global_reloc.phase2d.build_completion import (
    evaluate_coverage_completion,
    remaining_gap_cells,
)
from xw_global_reloc.phase2d.config_loader import load_phase2d_config
from xw_global_reloc.phase2d.coverage_model import build_coverage_model


MAPS = Path('/ros2_ws/maps')
MAP_YAML = MAPS / 'vp.yaml'


@pytest.mark.skipif(not MAP_YAML.is_file(), reason='vp map not present')
def test_eligible_and_gate_v12_not_complete():
    cfg = load_phase2d_config()
    cfg['maps_dir'] = str(MAPS)
    cfg['map_name'] = 'vp'
    model = build_coverage_model(cfg, load_descriptors=False)
    assert sum(1 for f in model.frames.values() if f.lifecycle == 'ACTIVE') >= 18

    micro = evaluate_coverage_completion(
        model, MAP_YAML, cfg, patrol_mode='micro', build_kind='AUTO_BUILD'
    )
    assert micro.eligible.eligible_visual_cells > 0
    assert micro.spatial_coverage_ratio < 0.5
    assert micro.map_complete_claim_allowed is False
    assert any('micro' in r for r in micro.gate_reasons)

    full = evaluate_coverage_completion(
        model, MAP_YAML, cfg, patrol_mode='full', build_kind='AUTO_BUILD'
    )
    assert full.gate_pass is False
    assert full.map_complete_claim_allowed is False
    gaps = remaining_gap_cells(full)
    assert len(gaps) > 0
    assert full.counts['covered'] == len(full.covered_eligible)


def test_micro_never_claims_complete_even_if_ratio_high(tmp_path, monkeypatch):
    """Guard: micro_may_claim_full_complete=false blocks claim."""
    cfg = load_phase2d_config()
    cfg['build_completion'] = dict(cfg.get('build_completion') or {})
    cfg['build_completion']['micro_may_claim_full_complete'] = False
    cfg['build_completion']['target_spatial_coverage_ratio'] = 0.0
    cfg['build_completion']['min_yaw_completeness_ratio'] = 0.0
    cfg['build_completion']['max_unresolved_nav_fail_ratio'] = 1.0
    if not MAP_YAML.is_file():
        pytest.skip('vp map not present')
    cfg['maps_dir'] = str(MAPS)
    model = build_coverage_model(cfg, load_descriptors=False)
    micro = evaluate_coverage_completion(
        model, MAP_YAML, cfg, patrol_mode='micro', build_kind='AUTO_BUILD'
    )
    # Even if spatial target is 0, micro must not claim full map complete
    assert micro.map_complete_claim_allowed is False
