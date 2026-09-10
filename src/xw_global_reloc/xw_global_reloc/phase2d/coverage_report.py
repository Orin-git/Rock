"""Coverage report writers: coverage.json + coverage_report.md."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, Optional

from xw_global_reloc.phase2d.coverage_model import CoverageModel, build_coverage_model


def build_coverage_dict(model: CoverageModel) -> Dict[str, Any]:
    base = model.baseline_stats()
    cells = []
    active_cells = set()
    cand_only_cells = set()
    for s in model.cell_summaries():
        yaw_mask = [0] * model.yaw_bins
        for b in s.covered_yaw_bins:
            if 0 <= b < model.yaw_bins:
                yaw_mask[b] = 1
        active_mask = [0] * model.yaw_bins
        for b in s.active_yaw_bins:
            if 0 <= b < model.yaw_bins:
                active_mask[b] = 1
        cand_mask = [0] * model.yaw_bins
        for b in s.candidate_yaw_bins:
            if 0 <= b < model.yaw_bins:
                cand_mask[b] = 1
        if s.active_count > 0:
            active_cells.add(s.spatial_cell)
        if s.candidate_count > 0 and s.active_count == 0:
            cand_only_cells.add(s.spatial_cell)
        cells.append(
            {
                'spatial_cell': s.spatial_cell,
                'cell_x': s.cell_x,
                'cell_y': s.cell_y,
                'active_count': s.active_count,
                'candidate_count': s.candidate_count,
                'covered_yaw_bins': s.covered_yaw_bins,
                'active_yaw_bins': s.active_yaw_bins,
                'candidate_yaw_bins': s.candidate_yaw_bins,
                'yaw_mask': yaw_mask,
                'active_yaw_mask': active_mask,
                'candidate_yaw_mask': cand_mask,
                'yaw_coverage_ratio': float(sum(yaw_mask)) / float(max(model.yaw_bins, 1)),
            }
        )

    # Candidate-added yaw bins relative to Active-only
    new_yaw_bins = 0
    for c in cells:
        for b, (a, cand) in enumerate(zip(c['active_yaw_mask'], c['candidate_yaw_mask'])):
            if cand and not a:
                new_yaw_bins += 1

    return {
        'generated_at': time.time(),
        'map_name': model.cfg.get('map_name'),
        'cell_size_m': model.cell_size_m,
        'yaw_bins': model.yaw_bins,
        'summary': {
            **base,
            'candidate_new_cells': len(cand_only_cells),
            'candidate_new_yaw_bins': new_yaw_bins,
            'duplicate_skips': int(model.stats.get('duplicate_skips', 0)),
            'novelty_captures': int(model.stats.get('novelty_captures', 0)),
            'quota_skips': int(model.stats.get('quota_skips', 0)),
            'covered_skips': int(model.stats.get('covered_skips', 0)),
            'captures': int(model.stats.get('captures', 0)),
        },
        'cells': cells,
        'note': 'Does not require full free-space coverage; reports occupied cells only.',
    }


def render_coverage_markdown(data: Dict[str, Any]) -> str:
    s = data.get('summary') or {}
    lines = [
        '# Visual Keyframe Coverage Report',
        '',
        f"- map: `{data.get('map_name')}`",
        f"- cell_size_m: **{data.get('cell_size_m')}**",
        f"- yaw_bins: **{data.get('yaw_bins')}**",
        '',
        '## Totals',
        '',
        f"- Active frames: **{s.get('active_frames')}**",
        f"- Candidate frames: **{s.get('candidate_frames')}**",
        f"- Occupied spatial cells: **{s.get('occupied_spatial_cells')}**",
        f"- Active occupied cells: **{s.get('active_occupied_cells')}**",
        f"- Candidate-only new cells: **{s.get('candidate_new_cells')}**",
        f"- Active yaw coverage ratio (over Active cells): **{s.get('active_yaw_coverage_ratio'):.3f}**",
        f"- Candidate new yaw bins (not in Active): **{s.get('candidate_new_yaw_bins')}**",
        '',
        '## Capture / Dedup stats (this model session)',
        '',
        f"- captures: {s.get('captures')}",
        f"- duplicate_skips: {s.get('duplicate_skips')}",
        f"- novelty_captures: {s.get('novelty_captures')}",
        f"- quota_skips: {s.get('quota_skips')}",
        f"- covered_skips: {s.get('covered_skips')}",
        '',
        '## Per-cell yaw masks',
        '',
        '| cell | active | candidate | yaw_mask | active_bins | cand_bins |',
        '|------|--------|-----------|----------|-------------|-----------|',
    ]
    for c in data.get('cells') or []:
        lines.append(
            f"| `{c['spatial_cell']}` | {c['active_count']} | {c['candidate_count']} | "
            f"`{''.join(str(x) for x in c['yaw_mask'])}` | {c['active_yaw_bins']} | {c['candidate_yaw_bins']} |"
        )
    lines.append('')
    lines.append(
        '_Note: A3 reports where coverage exists; it does not require every free map cell to hold a keyframe._'
    )
    lines.append('')
    return '\n'.join(lines)


def write_coverage_report(
    cfg: Dict[str, Any],
    out_dir: Optional[Path] = None,
    *,
    model: Optional[CoverageModel] = None,
) -> Dict[str, Any]:
    m = model or build_coverage_model(cfg, load_descriptors=False)
    data = build_coverage_dict(m)
    if out_dir is None:
        from xw_global_reloc.phase2d.config_loader import candidate_root_from_cfg

        out_dir = candidate_root_from_cfg(cfg) / 'reports'
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / 'coverage.json'
    md_path = out_dir / 'coverage_report.md'
    json_path.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')
    md_path.write_text(render_coverage_markdown(data), encoding='utf-8')
    return {'coverage_json': json_path, 'coverage_md': md_path, 'data': data}
