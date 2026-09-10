"""Coverage model: read-only Active (versioned pointer) + Candidate inventory."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
import yaml

from xw_global_reloc.phase2d.config_loader import candidate_root_from_cfg, legacy_production_paths
from xw_global_reloc.phase2d.spatial import (
    cell_indices,
    coverage_params,
    spatial_cell_id,
    yaw_bin_index,
    yaw_delta_deg,
)
from xw_global_reloc.phase2d.version_store import resolve_active_root


@dataclass
class FrameRef:
    keyframe_id: str
    lifecycle: str  # ACTIVE | CANDIDATE
    x: float
    y: float
    yaw: float
    spatial_cell: str
    yaw_bin: int
    descriptors_path: Optional[Path] = None
    descriptors: Any = None  # optional cached np array
    timestamp: float = 0.0
    source: str = ''


@dataclass
class CellYawSlot:
    active_ids: List[str] = field(default_factory=list)
    candidate_ids: List[str] = field(default_factory=list)
    latest_capture: float = 0.0
    appearance_count: int = 0  # reserved; V1 counts frames as proxy


@dataclass
class CellSummary:
    spatial_cell: str
    cell_x: int
    cell_y: int
    active_count: int = 0
    candidate_count: int = 0
    covered_yaw_bins: List[int] = field(default_factory=list)
    active_yaw_bins: List[int] = field(default_factory=list)
    candidate_yaw_bins: List[int] = field(default_factory=list)


class CoverageModel:
    """In-memory coverage index. Never mutates legacy Active meta on disk."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        params = coverage_params(cfg)
        self.cell_size_m = float(params['cell_size_m'])
        self.yaw_bins = int(params['yaw_bins'])
        self.neighbor_cell_radius = int(params['neighbor_cell_radius'])
        self.frames: Dict[str, FrameRef] = {}
        self.slots: Dict[Tuple[str, int], CellYawSlot] = {}
        self.session_writes: int = 0
        self.stats: Dict[str, int] = {
            'duplicate_skips': 0,
            'novelty_captures': 0,
            'quota_skips': 0,
            'covered_skips': 0,
            'captures': 0,
        }

    def assign(self, x: float, y: float, yaw: float) -> Tuple[str, int]:
        return spatial_cell_id(x, y, self.cell_size_m), yaw_bin_index(yaw, self.yaw_bins)

    def _slot(self, cell: str, yaw_bin: int) -> CellYawSlot:
        key = (cell, int(yaw_bin))
        if key not in self.slots:
            self.slots[key] = CellYawSlot()
        return self.slots[key]

    def add_frame(self, ref: FrameRef, *, cache_desc: bool = False) -> None:
        if cache_desc and ref.descriptors_path is not None and ref.descriptors is None:
            try:
                if ref.descriptors_path.is_file():
                    ref.descriptors = np.load(str(ref.descriptors_path))
            except Exception:  # noqa: BLE001
                ref.descriptors = None
        self.frames[ref.keyframe_id] = ref
        slot = self._slot(ref.spatial_cell, ref.yaw_bin)
        if ref.lifecycle == 'ACTIVE':
            if ref.keyframe_id not in slot.active_ids:
                slot.active_ids.append(ref.keyframe_id)
        else:
            if ref.keyframe_id not in slot.candidate_ids:
                slot.candidate_ids.append(ref.keyframe_id)
        slot.latest_capture = max(slot.latest_capture, float(ref.timestamp or 0.0))
        slot.appearance_count = len(slot.active_ids) + len(slot.candidate_ids)

    def load_legacy_active(self, *, load_descriptors: bool = False) -> int:
        """Read-only scan of Active keyframes (prefer current_active_version)."""
        maps_dir = Path(str(self.cfg.get('maps_dir') or '/ros2_ws/maps'))
        map_name = str(self.cfg.get('map_name') or 'vp')
        active_root, version, source = resolve_active_root(maps_dir, map_name)
        if source != 'missing' and (active_root / 'keyframes').is_dir():
            kf_root = active_root / 'keyframes'
            source_tag = str(version or 'active')
        else:
            paths = legacy_production_paths(self.cfg)
            kf_root = paths['keyframes']
            source_tag = 'legacy_seed'
        n = 0
        if not kf_root.is_dir():
            return 0
        for kdir in sorted(kf_root.iterdir()):
            if not kdir.is_dir():
                continue
            meta_path = kdir / 'meta.yaml'
            if not meta_path.is_file():
                continue
            try:
                meta = yaml.safe_load(meta_path.read_text(encoding='utf-8')) or {}
            except Exception:  # noqa: BLE001
                continue
            pose = meta.get('map_pose') or {}
            try:
                x = float(pose['x'])
                y = float(pose['y'])
                yaw = float(pose['yaw'])
            except (KeyError, TypeError, ValueError):
                continue
            cell, yb = self.assign(x, y, yaw)
            kid = str(meta.get('keyframe_id') or kdir.name)
            ref = FrameRef(
                keyframe_id=kid,
                lifecycle='ACTIVE',
                x=x,
                y=y,
                yaw=yaw,
                spatial_cell=cell,
                yaw_bin=yb,
                descriptors_path=kdir / 'descriptors.npy',
                timestamp=float(meta.get('timestamp') or 0.0),
                source=source_tag,
            )
            self.add_frame(ref, cache_desc=load_descriptors)
            n += 1
        return n

    def load_candidates(self, *, load_descriptors: bool = False) -> int:
        root = candidate_root_from_cfg(self.cfg)
        kf_root = root / 'keyframes'
        n = 0
        if not kf_root.is_dir():
            return 0
        for kdir in sorted(kf_root.iterdir()):
            if not kdir.is_dir():
                continue
            meta_path = kdir / 'meta.yaml'
            if not meta_path.is_file():
                continue
            try:
                meta = yaml.safe_load(meta_path.read_text(encoding='utf-8')) or {}
            except Exception:  # noqa: BLE001
                continue
            try:
                x = float(meta.get('x'))
                y = float(meta.get('y'))
                yaw = float(meta.get('yaw'))
            except (TypeError, ValueError):
                pose = meta.get('map_pose') or {}
                try:
                    x = float(pose['x'])
                    y = float(pose['y'])
                    yaw = float(pose['yaw'])
                except (KeyError, TypeError, ValueError):
                    continue
            cell = str(meta.get('spatial_cell') or '') or spatial_cell_id(x, y, self.cell_size_m)
            yb = meta.get('yaw_bin')
            if yb is None:
                yb = yaw_bin_index(yaw, self.yaw_bins)
            else:
                yb = int(yb)
            kid = str(meta.get('keyframe_id') or kdir.name)
            ref = FrameRef(
                keyframe_id=kid,
                lifecycle='CANDIDATE',
                x=x,
                y=y,
                yaw=yaw,
                spatial_cell=cell,
                yaw_bin=yb,
                descriptors_path=kdir / 'descriptors.npy',
                timestamp=float(meta.get('timestamp') or 0.0),
                source=str(meta.get('source') or 'candidate'),
            )
            self.add_frame(ref, cache_desc=load_descriptors)
            n += 1
        return n

    def reload(self, *, load_descriptors: bool = False) -> Dict[str, int]:
        self.frames.clear()
        self.slots.clear()
        a = self.load_legacy_active(load_descriptors=load_descriptors)
        c = self.load_candidates(load_descriptors=load_descriptors)
        return {'active': a, 'candidate': c}

    def cell_has_any(self, cell: str) -> bool:
        for (c, _), slot in self.slots.items():
            if c == cell and (slot.active_ids or slot.candidate_ids):
                return True
        return False

    def cell_yaw_covered(self, cell: str, yaw_bin: int) -> bool:
        slot = self.slots.get((cell, int(yaw_bin)))
        if slot is None:
            return False
        return bool(slot.active_ids or slot.candidate_ids)

    def count_cell_yaw(self, cell: str, yaw_bin: int) -> int:
        slot = self.slots.get((cell, int(yaw_bin)))
        if slot is None:
            return 0
        return len(slot.active_ids) + len(slot.candidate_ids)

    def neighbor_cells(self, cell: str) -> List[str]:
        try:
            cx, cy = cell_indices(0, 0, self.cell_size_m)  # placeholder
        except Exception:  # noqa: BLE001
            cx = cy = 0
        # parse from id
        from xw_global_reloc.phase2d.spatial import parse_spatial_cell

        try:
            cx, cy = parse_spatial_cell(cell)
        except ValueError:
            return [cell]
        r = max(0, int(self.neighbor_cell_radius))
        out = []
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                out.append(f'cell_{cx + dx}_{cy + dy}')
        return out

    def frames_near(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        max_xy_m: float,
        max_yaw_deg: float,
        include_active: bool = True,
        include_candidate: bool = True,
    ) -> List[FrameRef]:
        cell, _ = self.assign(x, y, yaw)
        cells = set(self.neighbor_cells(cell))
        out: List[FrameRef] = []
        for fr in self.frames.values():
            if fr.spatial_cell not in cells:
                continue
            if fr.lifecycle == 'ACTIVE' and not include_active:
                continue
            if fr.lifecycle == 'CANDIDATE' and not include_candidate:
                continue
            dist = math_hypot(fr.x - x, fr.y - y)
            if dist > float(max_xy_m):
                continue
            if yaw_delta_deg(fr.yaw, yaw) > float(max_yaw_deg):
                continue
            out.append(fr)
        return out

    def nearest_frame(self, x: float, y: float, yaw: float) -> Tuple[Optional[FrameRef], float, float]:
        best: Optional[FrameRef] = None
        best_d = float('inf')
        best_dyaw = float('inf')
        cell, _ = self.assign(x, y, yaw)
        neighborhood = set(self.neighbor_cells(cell))
        pool = [fr for fr in self.frames.values() if fr.spatial_cell in neighborhood]
        if not pool:
            pool = list(self.frames.values())
        for fr in pool:
            d = math_hypot(fr.x - x, fr.y - y)
            dy = yaw_delta_deg(fr.yaw, yaw)
            if d < best_d or (abs(d - best_d) < 1e-9 and dy < best_dyaw):
                best = fr
                best_d = d
                best_dyaw = dy
        if best is None:
            return None, float('inf'), float('inf')
        return best, best_d, best_dyaw

    def cell_summaries(self) -> List[CellSummary]:
        by_cell: Dict[str, CellSummary] = {}
        for (cell, yb), slot in self.slots.items():
            if cell not in by_cell:
                try:
                    from xw_global_reloc.phase2d.spatial import parse_spatial_cell

                    cx, cy = parse_spatial_cell(cell)
                except ValueError:
                    cx = cy = 0
                by_cell[cell] = CellSummary(spatial_cell=cell, cell_x=cx, cell_y=cy)
            s = by_cell[cell]
            s.active_count += len(slot.active_ids)
            s.candidate_count += len(slot.candidate_ids)
            if slot.active_ids:
                s.active_yaw_bins.append(int(yb))
            if slot.candidate_ids:
                s.candidate_yaw_bins.append(int(yb))
            if slot.active_ids or slot.candidate_ids:
                s.covered_yaw_bins.append(int(yb))
        for s in by_cell.values():
            s.active_yaw_bins = sorted(set(s.active_yaw_bins))
            s.candidate_yaw_bins = sorted(set(s.candidate_yaw_bins))
            s.covered_yaw_bins = sorted(set(s.covered_yaw_bins))
        return sorted(by_cell.values(), key=lambda c: (c.cell_x, c.cell_y))

    def baseline_stats(self) -> Dict[str, Any]:
        cells = self.cell_summaries()
        active_cells = [c for c in cells if c.active_count > 0]
        cand_cells = [c for c in cells if c.candidate_count > 0]
        n_active = sum(1 for f in self.frames.values() if f.lifecycle == 'ACTIVE')
        n_cand = sum(1 for f in self.frames.values() if f.lifecycle == 'CANDIDATE')
        yaw_occ = 0
        yaw_possible = max(len(active_cells), 1) * self.yaw_bins
        for c in active_cells:
            yaw_occ += len(c.active_yaw_bins)
        return {
            'active_frames': n_active,
            'candidate_frames': n_cand,
            'occupied_spatial_cells': len(cells),
            'active_occupied_cells': len(active_cells),
            'candidate_occupied_cells': len(cand_cells),
            'yaw_bins': self.yaw_bins,
            'active_yaw_bin_occupancy': yaw_occ,
            'active_yaw_coverage_ratio': float(yaw_occ) / float(yaw_possible) if active_cells else 0.0,
            'cell_size_m': self.cell_size_m,
        }


def math_hypot(a: float, b: float) -> float:
    return float((a * a + b * b) ** 0.5)


def build_coverage_model(cfg: Dict[str, Any], *, load_descriptors: bool = False) -> CoverageModel:
    m = CoverageModel(cfg)
    m.reload(load_descriptors=load_descriptors)
    return m
