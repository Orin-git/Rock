"""Candidate-only disk writer. Never touches production visual/keyframes."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import yaml

from xw_global_reloc.orb_utils import OrbFrame, pack_keypoints
from xw_global_reloc.phase2d.config_loader import candidate_root_from_cfg, production_visual_root


class CandidateWriter:
    """Append-only writer under maps/<map>/visual/candidate/."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self._cfg = cfg
        self.root = candidate_root_from_cfg(cfg)
        self.keyframes_dir = self.root / 'keyframes'
        self.index_path = self.root / 'index.json'
        self.stats_path = self.root / 'capture_stats.json'
        self._lock = threading.Lock()
        self.stats: Dict[str, int] = {
            'accepted': 0,
            'rejected_pose': 0,
            'rejected_laser': 0,
            'rejected_blur': 0,
            'rejected_dark': 0,
            'rejected_overexposed': 0,
            'rejected_low_features': 0,
            'rejected_invalid_frame': 0,
            'skip_duplicate': 0,
            'skip_covered': 0,
            'skip_cell_yaw_quota': 0,
            'skip_session_quota': 0,
            'capture_visual_novelty': 0,
            'dry_run': 0,
        }

    def ensure_dirs(self) -> None:
        self.keyframes_dir.mkdir(parents=True, exist_ok=True)
        if not self.index_path.is_file():
            self.index_path.write_text('[]\n', encoding='utf-8')

    def assert_not_production(self, path: Path) -> None:
        prod = production_visual_root(self._cfg).resolve()
        kf = (prod / 'keyframes').resolve()
        resolved = path.resolve()
        if resolved == kf or kf in resolved.parents:
            raise RuntimeError(f'refused write into production keyframes: {resolved}')
        # Also refuse writing manifest/index of production visual root
        if resolved == (prod / 'manifest.yaml').resolve():
            raise RuntimeError('refused write into production manifest')
        if resolved == (prod / 'descriptors' / 'index.json').resolve():
            raise RuntimeError('refused write into production index')

    def _next_id(self) -> str:
        ts = time.strftime('%Y%m%d_%H%M%S')
        # short monotonic suffix
        suffix = int(time.time() * 1000) % 1_000_000
        return f'cand_{ts}_{suffix:06d}'

    def _load_index(self) -> List[Dict[str, Any]]:
        if not self.index_path.is_file():
            return []
        try:
            data = json.loads(self.index_path.read_text(encoding='utf-8'))
            return data if isinstance(data, list) else []
        except json.JSONDecodeError:
            return []

    def record_reject(self, bucket: str) -> None:
        with self._lock:
            if bucket in self.stats:
                self.stats[bucket] += 1
            self._flush_stats()

    def record_dry_run(self) -> None:
        with self._lock:
            self.stats['dry_run'] += 1
            self._flush_stats()

    def _flush_stats(self) -> None:
        self.ensure_dirs()
        self.assert_not_production(self.stats_path)
        self.stats_path.write_text(json.dumps(self.stats, indent=2) + '\n', encoding='utf-8')

    def write_candidate(
        self,
        *,
        bgr: np.ndarray,
        orb: OrbFrame,
        meta: Dict[str, Any],
        scan_npz: Optional[Dict[str, Any]] = None,
        dry_run: bool = False,
    ) -> Dict[str, Any]:
        """Write Candidate artifacts. dry_run skips disk and returns meta only."""
        if dry_run:
            self.record_dry_run()
            out = dict(meta)
            out['dry_run'] = True
            out['written'] = False
            return out

        with self._lock:
            self.ensure_dirs()
            kid = str(meta.get('keyframe_id') or self._next_id())
            meta = dict(meta)
            meta['keyframe_id'] = kid
            meta['lifecycle'] = 'CANDIDATE'
            kdir = self.keyframes_dir / kid
            self.assert_not_production(kdir)
            kdir.mkdir(parents=True, exist_ok=True)

            jpg = kdir / 'rgb.jpg'
            desc = kdir / 'descriptors.npy'
            kps = kdir / 'keypoints.npy'
            meta_path = kdir / 'meta.yaml'
            for p in (jpg, desc, kps, meta_path):
                self.assert_not_production(p)

            quality = int(self._cfg.get('capture', {}).get('jpeg_quality', 90))
            cv2.imwrite(str(jpg), bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            np.save(str(desc), orb.descriptors)
            np.save(str(kps), pack_keypoints(orb.keypoints))

            if scan_npz is not None and bool(self._cfg.get('capture', {}).get('save_scan', True)):
                scan_path = kdir / 'scan.npz'
                self.assert_not_production(scan_path)
                np.savez_compressed(str(scan_path), **scan_npz)
                meta['scan_ref'] = 'scan.npz'

            meta_path.write_text(yaml.safe_dump(meta, sort_keys=False), encoding='utf-8')

            index = self._load_index()
            index.append(
                {
                    'id': kid,
                    'lifecycle': 'CANDIDATE',
                    'pose': {'x': meta.get('x'), 'y': meta.get('y'), 'yaw': meta.get('yaw')},
                    'spatial_cell': meta.get('spatial_cell'),
                    'yaw_bin': meta.get('yaw_bin'),
                    'map_name': meta.get('map_name'),
                    'map_hash': meta.get('map_hash'),
                    'laser_score': meta.get('laser_score'),
                    'orb_features': (meta.get('image_quality') or {}).get('orb_features'),
                    'source': meta.get('source'),
                    'timestamp': meta.get('timestamp'),
                    'capture_reason': (meta.get('capture_decision') or {}).get('reason'),
                }
            )
            self.assert_not_production(self.index_path)
            self.index_path.write_text(json.dumps(index, indent=2) + '\n', encoding='utf-8')
            self.stats['accepted'] += 1
            self._flush_stats()
            return {
                'keyframe_id': kid,
                'path': str(kdir),
                'written': True,
                'dry_run': False,
                'meta': meta,
            }


def production_index_ids(cfg: Dict[str, Any]) -> List[str]:
    root = production_visual_root(cfg)
    idx = root / 'descriptors' / 'index.json'
    if not idx.is_file():
        return []
    data = json.loads(idx.read_text(encoding='utf-8'))
    return [str(e.get('id')) for e in data if isinstance(e, dict)]
