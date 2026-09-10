"""Load phase2d_v1.yaml with defaults."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


_DEFAULTS: Dict[str, Any] = {
    'maps_dir': '/ros2_ws/maps',
    'map_name': 'vp',
    'camera_id': 'front_up',
    'candidate_root': '',
    'pose_gate': {
        'require_localization_status_0': True,
        'require_phase2c_ready': True,
        'allowed_phase2c_states': ['READY'],
        'blocked_phase2c_states': [
            'BOOT_LOCALIZING',
            'LOST',
            'RECOVERING',
            'NEED_OPERATOR',
            'VERIFYING_OPERATOR_POSE',
            'DEGRADED',
            'UNKNOWN',
        ],
        'block_if_follow_active': True,
        'block_if_legacy_freeze_active': True,
        'max_amcl_cov_xy': 0.6,
        'max_amcl_cov_yaw': 0.35,
        'max_map_base_age_sec': 1.5,
        'max_map_odom_age_sec': 1.5,
        'max_scan_age_sec': 1.0,
        'max_speed_mps': 0.20,
        'max_yaw_rate': 0.35,
        'require_finite_pose': True,
        'require_map_name': True,
        'require_map_hash': True,
    },
    'laser_gate': {
        'min_laser_score': 0.38,
        'min_valid_beams': 20,
        'match_dist_m': 0.25,
        'beam_stride': 6,
    },
    'image_gate': {
        'min_sharpness': 40.0,
        'brightness_min': 25.0,
        'brightness_max': 230.0,
        'max_underexposure_ratio': 0.45,
        'max_overexposure_ratio': 0.35,
        'underexpose_thresh': 20,
        'overexpose_thresh': 235,
        'min_orb_features': 80,
        'expected_min_width': 160,
        'expected_min_height': 120,
    },
    'orb': {'n_features': 1000},
    'capture': {
        'source_default': 'manual_test',
        'jpeg_quality': 90,
        'save_scan': True,
        'arm_timeout_sec': 3.0,
        'min_translation_m': 0.75,
        'min_yaw_deg': 35.0,
        'min_visual_diff': 0.30,
    },
    'coverage': {
        'cell_size_m': 1.0,
        'yaw_bins': 8,
        'neighbor_cell_radius': 1,
    },
    'dedup': {
        'max_xy_m': 0.35,
        'max_yaw_deg': 15.0,
        'max_visual_diff_for_duplicate': 0.25,
    },
    'candidate_limits': {
        'max_per_cell_yaw': 5,
        'max_per_build_session': 500,
    },
    'patrol': {
        'max_yaw_targets_per_cell': 2,
        'max_total_goals': 40,
        'max_planning_rounds': 2,
        'max_session_sec': 1800,
        'goal_clearance_m': 0.45,
        'free_kernel_cells': 2,
        'cell_sample_stride': 1,
        'prefer_near_existing_m': 8.0,
        'settle_sec': 1.0,
        'capture_retry': 1,
        'nav_timeout_sec': 120.0,
        'micro_max_goals': 6,
        'mode_default': 'micro',
        'use_nav2_action': True,
        'simulate_nav': False,
    },
    'auto_build': {
        'patrol_mode_default': 'micro',
        'micro_max_goals': 6,
        'max_planning_rounds': 2,
        'max_session_sec': 1800,
    },
}


def _deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    out = deepcopy(base)
    for k, v in (overlay or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = deepcopy(v)
    return out


def default_config_path() -> Path:
    here = Path(__file__).resolve()
    # .../xw_global_reloc/xw_global_reloc/phase2d/config_loader.py
    pkg = here.parents[2]  # xw_global_reloc package root (src/xw_global_reloc)
    cand = pkg / 'config' / 'phase2d_v1.yaml'
    if cand.is_file():
        return cand
    share = Path('/ros2_ws/install/xw_global_reloc/share/xw_global_reloc/config/phase2d_v1.yaml')
    if share.is_file():
        return share
    return cand


def load_phase2d_config(path: Optional[Path] = None) -> Dict[str, Any]:
    cfg_path = Path(path) if path else default_config_path()
    raw: Dict[str, Any] = {}
    if cfg_path.is_file():
        data = yaml.safe_load(cfg_path.read_text(encoding='utf-8')) or {}
        if isinstance(data, dict):
            raw = data.get('phase2d_v1') or data
            if not isinstance(raw, dict):
                raw = {}
    return _deep_merge(_DEFAULTS, raw)


def candidate_root_from_cfg(cfg: Dict[str, Any]) -> Path:
    explicit = str(cfg.get('candidate_root') or '').strip()
    if explicit:
        return Path(explicit)
    maps_dir = Path(str(cfg.get('maps_dir') or '/ros2_ws/maps'))
    map_name = str(cfg.get('map_name') or 'vp')
    return maps_dir / map_name / 'visual' / 'candidate'


def production_visual_root(cfg: Dict[str, Any]) -> Path:
    maps_dir = Path(str(cfg.get('maps_dir') or '/ros2_ws/maps'))
    map_name = str(cfg.get('map_name') or 'vp')
    return maps_dir / map_name / 'visual'


def legacy_production_paths(cfg: Dict[str, Any]) -> Dict[str, Path]:
    root = production_visual_root(cfg)
    return {
        'visual_root': root,
        'manifest': root / 'manifest.yaml',
        'index': root / 'descriptors' / 'index.json',
        'keyframes': root / 'keyframes',
    }
