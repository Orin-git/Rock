"""Versioned Visual Active DB: resolve, atomic pointer, migrate, load helpers.

Does not change ORB/laser/retrieval algorithms. Candidate/rejected never loaded.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files


POINTER_NAME = 'current_active_version'
LEGACY_SEED = 'legacy_seed'
VERSIONS_DIR = 'versions'
CANDIDATE_DIR = 'candidate'
REJECTED_DIR = 'rejected'


def file_sha256(path: Path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def visual_root(maps_dir: Path, map_name: str) -> Path:
    return Path(maps_dir) / map_name / 'visual'


def versions_root(vroot: Path) -> Path:
    return vroot / VERSIONS_DIR


def version_dir(vroot: Path, version: str) -> Path:
    return versions_root(vroot) / version


def pointer_path(vroot: Path) -> Path:
    return vroot / POINTER_NAME


def resolve_active_root(
    maps_dir: Path,
    map_name: str,
    *,
    db_root_override: str = '',
) -> Tuple[Path, str, str]:
    """Return (db_path, version_or_label, source).

    source: explicit | current_active_version | fallback_legacy
    """
    if db_root_override and str(db_root_override).strip():
        p = Path(str(db_root_override).strip())
        return p, _version_label_from_path(p), 'explicit'

    vroot = visual_root(maps_dir, map_name)
    ptr = pointer_path(vroot)
    if ptr.is_symlink() or ptr.is_dir():
        resolved = ptr.resolve()
        if (resolved / 'manifest.yaml').is_file() and (resolved / 'descriptors' / 'index.json').is_file():
            return resolved, _version_label_from_path(resolved), 'current_active_version'

    # Controlled fallback to legacy production layout
    if (vroot / 'manifest.yaml').is_file() and (vroot / 'descriptors' / 'index.json').is_file():
        return vroot, 'legacy_production_root', 'fallback_legacy'

    return vroot, 'missing', 'missing'


def _version_label_from_path(path: Path) -> str:
    name = path.name
    if name.startswith('vp_visual_') or name.startswith('visual_'):
        return name
    # symlink target name
    try:
        if path.is_symlink():
            return Path(os.readlink(path)).name
    except OSError:
        pass
    man = path / 'manifest.yaml'
    if man.is_file():
        try:
            data = yaml.safe_load(man.read_text(encoding='utf-8')) or {}
            if data.get('version'):
                return str(data['version'])
        except Exception:  # noqa: BLE001
            pass
    return name


def next_visual_version(current: str) -> str:
    """Bump minor: vp_visual_v1.1 → vp_visual_v1.2."""
    import re

    cur = str(current or '').strip()
    m = re.match(r'^(.*_v)(\d+)\.(\d+)$', cur)
    if not m:
        raise ValueError(f'unrecognized visual version id: {current!r}')
    major = int(m.group(2))
    minor = int(m.group(3))
    return f'{m.group(1)}{major}.{minor + 1}'


def short_version_label(version: str) -> str:
    """vp_visual_v1.1 → v1.1"""
    v = str(version or '')
    if '_v' in v:
        return 'v' + v.rsplit('_v', 1)[-1]
    return v


def atomic_set_pointer(vroot: Path, version: str) -> Path:
    """Atomically point current_active_version → versions/<version>.

    Creates temp symlink in same directory then os.replace.
    """
    target = version_dir(vroot, version)
    if not target.is_dir():
        raise FileNotFoundError(f'version not found: {target}')
    if not (target / 'manifest.yaml').is_file():
        raise FileNotFoundError(f'manifest missing in {target}')
    ptr = pointer_path(vroot)
    vroot.mkdir(parents=True, exist_ok=True)
    # Relative symlink keeps tree portable
    rel_target = Path(VERSIONS_DIR) / version
    tmp = vroot / f'.{POINTER_NAME}.tmp.{os.getpid()}.{time.time_ns()}'
    if tmp.exists() or tmp.is_symlink():
        tmp.unlink()
    os.symlink(rel_target, tmp)
    # Atomic replace over existing symlink/dir entry
    os.replace(tmp, ptr)
    return ptr.resolve()


def read_pointer_version(vroot: Path) -> Optional[str]:
    ptr = pointer_path(vroot)
    if not ptr.exists() and not ptr.is_symlink():
        return None
    try:
        if ptr.is_symlink():
            return Path(os.readlink(ptr)).name
        return _version_label_from_path(ptr.resolve())
    except OSError:
        return _version_label_from_path(ptr)


def inventory_keyframe_hashes(db_root: Path) -> Dict[str, Any]:
    """Per-keyframe content hashes for equivalence checks."""
    idx_path = db_root / 'descriptors' / 'index.json'
    idx = json.loads(idx_path.read_text(encoding='utf-8')) if idx_path.is_file() else []
    frames = {}
    for item in idx:
        kid = str(item.get('id'))
        kdir = db_root / 'keyframes' / kid
        entry = {'id': kid, 'files': {}}
        for name in ('rgb.jpg', 'descriptors.npy', 'keypoints.npy', 'meta.yaml', 'depth.png', 'scan.npz'):
            p = kdir / name
            if p.is_file():
                entry['files'][name] = file_sha256(p)
        frames[kid] = entry
    return {
        'manifest_sha256': file_sha256(db_root / 'manifest.yaml')
        if (db_root / 'manifest.yaml').is_file()
        else None,
        'index_sha256': file_sha256(idx_path) if idx_path.is_file() else None,
        'keyframe_count': len(frames),
        'keyframes': frames,
        'index_ids': [str(i.get('id')) for i in idx],
    }


def copy_tree_exact(src: Path, dst: Path) -> None:
    if dst.exists():
        raise FileExistsError(f'destination exists: {dst}')
    shutil.copytree(src, dst, symlinks=False, dirs_exist_ok=False)


def build_v1_manifest(
    *,
    base_manifest: Dict[str, Any],
    map_hash: str,
    keyframe_count: int,
    spatial_cells: int,
    yaw_coverage_ratio: float,
    created_at: Optional[float] = None,
) -> Dict[str, Any]:
    """schema_version 3 while keeping Reloc-required fields."""
    out = dict(base_manifest)
    out['schema_version'] = 3
    out['version'] = 'vp_visual_v1.0'
    out['map_name'] = str(base_manifest.get('map_name') or 'vp')
    out['map_hash'] = map_hash or str(base_manifest.get('map_hash') or '')
    out['created_at'] = float(created_at if created_at is not None else time.time())
    out['keyframe_count'] = int(keyframe_count)
    out['lifecycle'] = 'ACTIVE'
    out['source'] = {'legacy_seed': True}
    out['previous_version'] = None
    out['validation_status'] = 'LEGACY_BASELINE'
    out['false_accept_count'] = 0
    out['camera_id_primary'] = 'front_up'
    out['spatial_cells'] = int(spatial_cells)
    out['yaw_coverage_ratio'] = float(yaw_coverage_ratio)
    out['production_ready'] = True
    # Keep legacy fields Reloc may read
    if 'pipeline' not in out:
        out['pipeline'] = 'visual_laser'
    if 'tiers' not in out:
        out['tiers'] = ['retrieval_ready', 'geometry_ready']
    return out


@dataclass
class MigrationResult:
    ok: bool
    visual_root: str
    legacy_seed: str
    active_version: str
    pointer: str
    pre_inventory: Dict[str, Any] = field(default_factory=dict)
    post_inventory: Dict[str, Any] = field(default_factory=dict)
    equivalence_ok: bool = False
    message: str = ''


def migrate_legacy_to_v1(
    maps_dir: Path,
    map_name: str = 'vp',
    *,
    force: bool = False,
) -> MigrationResult:
    """Copy production 37 → legacy_seed + versions/vp_visual_v1.0; set pointer.

    Never deletes the original visual/{manifest,keyframes,descriptors}.
    """
    vroot = visual_root(maps_dir, map_name)
    if not (vroot / 'manifest.yaml').is_file():
        return MigrationResult(False, str(vroot), '', '', '', message='no production manifest')

    pre = inventory_keyframe_hashes(vroot)
    if pre['keyframe_count'] != 37:
        # Still allow if force, but warn via message
        if not force and pre['keyframe_count'] < 1:
            return MigrationResult(False, str(vroot), '', '', '', pre_inventory=pre, message='empty db')

    legacy = vroot / LEGACY_SEED
    ver = version_dir(vroot, 'vp_visual_v1.0')
    (vroot / REJECTED_DIR).mkdir(parents=True, exist_ok=True)
    (vroot / CANDIDATE_DIR).mkdir(parents=True, exist_ok=True)

    if legacy.exists() and not force:
        # Idempotent if already migrated and equivalent
        if ver.exists() and pointer_path(vroot).exists():
            post = inventory_keyframe_hashes(ver)
            eq = _equiv_payload(pre, post)
            return MigrationResult(
                True,
                str(vroot),
                str(legacy),
                'vp_visual_v1.0',
                str(pointer_path(vroot)),
                pre_inventory=pre,
                post_inventory=post,
                equivalence_ok=eq,
                message='already_migrated',
            )
        return MigrationResult(
            False, str(vroot), str(legacy), '', '', pre_inventory=pre, message='legacy_seed exists'
        )

    # Copy keyframe payload pieces into legacy_seed structure
    if legacy.exists() and force:
        shutil.rmtree(legacy)
    legacy.mkdir(parents=True)
    for name in ('manifest.yaml',):
        shutil.copy2(vroot / name, legacy / name)
    shutil.copytree(vroot / 'descriptors', legacy / 'descriptors')
    shutil.copytree(vroot / 'keyframes', legacy / 'keyframes')
    if (vroot / 'state').is_dir():
        shutil.copytree(vroot / 'state', legacy / 'state')

    if ver.exists() and force:
        shutil.rmtree(ver)
    if ver.exists():
        return MigrationResult(
            False, str(vroot), str(legacy), '', '', pre_inventory=pre, message='v1.0 exists'
        )

    # Build version tree by copying legacy_seed (byte-identical keyframes)
    shutil.copytree(legacy, ver)

    # Upgrade manifest (keyframes/descriptors unchanged)
    base_man = yaml.safe_load((ver / 'manifest.yaml').read_text(encoding='utf-8')) or {}
    try:
        y, p = resolve_map_files(maps_dir, map_name)
        mhash = map_pair_hash(y, p)
    except FileNotFoundError:
        mhash = str(base_man.get('map_hash') or '')

    # Coverage sidecar via A3 if available
    spatial_cells = 9
    yaw_ratio = 0.0
    try:
        from xw_global_reloc.phase2d.config_loader import load_phase2d_config
        from xw_global_reloc.phase2d.coverage_model import CoverageModel
        from xw_global_reloc.phase2d.coverage_report import build_coverage_dict

        cfg = load_phase2d_config()
        cfg['maps_dir'] = str(maps_dir)
        cfg['map_name'] = map_name
        # Point coverage Active loader at this version dir by temporarily treating
        # production keyframes as the source — CoverageModel reads legacy production
        # path. Use production root for baseline (same 37 bytes).
        model = CoverageModel(cfg)
        model.load_legacy_active(load_descriptors=False)
        cov = build_coverage_dict(model)
        spatial_cells = int(cov['summary'].get('active_occupied_cells') or 9)
        yaw_ratio = float(cov['summary'].get('active_yaw_coverage_ratio') or 0.0)
        (ver / 'coverage.json').write_text(json.dumps(cov, indent=2) + '\n', encoding='utf-8')
    except Exception as exc:  # noqa: BLE001
        (ver / 'coverage.json').write_text(
            json.dumps(
                {
                    'summary': {
                        'active_frames': pre['keyframe_count'],
                        'active_occupied_cells': 9,
                        'note': f'coverage_fallback:{exc}',
                    }
                },
                indent=2,
            )
            + '\n',
            encoding='utf-8',
        )

    man = build_v1_manifest(
        base_manifest=base_man,
        map_hash=mhash,
        keyframe_count=int(pre['keyframe_count']),
        spatial_cells=spatial_cells,
        yaw_coverage_ratio=yaw_ratio,
    )
    (ver / 'manifest.yaml').write_text(yaml.safe_dump(man, sort_keys=False), encoding='utf-8')

    validation = {
        'status': 'LEGACY_BASELINE',
        'keyframe_count': int(pre['keyframe_count']),
        'source': 'phase2a_phase2c_existing_production',
        'false_accept_count': 0,
        'promoted_by_phase2d': False,
        'historical_evidence': [
            'PHASE2A_LASER_SCORE_RUNTIME_VALIDATION_REPORT_2026-09-07.md',
            'PHASE2C_FINAL_USABILITY_AUDIT_2026-09-10.md',
        ],
        'note': 'Not re-run in Phase2D-B1; packaged existing production DB.',
    }
    (ver / 'validation_report.json').write_text(json.dumps(validation, indent=2) + '\n', encoding='utf-8')

    atomic_set_pointer(vroot, 'vp_visual_v1.0')
    post = inventory_keyframe_hashes(ver)
    # Equivalence ignores manifest content (schema upgraded); compare keyframe files + index
    eq = _equiv_payload(pre, post, ignore_manifest=True)
    return MigrationResult(
        True,
        str(vroot),
        str(legacy),
        'vp_visual_v1.0',
        str(pointer_path(vroot)),
        pre_inventory=pre,
        post_inventory=post,
        equivalence_ok=eq,
        message='migrated',
    )


def _equiv_payload(pre: Dict[str, Any], post: Dict[str, Any], *, ignore_manifest: bool = False) -> bool:
    if pre.get('keyframe_count') != post.get('keyframe_count'):
        return False
    if pre.get('index_ids') != post.get('index_ids'):
        return False
    if not ignore_manifest and pre.get('index_sha256') != post.get('index_sha256'):
        return False
    # Index file may be identical; require same
    if pre.get('index_sha256') != post.get('index_sha256'):
        return False
    pk = pre.get('keyframes') or {}
    qk = post.get('keyframes') or {}
    if set(pk) != set(qk):
        return False
    for kid, pe in pk.items():
        qe = qk[kid]
        if pe.get('files') != qe.get('files'):
            return False
    return True


@dataclass
class LoadedDb:
    root: Path
    version: str
    source: str
    manifest: Dict[str, Any]
    keyframes: List[Dict[str, Any]]
    kf_by_id: Dict[str, Dict[str, Any]]
    db_hash: str
    error: str = ''


def load_visual_db_from_root(root: Path) -> LoadedDb:
    """Load Active DB into new lists (for atomic swap). Never reads candidate/."""
    import cv2
    import numpy as np
    from xw_global_reloc.orb_utils import unpack_keypoints

    man_p = root / 'manifest.yaml'
    if not man_p.is_file():
        return LoadedDb(root, '', '', {}, [], {}, '', error='MANIFEST_INVALID')
    try:
        manifest = yaml.safe_load(man_p.read_text(encoding='utf-8')) or {}
        if not isinstance(manifest, dict):
            return LoadedDb(root, '', '', {}, [], {}, '', error='MANIFEST_INVALID')
    except Exception:  # noqa: BLE001
        return LoadedDb(root, '', '', {}, [], {}, '', error='MANIFEST_INVALID')

    idx_path = root / 'descriptors' / 'index.json'
    if not idx_path.is_file():
        return LoadedDb(root, '', '', manifest, [], {}, '', error='INDEX_INVALID')
    try:
        idx = json.loads(idx_path.read_text(encoding='utf-8'))
        if not isinstance(idx, list):
            return LoadedDb(root, '', '', manifest, [], {}, '', error='INDEX_INVALID')
    except Exception:  # noqa: BLE001
        return LoadedDb(root, '', '', manifest, [], {}, '', error='INDEX_INVALID')

    # Refuse paths that look like candidate/rejected/legacy_seed when used as Active
    resolved = root.resolve()
    for banned in (CANDIDATE_DIR, REJECTED_DIR, LEGACY_SEED):
        if resolved.name == banned:
            return LoadedDb(root, '', '', manifest, [], {}, '', error='LOAD_FAILED')

    keyframes: List[Dict[str, Any]] = []
    kf_by_id: Dict[str, Dict[str, Any]] = {}
    for item in idx:
        kid = item['id']
        if str(kid).startswith('cand_'):
            # Hard skip any candidate ids if they somehow appear in an Active index
            continue
        kdir = root / 'keyframes' / kid
        desc_p = kdir / 'descriptors.npy'
        kp_p = kdir / 'keypoints.npy'
        meta_p = kdir / 'meta.yaml'
        if not (desc_p.is_file() and kp_p.is_file() and meta_p.is_file()):
            continue
        desc = np.load(str(desc_p))
        kps = unpack_keypoints(np.load(str(kp_p)))
        meta = yaml.safe_load(meta_p.read_text(encoding='utf-8')) or {}
        depth = cv2.imread(str(kdir / 'depth.png'), cv2.IMREAD_UNCHANGED)
        entry = {
            'id': kid,
            'descriptors': desc,
            'keypoints': kps,
            'meta': meta,
            'depth': depth,
            'retrieval_ready': bool(meta.get('retrieval_ready', True)),
            'geometry_ready': bool(meta.get('geometry_ready', depth is not None)),
            'dir': kdir,
        }
        keyframes.append(entry)
        kf_by_id[kid] = entry

    if not keyframes:
        return LoadedDb(
            root,
            str(manifest.get('version') or root.name),
            '',
            manifest,
            [],
            {},
            str(manifest.get('map_hash') or ''),
            error='LOAD_FAILED',
        )

    return LoadedDb(
        root=root,
        version=str(manifest.get('version') or _version_label_from_path(root)),
        source='',
        manifest=manifest,
        keyframes=keyframes,
        kf_by_id=kf_by_id,
        db_hash=str(manifest.get('map_hash') or ''),
        error='',
    )


def validate_version_for_activate(
    vroot: Path,
    version: str,
    *,
    maps_dir: Path,
    map_name: str,
    require_map_hash_match: bool = True,
) -> Tuple[bool, str]:
    target = version_dir(vroot, version)
    if not target.is_dir():
        return False, 'VERSION_NOT_FOUND'
    loaded = load_visual_db_from_root(target)
    if loaded.error:
        return False, loaded.error
    if require_map_hash_match:
        try:
            y, p = resolve_map_files(maps_dir, map_name)
            cur = map_pair_hash(y, p)
        except FileNotFoundError:
            cur = ''
        if loaded.db_hash and cur and loaded.db_hash != cur:
            return False, 'MAP_HASH_MISMATCH'
    return True, 'OK'
