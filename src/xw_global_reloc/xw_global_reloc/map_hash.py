"""Map file hash binding for keyframe DB."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Optional, Tuple


def file_sha256(path: Path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def map_pair_hash(yaml_path: Path, pgm_path: Optional[Path] = None) -> str:
    """Hash yaml + pgm so Relocalizer rejects DB after map edits."""
    parts = [f'yaml:{file_sha256(yaml_path)}']
    if pgm_path is None:
        # Prefer image field in yaml if present; else sibling .pgm
        try:
            text = yaml_path.read_text(encoding='utf-8')
            for line in text.splitlines():
                if line.strip().startswith('image:'):
                    img = line.split(':', 1)[1].strip()
                    cand = (yaml_path.parent / img).resolve()
                    if cand.is_file():
                        pgm_path = cand
                    break
        except OSError:
            pass
        if pgm_path is None:
            sibling = yaml_path.with_suffix('.pgm')
            if sibling.is_file():
                pgm_path = sibling
    if pgm_path is not None and pgm_path.is_file():
        parts.append(f'pgm:{file_sha256(pgm_path)}')
    return hashlib.sha256('|'.join(parts).encode('utf-8')).hexdigest()


def resolve_map_files(maps_dir: Path, map_name: str) -> Tuple[Path, Path]:
    """Support flat maps/vp.yaml and maps/vp/navigation/vp.yaml."""
    flat_yaml = maps_dir / f'{map_name}.yaml'
    flat_pgm = maps_dir / f'{map_name}.pgm'
    nested_yaml = maps_dir / map_name / 'navigation' / f'{map_name}.yaml'
    nested_pgm = maps_dir / map_name / 'navigation' / f'{map_name}.pgm'
    if flat_yaml.is_file():
        pgm = flat_pgm if flat_pgm.is_file() else flat_yaml.with_suffix('.pgm')
        return flat_yaml, pgm
    if nested_yaml.is_file():
        pgm = nested_pgm if nested_pgm.is_file() else nested_yaml.with_suffix('.pgm')
        return nested_yaml, pgm
    raise FileNotFoundError(f'map yaml not found for {map_name} under {maps_dir}')
