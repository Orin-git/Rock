#!/usr/bin/env python3
"""Migrate production Visual DB → legacy_seed + Active vp_visual_v1.0 + pointer."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from xw_global_reloc.phase2d.version_store import migrate_legacy_to_v1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='Phase2D-B1 Visual DB migration')
    p.add_argument('--maps-dir', default='/ros2_ws/maps')
    p.add_argument('--map-name', default='vp')
    p.add_argument('--force', action='store_true')
    p.add_argument('--out-json', default='')
    args = p.parse_args(argv)

    result = migrate_legacy_to_v1(Path(args.maps_dir), args.map_name, force=args.force)
    payload = {
        'ok': result.ok,
        'message': result.message,
        'visual_root': result.visual_root,
        'legacy_seed': result.legacy_seed,
        'active_version': result.active_version,
        'pointer': result.pointer,
        'equivalence_ok': result.equivalence_ok,
        'pre_keyframe_count': result.pre_inventory.get('keyframe_count'),
        'post_keyframe_count': result.post_inventory.get('keyframe_count'),
        'pre_index_sha256': result.pre_inventory.get('index_sha256'),
        'post_index_sha256': result.post_inventory.get('index_sha256'),
    }
    text = json.dumps(payload, indent=2)
    print(text)
    if args.out_json:
        outp = Path(args.out_json)
        outp.parent.mkdir(parents=True, exist_ok=True)
        outp.write_text(text + '\n', encoding='utf-8')
    return 0 if result.ok and result.equivalence_ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
