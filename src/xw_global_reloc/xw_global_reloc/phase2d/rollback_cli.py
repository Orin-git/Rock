#!/usr/bin/env python3
"""Rollback Visual Active DB to a previous version (transactional pointer + reload)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from xw_global_reloc.phase2d.set_active_cli import set_active_version


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='Rollback Visual Active DB version')
    p.add_argument('version', help='Target version e.g. vp_visual_v1.0')
    p.add_argument('--maps-dir', default='/ros2_ws/maps')
    p.add_argument('--map-name', default='vp')
    p.add_argument('--no-reload', action='store_true')
    p.add_argument('--timeout', type=float, default=30.0)
    args = p.parse_args(argv)
    out = set_active_version(
        maps_dir=Path(args.maps_dir),
        map_name=args.map_name,
        version=args.version,
        reload=not args.no_reload,
        timeout=args.timeout,
    )
    out['action'] = 'rollback'
    print(json.dumps(out, indent=2))
    return 0 if out.get('status') in ('OK', 'POINTER_OK') else 1


if __name__ == '__main__':
    raise SystemExit(main())
