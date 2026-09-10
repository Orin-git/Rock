#!/usr/bin/env python3
"""Write coverage.json + coverage_report.md from current Active+Candidate inventory."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from xw_global_reloc.phase2d.config_loader import load_phase2d_config
from xw_global_reloc.phase2d.coverage_report import write_coverage_report


def main(argv=None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', default='')
    p.add_argument('--config', default='')
    args = p.parse_args(argv)
    cfg = load_phase2d_config(Path(args.config) if args.config else None)
    out = write_coverage_report(cfg, Path(args.out_dir) if args.out_dir else None)
    summary = (out.get('data') or {}).get('summary') if isinstance(out.get('data'), dict) else None
    # write_coverage_report returns paths; rebuild summary for print
    from xw_global_reloc.phase2d.coverage_model import build_coverage_model
    from xw_global_reloc.phase2d.coverage_report import build_coverage_dict

    data = build_coverage_dict(build_coverage_model(cfg))
    print(json.dumps(data['summary'], indent=2))
    print('wrote', out['coverage_json'])
    print('wrote', out['coverage_md'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
