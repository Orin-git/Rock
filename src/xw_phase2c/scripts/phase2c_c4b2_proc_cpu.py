#!/usr/bin/env python3
"""Fixed-window /proc/<pid>/stat CPU% for Phase2C resident nodes.

Does not use ps lifetime average. Samples utime+stime over WINDOW_SEC.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

WANT = (
    'boot_localizer',
    'lost_recovery',
    'last_good_pose_writer',
    'global_reloc_poc',
    'charger_prior',
)


def clk_tck() -> float:
    return float(os.sysconf('SC_CLK_TCK') or 100)


def iter_procs() -> List[Tuple[int, str]]:
    out = []
    for ent in os.listdir('/proc'):
        if not ent.isdigit():
            continue
        pid = int(ent)
        try:
            raw = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\x00', b' ').decode(
                'utf-8', 'replace'
            )
        except OSError:
            continue
        if not raw.strip():
            continue
        out.append((pid, raw))
    return out


def match_wanted(cmd: str) -> Optional[str]:
    for w in WANT:
        if w in cmd and 'python' in cmd:
            return w
    return None


def read_ticks(pid: int) -> Optional[int]:
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().split()
    except OSError:
        return None
    # comm may contain spaces; utime is field 14 (1-based) after comm ends.
    # After the last ')' :
    try:
        rparen = Path(f'/proc/{pid}/stat').read_text().rindex(')')
        rest = Path(f'/proc/{pid}/stat').read_text()[rparen + 2 :].split()
        # state=0 ... utime is index 11, stime 12 (0-based after comm)
        utime = int(rest[11])
        stime = int(rest[12])
        return utime + stime
    except (OSError, ValueError, IndexError):
        return None


def sample(window_sec: float = 45.0) -> Dict:
    procs = []
    for pid, cmd in iter_procs():
        key = match_wanted(cmd)
        if key:
            procs.append({'pid': pid, 'key': key, 'cmd': cmd[:180]})
    t0 = {p['pid']: read_ticks(p['pid']) for p in procs}
    time.sleep(window_sec)
    t1 = {p['pid']: read_ticks(p['pid']) for p in procs}
    hz = clk_tck()
    rows = []
    for p in procs:
        a, b = t0.get(p['pid']), t1.get(p['pid'])
        if a is None or b is None:
            cpu = None
        else:
            cpu = 100.0 * (b - a) / (hz * window_sec)
        rows.append({**p, 'cpu_pct': None if cpu is None else round(cpu, 2)})
    return {
        'window_sec': window_sec,
        'clk_tck': hz,
        'samples': rows,
        'by_key': {r['key']: r['cpu_pct'] for r in rows},
    }


def main() -> None:
    window = float(os.environ.get('CPU_WINDOW_SEC', '45'))
    out = sample(window)
    text = json.dumps(out, indent=2)
    print(text)
    dest = os.environ.get('CPU_OUT', '')
    if dest:
        Path(dest).write_text(text + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
