#!/usr/bin/env python3
"""Step 2 acceptance table for the running/finished Phase2D RESUME session.

One command, seven rows. Everything here is measured — the trace is the same
latched status topic the node publishes, the session JSON is what the node
persisted, the ledger is the file the gate reads.

Read-only.
"""
import hashlib
import json
import re
import time
from collections import Counter, defaultdict
from pathlib import Path

STATE = Path('/ros2_ws/maps/vp/visual/state')
TRACE = Path('/ros2_ws/log/build_status_trace.log')
LEDGER = STATE / 'nav_fail_history.json'
LEDGER_BASELINE = '1de9e08323f4cea169ad870d022a01ca'  # measured at session start
TS = re.compile(r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)\]\s+\(\s*([\d.]+)s\)\s+state=(\S+)\s+msg=(.*?)\s+nav=(\S+)/(\S+)\s+'
                r'new_cells=(\S+)\s+covered=(\S+)\s+eligible=(\S+)\s+spatial=(\S+)\s*$')
KEYS = ('nav%', 'new_cells', 'covered', 'eligible', 'spatial')


def secs(hhmmss):
    h, m, s = (int(x) for x in hhmmss.split(':'))
    return h * 3600 + m * 60 + s


def read_trace():
    """Return [(t, state, msg, nav_reached, nav_planned, new_cells, covered, ...)]."""
    out = []
    if not TRACE.is_file():
        return out
    for ln in TRACE.read_text(encoding='utf-8', errors='replace').splitlines():
        m = TS.match(ln)
        if not m:
            continue
        g = m.groups()
        nav = (g[4], g[5]) if g[4] != 'None' else (None, None)
        out.append((secs(g[0].split(' ')[1]), g[0], g[2], g[3].strip(), *nav, *g[6:10]))
    return out


def time_in_state(rows):
    """Attribute wall-clock to each state.

    The trace only writes on change, so a state's duration is the gap to the
    NEXT line that has a different state — not the next line, which may be a
    message change inside the same state.
    """
    tot = defaultdict(float)
    for i, r in enumerate(rows):
        nxt = next((x for x in rows[i + 1:] if x[2] != r[2]), None)
        end = nxt[0] if nxt else rows[-1][0]
        tot[r[2]] += max(0.0, end - r[0])
    return tot


rows = read_trace()
print('=' * 78)
print('STEP 2  ACCEPTANCE  —  Phase2D RESUME session')
print('=' * 78)

# ---------------------------------------------------------------- Fix-1
print('\n[Fix-1] exclude_pairs: does round >= 2 still get FRESH goals?')
print('        (yesterday: stop_reason=planner_no_new_goals after 2 goals)')
# Filenames are `build_YYYYMMDD_HHMMSS_<hex>`, so NAME order IS time order.
# Sorting by mtime picks yesterday's file while the live session is still in
# round 1 — its JSON is only written at round boundaries, never continuously.
sessions = sorted(STATE.glob('build_2026*.json'))
trace_last = time.mktime(time.strptime(rows[-1][1], '%Y-%m-%d %H:%M:%S')) if rows else 0.0
if not sessions:
    print('        no session files on disk at all')
    s = None
else:
    f = sessions[-1]
    s = json.loads(f.read_text(encoding='utf-8'))
    age = trace_last - f.stat().st_mtime if trace_last else 0.0
    print(f'        newest on disk = {f.name}')
    print(f'        file mtime {time.strftime("%F %T", time.localtime(f.stat().st_mtime))}'
          f'  vs last trace line {rows[-1][1] if rows else "n/a"}'
          f'  => {age:.0f}s behind')
    if age > 120:
        print('        ** STALE: the live session has NOT been persisted yet (round 1 '
              'still running). Every goals[] row below is YESTERDAY\'s data. **')
    print(f'        state={s.get("state")}  stop_reason={s.get("stop_reason")!r}  '
          f'mode={s.get("mode")}  patrol_mode={s.get("patrol_mode")}')

plan_lines = [r for r in rows if r[3].startswith('round=')]
print(f'        PLANNING rounds seen in trace: '
      f'{[r[3] for r in plan_lines] or "none yet"}')
print(f'        final nav = {rows[-1][4]}/{rows[-1][5]}' if rows else '        (no trace)')

goals = (s or {}).get('goals') or []
if goals:
    print(f'        goals persisted = {len(goals)}')
    print(f'        goal keys       = {sorted(goals[0].keys())}')

# ---------------------------------------------------------------- Fix-2
print('\n[Fix-2] cancel semantics: cancelled goals must NOT be NAV_FAILED')
if goals:
    navc = Counter(str(g.get('nav')) for g in goals)
    print(f'        goals[].nav     = {dict(navc)}')
    bad = [g for g in goals if str(g.get('nav')) == 'NAV_FAILED'
           and str(g.get('nav_result_code')) in ('CANCELLED', 'INTERLOCK', 'ABORTED')]
    print(f'        cancelled-but-recorded-as-NAV_FAILED = {len(bad)}'
          f'   (want 0)')
    print(f'        nav_result_code = {dict(Counter(str(g.get("nav_result_code")) for g in goals))}')
else:
    print('        (no persisted goals yet)')

# ---------------------------------------------------------------- Fix-2 elapsed
print('\n[Fix-2] nav_elapsed_sec present on EVERY goal')
if goals:
    el = [g.get('nav_elapsed_sec') for g in goals if isinstance(g.get('nav_elapsed_sec'), (int, float))]
    print(f'        present = {len(el)}/{len(goals)}'
          + (f'   min={min(el):.2f}s  max={max(el):.2f}s' if el else ''))
else:
    print('        (no persisted goals yet)')

# ---------------------------------------------------------------- Fix-2b
print('\n[Fix-2b] stable-window gate: sub-1s NAV2_STATUS:6 should be gone')
print('        (yesterday: 14/14 aborts at 0.16-0.95 s)')
if goals:
    fast = [g for g in goals
            if str(g.get('nav')) == 'NAV_FAILED'
            and isinstance(g.get('nav_elapsed_sec'), (int, float))
            and g['nav_elapsed_sec'] < 1.0]
    print(f'        NAV_FAILED with nav_elapsed_sec < 1.0 = {len(fast)}   (want ~0)')
    for g in fast[:10]:
        print(f'          {g.get("spatial_cell")} yaw={g.get("yaw_bin")} '
              f'{g.get("nav_elapsed_sec"):.2f}s code={g.get("nav_result_code")}')
else:
    print('        (no persisted goals yet)')

# ---------------------------------------------------------------- Fix-3b
print('\n[Fix-3b] ledger invariant: no new entries, especially no cancel codes')
if LEDGER.is_file():
    raw = LEDGER.read_bytes()
    md5 = hashlib.md5(raw).hexdigest()
    ok = md5 == LEDGER_BASELINE
    print(f'        md5 = {md5}   baseline = {LEDGER_BASELINE}   '
          f'{"UNCHANGED ✅" if ok else "CHANGED ❌"}')
    print(f'        content = {raw.decode("utf-8", "replace")[:300]}')
else:
    print('        ledger missing')

# ---------------------------------------------------------------- Fix-4
print('\n[Fix-4] planner_starved fallback: diagnostics must land on disk')
diags = sorted(STATE.glob('diag_planner_starved_*.json'))
print(f'        diag_planner_starved_*.json found = {len(diags)}'
      f'   (0 is PASS unless the session actually starved)')
for d in diags[-3:]:
    print(f'          {d.name}  {d.stat().st_size} B')

# ---------------------------------------------------------------- throughput
print('\n[throughput] covered growth + state time share')
# The trace's `covered=` comes from the latched completion_summary, which the node
# only recomputes at ROUND BOUNDARIES — measured 2026-09-11: it sat at 30 for the
# whole of round 1 while the DB was already at 38. So the trace column cannot show
# growth within a round. Measure the DB itself; fall back to the trace if the
# coverage model is unavailable.
cov = [(r[0], r[7]) for r in rows if r[7] not in (None, 'None')]
if cov:
    print(f'        trace covered= column: first={cov[0][1]}  last={cov[-1][1]}'
          f'   (frozen mid-round by design — not evidence of no growth)')
try:
    from xw_global_reloc.phase2d.build_completion import _active_covered_and_yaw
    from xw_global_reloc.phase2d.config_loader import load_phase2d_config
    from xw_global_reloc.phase2d.coverage_model import build_coverage_model
    _m = build_coverage_model(load_phase2d_config(), load_descriptors=False)
    _covered, _yaw = _active_covered_and_yaw(_m)
    # `_yaw` is Dict[cell, Set[yaw_bin]] — a non-empty set is NOT the gate's
    # "yaw sufficient" (that needs enough bins per cell). Report what is
    # measured; do not re-derive a pass/fail the gate owns.
    print(f'        DB MEASURED NOW: covered={len(_covered)}  '
          f'cells_with_any_yaw={sum(1 for v in _yaw.values() if v)}  '
          f'total_yaw_bins={sum(len(v) for v in _yaw.values())}   <- the real numbers')
except Exception as exc:  # noqa: BLE001
    print(f'        DB measurement unavailable ({type(exc).__name__}: {exc}); '
          f'trace column above is all we have')
tot = time_in_state(rows)
grand = sum(tot.values()) or 1.0
for st in ('PATROLLING', 'COLLECTING', 'PAUSED', 'PLANNING', 'PRECHECK', 'IDLE'):
    if st in tot:
        print(f'        {st:<12} {tot[st]:7.1f}s  {100*tot[st]/grand:5.1f}%')
paused = [r for r in rows if r[2] == 'PAUSED']
if paused:
    print(f'        PAUSED reasons = {dict(Counter(r[3] for r in paused))}')

print('\n' + '=' * 78)
print(f'trace lines = {len(rows)}   last = '
      f'{rows[-1][1] + " " + rows[-1][2] + " " + rows[-1][3] if rows else "none"}')
print('=' * 78)
