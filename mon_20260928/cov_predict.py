#!/usr/bin/env python3
"""只读预测：15 条新 hash 候选能否让 coverage_increase 通过。

用生产代码自己的 _active_cells_yaw（与 run_c1 同一路径），不是我自己重写。
"""
import json
import sys
from pathlib import Path

from xw_global_reloc.phase2d.c1_validate_promote import _active_cells_yaw

VIS = Path('/ros2_ws/maps/vp/visual')
ACTIVE = VIS / 'current_active_version'
resolved = Path(str(ACTIVE)).resolve()
print(f'active 指针: {ACTIVE} -> {resolved}')

active_cells, active_yaw = _active_cells_yaw(resolved)
print(f'v1.6 实测: len(active_cells)={len(active_cells)}  len(active_yaw)={len(active_yaw)}')

idx = json.load(open(VIS / 'candidate' / 'index.json'))
es = idx if isinstance(idx, list) else (idx.get('entries') or idx.get('candidates') or [])
cur = '35949e3f3403dc7f0253addc37afbd3c5b42b3a96d8000c35ceaa03eebce6cfd'
new = [e for e in es if str(e.get('map_hash') or '') == cur]
print(f'候选总数={len(es)}  新 hash={len(new)}')

proj_cells = set(active_cells)
proj_yaw = set(active_yaw)
print('\n--- 逐条：是否带来新覆盖 ---')
for e in sorted(new, key=lambda x: str(x.get('id'))):
    cid = e.get('id')
    cell = str(e.get('spatial_cell'))
    yb = int(e.get('yaw_bin'))
    nc = cell not in active_cells
    ny = (cell, yb) not in active_yaw
    proj_cells.add(cell)
    proj_yaw.add((cell, yb))
    print(f'  {cid}  cell={cell:<14} yaw={yb}  新cell={nc}  新(cell,yaw)={ny}')

print(f'\n投影后: cells {len(active_cells)} -> {len(proj_cells)}   '
      f'yaw {len(active_yaw)} -> {len(proj_yaw)}')
cov_up = len(proj_cells) > len(active_cells) or len(proj_yaw) > len(active_yaw)
print(f'★ coverage_increase = {cov_up}')

# 再看每条候选的 meta.yaml 里已有的 validation 状态（只读）
print('\n--- 每条候选 meta.yaml 的 validation 状态 ---')
K = VIS / 'candidate' / 'keyframes'
import collections
st = collections.Counter()
for e in sorted(new, key=lambda x: str(x.get('id'))):
    mp = K / str(e.get('id')) / 'meta.yaml'
    s = '(meta.yaml 缺)'
    if mp.exists():
        try:
            import yaml
            m = yaml.safe_load(mp.read_text()) or {}
            v = m.get('validation') or {}
            s = f"{v.get('status')}  reasons={v.get('reasons')}"
        except Exception as ex:
            s = f'(读失败 {ex})'
    st[s.split()[0]] += 1
    print(f'  {e.get("id")}  {s}')
print(f'\n状态汇总: {dict(st)}  (分母={len(new)})')
