#!/usr/bin/env python3
"""只读：候选池按 (build_session_id, map_hash) 分组 + 逐条 validation 状态。"""
import json
import collections

P = '/ros2_ws/maps/vp/visual/candidate/index.json'
d = json.load(open(P))
es = d if isinstance(d, list) else (d.get('entries') or d.get('candidates') or [])
print(f'index 类型={type(d).__name__}  条数={len(es)}')
if es:
    print('单条字段:', sorted(es[0].keys()))

cur = '35949e3f3403dc7f0253addc37afbd3c5b42b3a96d8000c35ceaa03eebce6cfd'
g = collections.Counter()
st = collections.Counter()
rows = collections.defaultdict(list)
for e in es:
    sid = e.get('build_session_id')
    h = str(e.get('map_hash') or '')
    tag = 'NEW' if h == cur else ('OLD' if h else 'NONE')
    g[(sid, tag)] += 1
    v = (e.get('validation') or {}).get('status') or e.get('status') or '(none)'
    st[(tag, v)] += 1
    rows[(sid, tag)].append((e.get('id'), (e.get('spatial_cell') or {}), e.get('yaw_bin'), v))

print('\n--- 会话 x hash ---')
for k, v in sorted(g.items(), key=lambda x: str(x[0])):
    print(f'  {k[0]}  {k[1]}: {v}')

print('\n--- 状态 x hash ---')
for k, v in sorted(st.items(), key=lambda x: str(x[0])):
    print(f'  {k[0]}  {k[1]}: {v}')

print('\n--- 新 hash 每条明细 ---')
for k, v in sorted(rows.items(), key=lambda x: str(x[0])):
    if k[1] != 'NEW':
        continue
    print(f'  [{k[0]}]  {len(v)} 条')
    for cid, cell, yb, vs in sorted(v):
        print(f'     {cid}  cell={cell} yaw={yb}  status={vs}')
