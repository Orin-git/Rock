#!/usr/bin/env python3
"""离线复算尺子 —— 带强制校准自检。

为什么有这个文件
================
2026-09-15 上一轮我用一个手写的离线复算脚本得出「DB 全部 82 帧的 scan.npz 在各自
map_pose 上都不过闸门（max 0.1832 / median 0.0272）」和「相邻帧互相不一致」两条结论。
两条都是**错的**，根因是同一类：我的手写脚本没有对齐 map_server 的坐标约定。

    bug 1: .pgm -> OccupancyGrid 漏了垂直翻转
           nav_msgs/OccupancyGrid 的 data[0] 在 origin.y（图像底部），
           而 PGM 的第 0 行在图像顶部 => 必须 np.flipud。
           漏掉之后所有帧的分数塌到 0.02~0.18 —— 正是当时报出的那个签名。

    bug 2: np.roll 滚错方向（后面重跑 R3 扫描时用到的同一类错误）

本文件把约定写死在代码里，并且**内置校准自检**：
对 45 个 NEW 帧（meta 里有 laser_score 的那些）重算，断言与记录值逐帧一致。
自检不绿 => 本工具的任何输出一律作废。这是可证伪的断言，不是信心。

用法
====
    python3 db_replay.py check                 # 只跑校准自检（必须先绿）
    python3 db_replay.py rescore               # 全 82 帧重算表 + 统计
    python3 db_replay.py sweep [kf_id ...]     # 全图粗扫：真值邻域 / 错处天花板 / 各闸门计数
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import yaml

from xw_global_reloc.laser_verify import (
    DistanceField,
    prepare_scan,
    score_scan_at_pose,
    score_scan_at_poses,
)

MAPS = '/ros2_ws/maps'
MAP_YAML = f'{MAPS}/vp.yaml'
MAP_PGM = f'{MAPS}/vp.pgm'
VROOT = f'{MAPS}/vp/visual/versions/vp_visual_v1.4'

# 闸门候选值 —— 全图扫描时同时统计
GATES = (0.38, 0.45, 0.50, 0.55)

# ---------------------------------------------------------------- 地图读取
# 约定（与 map_server / nav_msgs 一致，写死，不许改）：
#   mode: trinary, negate: 0
#     occ_prob = (255 - pixel) / 255          # negate=0：黑=占据
#     occ_prob > occupied_thresh -> 100        # 占据
#     occ_prob < free_thresh     -> 0          # 自由
#     otherwise                  -> -1         # 未知
#   data 展平顺序：行 0 在 origin.y（图像底部）=> 对 PGM 必须先 flipud
#   is_free(unknown) == False （unknown 不是 free，laser_verify.py 定义）


def read_pgm(path: str) -> np.ndarray:
    b = open(path, 'rb').read()
    toks, i = [], 0
    while len(toks) < 4:
        while b[i:i + 1].isspace():
            i += 1
        if b[i:i + 1] == b'#':
            while b[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not b[j:j + 1].isspace():
            j += 1
        toks.append(b[i:j])
        i = j
    magic, w, h = toks[0], int(toks[1]), int(toks[2])
    if magic != b'P5':
        raise SystemExit(f'只支持 P5 PGM，实际 {magic!r}')
    return np.frombuffer(b[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)


class _Grid:
    """最小 OccupancyGrid 替身 —— DistanceField 只用到这几个属性。"""

    class _Info:
        pass

    def __init__(self, px: np.ndarray, yml: dict) -> None:
        h, w = px.shape
        occ = (255.0 - px.astype(np.float64)) / 255.0        # negate = 0
        g = np.full((h, w), -1, dtype=np.int8)
        g[occ > float(yml['occupied_thresh'])] = 100
        g[occ < float(yml['free_thresh'])] = 0
        g = np.flipud(g)                                     # <<< 关键，漏了全盘皆错
        info = self._Info()
        info.resolution = float(yml['resolution'])
        info.width, info.height = w, h
        o = self._Info()
        o.position = self._Info()
        o.orientation = self._Info()
        o.position.x = float(yml['origin'][0])
        o.position.y = float(yml['origin'][1])
        o.orientation.x = o.orientation.y = o.orientation.z = 0.0
        o.orientation.w = 1.0
        info.origin = o
        self.info = info
        self.data = g.reshape(-1).tolist()
        self._g = g

    @property
    def free_cells(self) -> int:
        return int((self._g == 0).sum())


def load_field() -> DistanceField:
    return DistanceField(_Grid(read_pgm(MAP_PGM), yaml.safe_load(open(MAP_YAML))))


class _Scan:
    """scan.npz 的替身。注意：_scan_to_npz 不存 header stamp，所以这里也没有。"""

    def __init__(self, z) -> None:
        for k in ('ranges', 'angle_min', 'angle_max', 'angle_increment',
                  'range_min', 'range_max'):
            setattr(self, k, z[k])


# ---------------------------------------------------------------- 帧读取
def frames() -> list[dict]:
    out = []
    kdir = f'{VROOT}/keyframes'
    for k in sorted(os.listdir(kdir)):
        if not k.startswith('kf_'):
            continue
        m = yaml.safe_load(open(f'{kdir}/{k}/meta.yaml'))
        mp = m.get('map_pose') or {}
        if mp.get('x') is None and m.get('x') is None:
            continue
        x = float(mp['x'] if mp.get('x') is not None else m['x'])
        y = float(mp['y'] if mp.get('y') is not None else m['y'])
        yaw = float(mp['yaw'] if mp.get('yaw') is not None else m['yaw'])
        out.append({
            'id': k,
            'x': x, 'y': y, 'yaw': yaw,
            'scan': _Scan(np.load(f'{kdir}/{k}/scan.npz')),
            'recorded': (None if m.get('laser_score') is None
                         else float(m['laser_score'])),
            'fmt': 'NEW' if m.get('scan_stamp') is not None else 'OLD',
        })
    return out


def score(field, fr, gate=0.38):
    return score_scan_at_pose(field, fr['scan'], fr['x'], fr['y'], fr['yaw'],
                              beam_stride=6, match_dist_m=0.25,
                              min_valid_beams=20, min_laser_score=gate)


TOL = 1e-6


def cmd_check() -> int:
    print('=' * 78)
    print('校准自检 —— 45 个 NEW 帧重算值必须与 meta 记录的 laser_score 一致 (<1e-6)')
    print('=' * 78)
    field = load_field()
    bad, n = [], 0
    for fr in frames():
        if fr['recorded'] is None:
            continue
        n += 1
        got = float(score(field, fr).laser_score)
        d = abs(got - fr['recorded'])
        if d >= TOL:
            bad.append((fr['id'], fr['recorded'], got, d))
    print(f'  参与自检帧数 = {n}')
    if bad:
        print(f'  ❌ 不绿 —— {len(bad)}/{n} 帧对不上：')
        for kid, rec, got, d in bad[:20]:
            print(f'     {kid}  记录={rec:.6f}  重算={got:.6f}  Δ={d:.6f}')
        print('\n  ▶ 本工具的任何输出一律作废，先查地图约定。')
        return 1
    print(f'  ✅ 绿 —— {n}/{n} 帧全部一致，尺子可用。')
    return 0


def cmd_rescore() -> int:
    if cmd_check() != 0:
        return 1
    print()
    field = load_field()
    rows = []
    for fr in frames():
        rows.append((fr, float(score(field, fr).laser_score)))

    for fmt in ('OLD', 'NEW'):
        v = [s for fr, s in rows if fr['fmt'] == fmt]
        if not v:
            continue
        print('  %-4s n=%-3d min=%.4f  P25=%.4f  中位=%.4f  max=%.4f   >=0.45: %d/%d   >=0.38: %d/%d'
              % (fmt, len(v), min(v), float(np.percentile(v, 25)),
                 float(np.median(v)), max(v),
                 sum(1 for x in v if x >= 0.45), len(v),
                 sum(1 for x in v if x >= 0.38), len(v)))

    print()
    print('  低于 0.45 的帧（这些是建库前要隔离的候选）：')
    low = sorted([(fr, s) for fr, s in rows if s < 0.45], key=lambda t: t[1])
    if not low:
        print('    （无）')
    for fr, s in low:
        rec = '--' if fr['recorded'] is None else f"{fr['recorded']:.4f}"
        print(f"    {fr['id']}  {fr['fmt']}  记录={rec}  重算={s:.4f}")
    return 0


# ---------------------------------------------------------------- 全图扫描
def cmd_sweep(ids: list[str]) -> int:
    if cmd_check() != 0:
        return 1
    print()
    field = load_field()
    by_id = {fr['id']: fr for fr in frames()}
    grid = _Grid(read_pgm(MAP_PGM), yaml.safe_load(open(MAP_YAML)))
    print(f'  地图：{grid.info.width}x{grid.info.height} res={grid.info.resolution} '
          f'自由格={grid.free_cells}  origin={MAP_YAML}')
    print()

    # 粗扫网格：0.40 m x 6 deg，只取自由格
    r, ystep = 0.40, math.radians(6.0)
    xs_all = np.arange(grid.info.origin.position.x,
                       grid.info.origin.position.x + grid.info.width * grid.info.resolution, r)
    ys_all = np.arange(grid.info.origin.position.y,
                       grid.info.origin.position.y + grid.info.height * grid.info.resolution, r)
    yaws_all = np.arange(-math.pi, math.pi, ystep)
    # 向量化枚举自由格。复刻 is_free 的约定（int() 截断；x/y 均从 origin 起
    # 正向步进，故截断 == floor）。
    gx = np.maximum(xs_all, grid.info.origin.position.x)
    gy = np.maximum(ys_all, grid.info.origin.position.y)
    ix = ((gx - field.origin_x) / field.resolution).astype(np.int64)
    iy = ((gy - field.origin_y) / field.resolution).astype(np.int64)
    ix = np.clip(ix, 0, field.width - 1)
    iy = np.clip(iy, 0, field.height - 1)
    fr_mask = field.free[np.ix_(iy, ix)]          # (n_y, n_x)
    yi, xi = np.nonzero(fr_mask)
    keep_x = gx[xi]
    keep_y = gy[yi]
    n_cell = len(keep_x)
    n_yaw = len(yaws_all)
    print(f'  扫描网格：{n_cell} 个自由格 x {n_yaw} 个偏航 = {n_cell * n_yaw} 次评估')

    for kid in ids:
        fr = by_id.get(kid)
        if fr is None:
            print(f'  {kid}: 不存在，跳过')
            continue
        prep = prepare_scan(fr['scan'], beam_stride=6)
        t0 = os.times().elapsed
        CH = 40000
        chunks, poses = [], []
        for i in range(0, n_cell, CH):
            cx = keep_x[i:i + CH]
            cy = keep_y[i:i + CH]
            nx = len(cx)
            xs = np.repeat(cx.astype(np.float64), n_yaw)
            ys = np.repeat(cy.astype(np.float64), n_yaw)
            yy = np.tile(yaws_all.astype(np.float64), nx)
            sc = score_scan_at_poses(field, prep, xs, ys, yy,
                                     match_dist_m=0.25, min_valid_beams=20,
                                     min_laser_score=min(GATES))
            chunks.append(np.asarray([s.laser_score for s in sc], dtype=np.float64))
            poses.append((xs, ys, yy))
        vals = np.concatenate(chunks)
        pxs = np.concatenate([p[0] for p in poses])
        pys = np.concatenate([p[1] for p in poses])
        pyaw = np.concatenate([p[2] for p in poses])
        dt = os.times().elapsed - t0

        dist_to_truth = np.hypot(pxs - fr['x'], pys - fr['y'])
        near = dist_to_truth <= 2.0          # 真值邻域
        far = ~near                          # 错地方
        k = int(np.argmax(vals))
        at_truth = float(score(field, fr).laser_score)

        print(f'  ── {kid}  真值=({fr["x"]:.3f},{fr["y"]:.3f},{math.degrees(fr["yaw"]):.0f}°)  '
              f'该处分={at_truth:.4f}   耗时 {dt:.1f}s')
        print(f'     全局最优 {vals[k]:.4f} @ ({pxs[k]:.2f},{pys[k]:.2f},'
              f'{math.degrees(pyaw[k]):.0f}°)  距真值 {dist_to_truth[k]:.2f} m')
        print(f'     真值邻域(<=2m) 最高 = {vals[near].max():.4f}'
              if near.any() else '     真值邻域为空')
        print(f'     错地方(>2m)   最高 = {vals[far].max():.4f}'
              f'  @ 距真值 {dist_to_truth[far][int(np.argmax(vals[far]))]:.2f} m')
        print(f'     {"闸门":>7}{"通过总数":>10}{"  邻域内":>9}{"  错地方":>9}   错地方占比')
        for g in GATES:
            m = vals >= g
            n_tot, n_near = int(m.sum()), int((m & near).sum())
            n_far = n_tot - n_near
            pct = 100.0 * n_far / max(1, n_tot)
            flag = '  ← 有错地方通过' if n_far else ''
            print(f'     {g:>7.2f}{n_tot:>10d}{n_near:>9d}{n_far:>9d}'
                  f'{pct:>10.2f}%{flag}')
        print()
    return 0


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    cmd = sys.argv[1]
    if cmd == 'check':
        return cmd_check()
    if cmd == 'rescore':
        return cmd_rescore()
    if cmd == 'sweep':
        ids = sys.argv[2:] or ['kf_000032', 'kf_000061', 'kf_000069']
        return cmd_sweep(ids)
    print(__doc__)
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
