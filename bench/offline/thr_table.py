#!/usr/bin/env python3
"""闸门取值的【召回代价】表 —— 复用 db_replay 的校准尺子（先跑自检，不绿即作废）。

回答的问题与 sweep 互补：
    sweep  问「阈值定在 X，错地方会不会漏进来」   -> 安全性
    本文件 问「阈值定在 X，真值位姿会不会被拒」   -> 召回代价

两者必须一起看，才能选阈值。单看任何一个都会选出错的数。
"""
from __future__ import annotations

import sys

sys.path.insert(0, '/ros2_ws/bench/offline')

import db_replay as R  # noqa: E402

THRESHOLDS = (0.38, 0.45, 0.48, 0.50, 0.52, 0.55, 0.58, 0.60, 0.65)


def main() -> int:
    if R.cmd_check() != 0:
        print('\n▶ 自检不绿，本表作废。')
        return 1
    print()
    field = R.load_field()
    rows = []
    for fr in R.frames():
        rows.append((fr, float(R.score(field, fr).laser_score)))

    n_old = sum(1 for fr, _ in rows if fr['fmt'] == 'OLD')
    n_new = sum(1 for fr, _ in rows if fr['fmt'] == 'NEW')

    print('=' * 74)
    print('  真值位姿处的分数 —— 提高阈值会拒掉多少【合法】帧')
    print('=' * 74)
    print('  阈值      OLD 保留        NEW 保留        合计保留')
    for t in THRESHOLDS:
        o = sum(1 for fr, s in rows if fr['fmt'] == 'OLD' and s >= t)
        n = sum(1 for fr, s in rows if fr['fmt'] == 'NEW' and s >= t)
        tot, ntot = o + n, n_old + n_new
        print('  %.2f      %2d/%2d %5.1f%%    %2d/%2d %5.1f%%    %3d/%3d %5.1f%%'
              % (t, o, n_old, 100.0 * o / n_old,
                 n, n_new, 100.0 * n / n_new,
                 tot, ntot, 100.0 * tot / ntot))

    print()
    print('  会被 0.55 拒掉的合法帧（真值位姿处的分数，逐帧）：')
    low = sorted([(fr, s) for fr, s in rows if s < 0.55], key=lambda t: t[1])
    if not low:
        print('    （无）')
    for fr, s in low:
        print('    %-12s %-4s %.4f' % (fr['id'], fr['fmt'], s))
    print('    合计 %d/%d 帧 = %.1f%% 的合法关键帧'
          % (len(low), n_old + n_new, 100.0 * len(low) / (n_old + n_new)))

    print()
    print('  分位数（真值位姿处）：')
    for fmt in ('OLD', 'NEW'):
        v = sorted(s for fr, s in rows if fr['fmt'] == fmt)
        if not v:
            continue
        q = lambda p: v[min(len(v) - 1, int(p * len(v)))]  # noqa: E731
        print('    %-4s n=%-3d  P10=%.4f P25=%.4f P50=%.4f P75=%.4f' %
              (fmt, len(v), q(0.10), q(0.25), q(0.50), q(0.75)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
