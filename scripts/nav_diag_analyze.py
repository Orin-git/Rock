#!/usr/bin/env python3
"""Analyze nav_diag_collect.py output and print a human-readable report."""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
from typing import Any, Dict, List, Optional


def _is_num(v: Any) -> bool:
    try:
        f = float(v)
        return math.isfinite(f)
    except (TypeError, ValueError):
        return False


def _col(rows: List[dict], key: str) -> List[float]:
    out = []
    for r in rows:
        v = r.get(key, '')
        if _is_num(v):
            out.append(float(v))
    return out


def _load_csv(path: str) -> List[dict]:
    import csv
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def _moving_segments(rows: List[dict], vx_key='cmd_nav_vx', thresh=0.05) -> List[dict]:
    return [r for r in rows if _is_num(r.get(vx_key)) and abs(float(r[vx_key])) >= thresh]


def _sign_changes(vals: List[float], deadband=0.02) -> int:
    signs = []
    for v in vals:
        if abs(v) < deadband:
            continue
        signs.append(1 if v > 0 else -1)
    if len(signs) < 2:
        return 0
    return sum(1 for i in range(1, len(signs)) if signs[i] != signs[i - 1])


def _pct(n: float, d: float) -> str:
    if d <= 0:
        return 'n/a'
    return f'{100.0 * n / d:.1f}%'


def analyze(out_dir: str) -> str:
    csv_path = os.path.join(out_dir, 'samples.csv')
    meta_path = os.path.join(out_dir, 'meta.json')
    if not os.path.isfile(csv_path):
        return f'ERROR: missing {csv_path}'

    rows = _load_csv(csv_path)
    meta = {}
    if os.path.isfile(meta_path):
        with open(meta_path, encoding='utf-8') as f:
            meta = json.load(f)

    n = len(rows)
    if n < 5:
        return f'ERROR: only {n} samples — run longer or check ROS topics'

    dur = float(meta.get('duration_sec', float(rows[-1]['t']) - float(rows[0]['t'])))
    lines: List[str] = []
    lines.append('=' * 60)
    lines.append('导航诊断分析报告')
    lines.append('=' * 60)
    lines.append(f'数据目录: {out_dir}')
    lines.append(f'时长: {dur:.1f}s  样本: {n}  采样率: ~{n / max(dur, 0.1):.1f} Hz')
    if meta.get('msg_counts'):
        lines.append(f'消息计数: {meta["msg_counts"]}')
    lines.append('')

    # --- Topic health ---
    lines.append('## 1. 数据链路')
    counts = meta.get('msg_counts', {})
    expected = {
        'amcl': ('AMCL 位姿', dur * 5),
        'odom': ('EKF /odom', dur * 15),
        'scan': ('激光 /scan', dur * 8),
        'cmd_nav': ('导航速度 /cmd_vel_nav', dur * 10),
        'pc_up': ('前上点云 points_nav', dur * 3),
        'pc_down': ('前下点云 points_nav', dur * 3),
    }
    for k, (label, exp) in expected.items():
        c = counts.get(k, 0)
        flag = '✓' if c >= exp * 0.3 else ('⚠' if c > 0 else '✗')
        lines.append(f'  {flag} {label}: {c} msgs (期望 ~{exp:.0f}+)')
    pc_en = _col(rows, 'pc_nav_en')
    if pc_en:
        en_pct = 100.0 * sum(1 for v in pc_en if v >= 0.5) / len(pc_en)
        lines.append(f'  导航点云开关 pc_nav_en: {en_pct:.0f}% 时间为 true')
    pc_up_n = _col(rows, 'pc_up_n')
    pc_down_n = _col(rows, 'pc_down_n')
    if pc_up_n:
        lines.append(
            f'  前上 points_nav 点数: median={statistics.median(pc_up_n):.0f} '
            f'max={max(pc_up_n):.0f}')
    if pc_down_n:
        lines.append(
            f'  前下 points_nav 点数: median={statistics.median(pc_down_n):.0f} '
            f'max={max(pc_down_n):.0f}')
    lines.append('')

    # --- Localization ---
    lines.append('## 2. 定位 (AMCL / map↔odom 漂移)')
    cov_xx = _col(rows, 'amcl_cov_xx')
    cov_yy = _col(rows, 'amcl_cov_yy')
    cov_yaw = _col(rows, 'amcl_cov_yaw')
    if cov_xx:
        lines.append(
            f'  AMCL 协方差 median: σ²_xy=({statistics.median(cov_xx):.4f}, '
            f'{statistics.median(cov_yy):.4f}) σ²_yaw={statistics.median(cov_yaw):.4f}')
        bad_cov = sum(1 for i in range(len(cov_xx)) if cov_xx[i] > 0.5 or cov_yy[i] > 0.5)
        lines.append(f'  高协方差样本 (xy>0.5): {_pct(bad_cov, len(cov_xx))}')

    drift_m = _col(rows, 'drift_map_odom_m')
    drift_yaw = _col(rows, 'drift_map_odom_yaw_deg')
    if drift_m:
        lines.append(
            f'  map-odom TF 航向差波动: std={statistics.pstdev(drift_yaw):.2f}° '
            f'(稳定应 <3°; 跨系绝对位置差无意义，已忽略)')
        lines.append(
            f'  map-odom TF 航向差范围: '
            f'[{min(drift_yaw):.1f}°, {max(drift_yaw):.1f}°] (常数偏移正常)')

    amcl_x = _col(rows, 'amcl_x')
    amcl_y = _col(rows, 'amcl_y')
    if len(amcl_x) >= 2:
        path_len = sum(
            math.hypot(amcl_x[i] - amcl_x[i - 1], amcl_y[i] - amcl_y[i - 1])
            for i in range(1, len(amcl_x))
        )
        lines.append(f'  AMCL 轨迹长度: {path_len:.2f}m')

    loc = _col(rows, 'loc_status')
    if loc:
        for code, name in [(0, 'OK'), (1, '未就绪'), (2, '漂移/自愈'), (3, '需人工')]:
            c = sum(1 for v in loc if int(v) == code)
            if c:
                lines.append(f'  localization_status={code}({name}): {_pct(c, len(loc))}')
    lines.append('')

    # --- Stamp age ---
    lines.append('## 3. 时间戳延迟 (越大越容易导致控制抖动)')
    for key, label in [
        ('amcl_age', 'AMCL'), ('odom_age', 'Odom'), ('scan_age', 'Scan'), ('tf_map_age', 'TF map→base'),
    ]:
        vals = _col(rows, key)
        if vals:
            lines.append(
                f'  {label} age: median={statistics.median(vals)*1000:.0f}ms '
                f'p95={sorted(vals)[int(0.95*len(vals))]*1000:.0f}ms')
    lines.append('')

    # --- Control wobble ---
    lines.append('## 4. 控制抖动 (歪歪扭扭的直接指标)')
    moving = _moving_segments(rows)
    lines.append(f'  运动段样本 (|cmd_vx|≥0.05): {len(moving)}/{n}')

    wz = _col(moving, 'cmd_nav_wz') if moving else _col(rows, 'cmd_nav_wz')
    vx = _col(moving, 'cmd_nav_vx') if moving else _col(rows, 'cmd_nav_vx')
    if wz:
        lines.append(
            f'  cmd_nav 角速度 wz: std={statistics.pstdev(wz):.4f} rad/s '
            f'max|wz|={max(abs(v) for v in wz):.3f}')
        flips = _sign_changes(wz, deadband=0.03)
        lines.append(f'  wz 符号翻转次数: {flips} (多=蛇形/左右修正频繁)')
    if vx:
        lines.append(
            f'  cmd_nav 线速度 vx: mean={statistics.mean(vx):.3f} std={statistics.pstdev(vx):.4f}')

    wz_sm = _col(moving, 'cmd_sm_wz') if moving else _col(rows, 'cmd_sm_wz')
    if wz_sm and wz:
        lines.append(
            f'  smoother 前后 wz std: nav={statistics.pstdev(wz):.4f} '
            f'sm={statistics.pstdev(wz_sm):.4f}')

    imu_wz = _col(rows, 'imu_wz')
    if imu_wz:
        lines.append(f'  IMU 实测 wz std: {statistics.pstdev(imu_wz):.4f} rad/s')
    lines.append('')

    # --- Scan quality ---
    lines.append('## 5. 激光质量')
    scan_pct = _col(rows, 'scan_valid_pct')
    scan_min = _col(rows, 'scan_min_m')
    if scan_pct:
        lines.append(
            f'  有效点比例: median={statistics.median(scan_pct):.1f}% '
            f'min={min(scan_pct):.1f}%')
    if scan_min:
        low = sum(1 for v in scan_min if v < 0.35)
        lines.append(f'  最小距离 <0.35m 样本: {_pct(low, len(scan_min))} (近距遮挡/自反射)')
    lines.append('')

    # --- Diagnosis summary ---
    lines.append('## 6. 初步结论')
    issues = []
    recs = []

    if counts.get('pc_up', 0) < dur * 0.5 or counts.get('pc_down', 0) < dur * 0.5:
        issues.append('导航点云 points_nav 数据稀疏或未发布')
        recs.append('确认 supervisor 已调用 set_pointcloud_nav；检查 pc_nav_filter 输入')
    elif pc_up_n and statistics.median(pc_up_n) < 50:
        issues.append('过滤后点云过稀，局部 costmap 可能缺障碍信息')
        recs.append('适当放宽 pc_nav_filter ROI 或降低 sor/radius 强度')

    if drift_yaw and statistics.pstdev(drift_yaw) > 5.0:
        issues.append('map-odom TF 航向差波动大 (定位在漂)')
        recs.append('检查 AMCL 更新率 / 初始位姿 / EKF 航向融合')

    if wz and statistics.pstdev(wz) > 0.15:
        issues.append('角速度指令波动大 → 路径跟踪蛇形')
        recs.append('RPP: 增大 lookahead_dist / 降低 rotate_to_heading 触发；检查 closed_loop')

    if wz and _sign_changes(wz) > len(wz) * 0.15:
        issues.append('wz 频繁正负翻转')
        recs.append('RotationShim 与 RPP 可能在抢控制权；调 angular_disengage_threshold')

    amcl_ages = _col(rows, 'amcl_age')
    if amcl_ages and statistics.median(amcl_ages) > 0.25:
        issues.append('AMCL 位姿时间戳偏旧')
        recs.append('检查 CPU 负载、scan 频率、AMCL max_beams/update_min_d')

    odom_ages = _col(rows, 'odom_age')
    if odom_ages and statistics.median(odom_ages) > 0.15:
        issues.append('Odom 延迟偏高')
        recs.append('EKF frequency / 轮速话题是否稳定')

    scan_p = _col(rows, 'scan_valid_pct')
    if scan_p and statistics.median(scan_p) < 70:
        issues.append('激光有效点比例偏低')
        recs.append('检查雷达安装、地面反射、amcl laser_min_range=0.35')

    if not issues:
        issues.append('未发现单项严重异常，问题可能在参数组合或地图质量')
        recs.append('对比 local_plan 曲率、代价地图虚假障碍')

    for i, t in enumerate(issues, 1):
        lines.append(f'  [{i}] {t}')
    lines.append('')
    lines.append('## 7. 优化建议 (待实跑验证)')
    for i, t in enumerate(recs, 1):
        lines.append(f'  {i}. {t}')

    report = '\n'.join(lines)
    report_path = os.path.join(out_dir, 'report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report)
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('out_dir', help='Directory with samples.csv from nav_diag_collect')
    args = parser.parse_args()
    print(analyze(args.out_dir))


if __name__ == '__main__':
    main()
