#!/usr/bin/env python3
"""R3 干跑取证：在【已知良好的位姿】上调一次 /xw/relocalize，看 R3 自己能不能过。

只回答一个问题，且是决定性的一刀：
    当机器人正确定位时，R3（视觉检索 + 激光精配）能否产出一个过 0.38 闸门的候选？
      能  -> R3 机器本身没坏，00:19/00:37 的 no_survivor 是【上游位姿错误】的下游后果
      不能-> R3 机器在位姿正确时也坏，病在 R3 自己身上

安全性（这是关键，逐条）
------------------------
  apply_initial_pose=False  -> 服务端在 ACCEPT 时会走 1028-1032 提前 return DRY_RUN_OK，
                               **绝不发 /initialpose**，AMCL 不会被改动
  allow_motion=False        -> 不产生任何运动指令
  前后各取一次 /amcl_pose + health_detail + initialpose_owner，自证「什么都没动」

只读、单次、无副作用。不做任何判定修改。
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from xw_interfaces.srv import Relocalize

RC = {0: 'READY', 1: 'UNKNOWN', 2: 'REJECTED', 3: 'AMCL_TIMEOUT',
      4: 'NO_DATA', 5: 'MAP_HASH_MISMATCH', 6: 'DRY_RUN_OK'}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class S(Node):
    def __init__(self):
        super().__init__('r3_dryrun_tmp')
        self.amcl = self.detail = self.owner = self.scan = None
        latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._a, latch)
        self.create_subscription(String, '/xw/localization/health_detail',
                                 self._d, latch)
        self.create_subscription(String, '/xw/localization/initialpose_owner',
                                 self._o, latch)

    def _a(self, m):
        self.amcl = m

    def _d(self, m):
        self.detail = str(m.data)

    def _o(self, m):
        self.owner = str(m.data)


def snap(node, label):
    """取一组位姿/健康快照（等新样本）。"""
    node.amcl = None
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 6.0 and node.amcl is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    out = {'label': label}
    if node.amcl is not None:
        p = node.amcl.pose.pose
        out['amcl'] = (round(float(p.position.x), 4),
                       round(float(p.position.y), 4),
                       round(math.degrees(yaw_of(p.orientation)) % 360, 2))
    if node.detail:
        try:
            d = json.loads(node.detail)
            la = d.get('laser') or {}
            out['health'] = {'status': d.get('status'),
                             'raw': d.get('raw'),
                             'laser_score': la.get('score'),
                             'beams': la.get('beams')}
        except Exception:  # noqa: BLE001
            out['health_raw'] = node.detail[:120]
    if node.owner:
        out['initialpose_owner'] = node.owner
    return out


def show_topk(diag):
    tk = diag.get('topk') or []
    if not tk:
        print('  topk: 空（检索一个候选都没出）')
        return
    print(f'  topk 候选数 = {len(tk)}')
    hdr = (f'    {"id":<12}{"rank":>5}{"retr":>8}{"ratio":>6}{"region":<16}'
           f'{"laser":>8}{"beams":>6}{"match":>7}{"mean_d":>8}'
           f'{"hypot":>7}{"dyaw°":>7}  reason')
    print(hdr)
    for c in tk:
        dx, dy = float(c.get('dx') or 0.0), float(c.get('dy') or 0.0)
        print(f'    {str(c.get("id")):<12}{c.get("retrieval_rank"):>5}'
              f'{float(c.get("retrieval_score") or 0):>8.4f}'
              f'{c.get("ratio_matches"):>6}'
              f'{str(c.get("visual_region"))[:15]:<16}'
              f'{float(c.get("laser_score") or 0):>8.4f}'
              f'{c.get("valid_beams"):>6}'
              f'{float(c.get("matched_ratio") or 0):>7.3f}'
              f'{float(c.get("mean_dist") or 0):>8.3f}'
              f'{math.hypot(dx, dy):>7.3f}'
              f'{math.degrees(float(c.get("dyaw") or 0)):>7.1f}  '
              f'{c.get("laser_reason")}')
    ls = [float(c.get('laser_score') or 0) for c in tk]
    print(f'    laser_score: max={max(ls):.4f} min={min(ls):.4f} '
          f'中位={sorted(ls)[len(ls) // 2]:.4f}')
    # 边界钉死统计：位移是否顶到搜索窗外沿
    pinned = [c for c in tk
              if math.hypot(float(c.get('dx') or 0), float(c.get('dy') or 0)) > 1.0]
    print(f'    位移 >1.0m（粗搜半径）的候选数 = {len(pinned)}/{len(tk)}'
          f'  ← 这些是「真峰在窗外」的签名')


def main():
    rclpy.init()
    n = S()
    print('=' * 78)
    print('R3 干跑 —— apply_initial_pose=False / allow_motion=False / 单次')
    print('=' * 78)

    before = snap(n, 'BEFORE')
    print(f'  [调用前] {json.dumps(before, ensure_ascii=False)}')
    if before.get('amcl'):
        print('  （这就是「已知良好」的位姿：health 的 laser.score 应为高位）')

    cli = n.create_client(Relocalize, '/xw/relocalize')
    if not cli.wait_for_service(timeout_sec=15.0):
        print('  ABORT: /xw/relocalize 不可达')
        n.destroy_node(); rclpy.shutdown(); sys.exit(3)

    req = Relocalize.Request()
    req.map_name = 'vp'
    req.force_visual = True
    req.max_candidates = 10
    req.apply_initial_pose = False     # <<< 关键：绝不播种
    req.allow_motion = False           # <<< 关键：绝不运动

    t0 = time.time()
    fut = cli.call_async(req)
    print(f'  [调用中] 已发出，等待应答（最多 240s；10 个候选×精配 2-7s/个）...')
    while rclpy.ok() and time.time() - t0 < 240.0 and not fut.done():
        rclpy.spin_once(n, timeout_sec=0.2)
    dt = time.time() - t0

    if not fut.done():
        print(f'  ABORT: {dt:.1f}s 无应答')
        n.destroy_node(); rclpy.shutdown(); sys.exit(4)
    res = fut.result()
    code = int(res.result_code)
    print()
    print('-' * 78)
    print(f'  耗时 = {dt:.1f}s')
    print(f'  success        = {bool(res.success)}')
    print(f'  result_code    = {code}  ({RC.get(code, "?")})')
    print(f'  candidate_count= {res.candidate_count}')
    print(f'  visual_score   = {float(res.visual_score):.4f}')
    print(f'  laser_score    = {float(res.laser_score):.4f}   ← 过闸门的那一个的分数')
    print(f'  composite      = {float(res.composite_score):.4f}')
    if res.stage_timings_json:
        print(f'  stage_timings  = {res.stage_timings_json}')
    print('-' * 78)

    diag = None
    if res.diagnostics_json:
        try:
            diag = json.loads(res.diagnostics_json)
        except Exception as exc:  # noqa: BLE001
            print(f'  diagnostics_json 解析失败: {exc}')
            print(f'  原文: {res.diagnostics_json}')

    if diag is not None:
        print('  diagnostics 顶层键 =', sorted(diag.keys()))
        print(f'  decision = {diag.get("decision")}   reason = {diag.get("reason")}')
        print(f'  pipeline_mode = {diag.get("pipeline_mode")}  '
              f'accept_policy = {diag.get("accept_policy")}')
        ca = diag.get('cluster_accept')
        if ca:
            print(f'  cluster_accept = {json.dumps(ca, ensure_ascii=False)}')
        sg = diag.get('sensor_gate')
        if sg:
            print(f'  sensor_gate = {json.dumps(sg, ensure_ascii=False)[:400]}')
        print()
        show_topk(diag)

    after = snap(n, 'AFTER')
    print()
    print(f'  [调用后] {json.dumps(after, ensure_ascii=False)}')
    print()
    print('  ▶ 副作用自检（必须：位姿未变、owner 未变）')
    if before.get('amcl') and after.get('amcl'):
        d = math.hypot(after['amcl'][0] - before['amcl'][0],
                       after['amcl'][1] - before['amcl'][1])
        print(f'    ΔAMCL 平移 = {d:.4f} m  '
              f'{"✅ 未动" if d < 0.02 else "❌ 动了！"}')
        print(f'    yaw {before["amcl"][2]}° -> {after["amcl"][2]}°')
    print(f'    initialpose_owner 前={before.get("initialpose_owner")!r} '
          f'后={after.get("initialpose_owner")!r}')

    print()
    print('=' * 78)
    print('  ▶ 判据')
    print('=' * 78)
    if code == 6:
        print(f'    **DRY_RUN_OK** —— R3 在【位姿正确】时能过闸门，过闸门的分数 '
              f'{float(res.laser_score):.4f}')
        print('    => R3 机器没坏。00:19/00:37 的 no_survivor 是【上游位姿错误】')
        print('       的下游后果，改法应落在定位/级联，不在这套检索+精配上。')
    elif code == 1:
        print(f'    **UNKNOWN/no_survivor（{diag.get("reason") if diag else "?"}）**')
        print('    => 即便机器人【位姿已知正确】，R3 依然产出不了过闸门的候选。')
        print('       病在 R3 自己身上（检索 or 精配窗），与位姿误差无关。')
    elif code == 2:
        print('    **REJECTED** —— 有候选但被否，看上面 cluster_accept 的具体理由。')
    else:
        print(f'    result_code={code}（{RC.get(code, "?")}），见上。')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
