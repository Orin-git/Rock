#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""odom_cov_norm.py —— `/odom` 协方差规范化 relay（P3A L3 实验专用，只读转发）

为什么存在
==========
见 /home/cjy/189_p3a_offline/PHASE3_L3_ODOM_COV_ROOTFIX_2026-09-29.md

rtabmap 把 `/odom` 的 **twist covariance** 直接当作图里邻接边（Link type=0）的
6DOF 信息矩阵 —— 三条维度【数值精确匹配】：
    x: 1/0.004571  = 218.78 ≈ DB 边信息 218.79
    y: 1/9.456e10  = 1.0575e-11 = DB 边信息 1.0575e-11
    z: 1/9.9867e-07= 1.0013e6 ≈ DB 边信息 1.00134e6
（反证排除 pose：1/7.7e18 = 1.3e-19 ≠ 218.79）

而 EKF（robot_localization）的 `odom0_config` 只勾 vx + vyaw ⇒ vy 是纯外推状态
⇒ 其方差无界增长、饱和在 ~1e11 ⇒ y 方向信息量 ~1e-11 ⇒ GTSAM 抛
`IndeterminantLinearSystemException`（underconstrained）⇒ rtabmap 每次图优化
都被 `Rtabmap.cpp:3919 Graph optimization failed! Rejecting last loop closures added`
回滚 ⇒ **回环一加进图就被撤**。
（跨 16 轮逐点单调，阈值 ~4.4e-11 ~ 6.2e-11；自变量是 EKF 运行时长，不是重启。）

本节点做什么
============
【只封上界，不抬高】：
  · header（stamp / frame_id / child_frame_id）、pose.pose、twist.twist
    —— 全部逐位原样转发，位姿一个字节都不改
  · covariance 只做 cov[i] = min(cov[i], CAP[i])；非对角项原样保留
    （rtabmap 侧 Mem/CovOffDiagIgnored=true，实测非对角全 0）
  · ★ **不发 TF**：odom→base_link 仍由 EKF 发布（绝不能出现第二个 TF 源）
  · 只订阅 in_topic、只发布 out_topic，不碰任何别的话题

默认 CAP（方案 §4.1）
====================
  twist: vx 1e-2 | vy 1.0 | vz 1e-2 | vroll 1e-2 | vpitch 1e-2 | vyaw 1e-2
  pose : x 1.0  | y  1.0 | z  1e-2 | roll  1e-2 | pitch  1e-2 | yaw  1.0
  ★ vy 取 1.0（而非与 vx 同级的 0.0046）＝**最小干预**：
    y 信息相对 x 弱 218 倍 ⇒ 不会主导优化；
    相对病态值 1e-11 强 1e11 倍 ⇒ 彻底脱离危险区。

用法
====
  source /ros2_ws/scripts/ros_env.sh        # ★ 不 source 会跑在 domain 0（静默收不到）
  python3 /ros2_ws/p3a_l3/odom_cov_norm.py \\
      --ros-args -p in_topic:=/odom -p out_topic:=/odom_cov_norm
  # 只封 twist、不动 pose：      -p cap_pose:=false
  # 改 vy 上限（与 x 同级档）：  -p twist_caps:="[1e-2, 218.8, 1e-2, 1e-2, 1e-2, 1e-2]"

自检
====
  启动后每 report_period 秒打一行心跳：收/发计数、最近一帧年龄、各分量被压过的次数。
  ★ 若「收 0」持续 —— 十有八九是没 source ros_env.sh（domain 0）。

作者注：本文件是【新增文件】，不在任何 launch 里；不会被自动起。
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from nav_msgs.msg import Odometry

# 6x6 row-major 的对角下标
DIAG = (0, 7, 14, 21, 28, 35)
NAMES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')

# ★ 实测：/odom 由 ekf_filter_node 发 RELIABLE / KEEP_LAST(10) / VOLATILE
#   ⇒ 订阅端用 RELIABLE 与发布端一致（无损）；发布端也用 RELIABLE
#     （RELIABLE 发布者能同时被 RELIABLE 与 BEST_EFFORT 订阅者匹配）
_QOS = QoSProfile(depth=10,
                  reliability=ReliabilityPolicy.RELIABLE,
                  history=HistoryPolicy.KEEP_LAST,
                  durability=DurabilityPolicy.VOLATILE)


def cap_inplace(cov, caps):
    """cov: 长度 36 的 row-major 6x6（原地改）。只压对角项【上界】。

    返回被改动过的分量标签列表：
      'y'  = 原值 > 上限，被压下来
      'y!' = 原值是 NaN 或非正（会给出"无穷信息"）⇒ 兜底给了上限（★ 守卫，正常不触发）
    """
    hit = []
    for k, i in enumerate(DIAG):
        v = float(cov[i])
        c = float(caps[k])
        if not (v == v) or v <= 0.0:      # NaN 或 ≤0
            cov[i] = c
            hit.append(NAMES[k] + '!')
        elif v > c:
            cov[i] = c
            hit.append(NAMES[k])
    return hit


def _diag_str(cov):
    """协方差（长 36）的 6 个对角项。"""
    return ' '.join('%s=%.6g' % (NAMES[k], float(cov[DIAG[k]])) for k in range(6))


def _caps_str(caps):
    """★ 上限表（长 6，不是协方差）—— 别再拿 _diag_str 套它（会 IndexError）。"""
    return ' '.join('%s=%.6g' % (NAMES[k], float(caps[k])) for k in range(6))


class OdomCovNorm(Node):

    def __init__(self):
        super().__init__('odom_cov_norm')
        self.declare_parameter('in_topic', '/odom')
        self.declare_parameter('out_topic', '/odom_cov_norm')
        self.declare_parameter('cap_twist', True)
        self.declare_parameter('cap_pose', True)
        self.declare_parameter('twist_caps', [1e-2, 1.0, 1e-2, 1e-2, 1e-2, 1e-2])
        self.declare_parameter('pose_caps', [1.0, 1.0, 1e-2, 1e-2, 1e-2, 1.0])
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('stale_warn', 3.0)

        g = lambda n: self.get_parameter(n).value
        self.in_topic = str(g('in_topic'))
        self.out_topic = str(g('out_topic'))
        self.cap_twist_on = bool(g('cap_twist'))
        self.cap_pose_on = bool(g('cap_pose'))
        self.twist_caps = [float(x) for x in g('twist_caps')]
        self.pose_caps = [float(x) for x in g('pose_caps')]
        self.report_period = float(g('report_period'))
        self.stale_warn = float(g('stale_warn'))

        if len(self.twist_caps) != 6 or len(self.pose_caps) != 6:
            raise ValueError('twist_caps / pose_caps 必须各是 6 个数')

        self.n_recv = 0
        self.n_pub = 0
        self.last_rx = 0.0
        self.hit_twist = {}
        self.hit_pose = {}
        self.guard_fired = 0
        self.t0 = time.time()

        self.pub = self.create_publisher(Odometry, self.out_topic, _QOS)
        self.create_subscription(Odometry, self.in_topic, self.cb, _QOS)
        self.create_timer(self.report_period, self.tick)

        self.get_logger().info('=' * 74)
        self.get_logger().info('odom_cov_norm 起： %s --> %s' % (self.in_topic, self.out_topic))
        self.get_logger().info('  twist 封顶 %s : %s'
                               % ('开' if self.cap_twist_on else '关', _caps_str(self.twist_caps)))
        self.get_logger().info('  pose  封顶 %s : %s'
                               % ('开' if self.cap_pose_on else '关', _caps_str(self.pose_caps)))
        self.get_logger().info('  ★ 不发 TF（odom->base_link 仍由 EKF 发）；位姿逐位原样转发')
        self.get_logger().info('=' * 74)

    # ------------------------------------------------------------------
    def cb(self, msg):
        self.n_recv += 1
        self.last_rx = time.time()

        out = Odometry()
        out.header = msg.header                    # stamp / frame_id 逐位原样
        out.child_frame_id = msg.child_frame_id    # 逐位原样
        out.pose.pose = msg.pose.pose              # 位姿逐位原样（一个字节都不改）
        out.twist.twist = msg.twist.twist

        tc = [float(x) for x in msg.twist.covariance]
        pc = [float(x) for x in msg.pose.covariance]

        thr = None
        phr = None
        if self.cap_twist_on:
            thr = cap_inplace(tc, self.twist_caps)
            for h in thr:
                self.hit_twist[h] = self.hit_twist.get(h, 0) + 1
                if h.endswith('!'):
                    self.guard_fired += 1
        if self.cap_pose_on:
            phr = cap_inplace(pc, self.pose_caps)
            for h in phr:
                self.hit_pose[h] = self.hit_pose.get(h, 0) + 1
                if h.endswith('!'):
                    self.guard_fired += 1

        out.twist.covariance = tc
        out.pose.covariance = pc
        self.pub.publish(out)
        self.n_pub += 1

        if self.n_recv == 1:
            self.get_logger().info('★ 首帧（原值 → 封顶后）')
            self.get_logger().info('    twist 原: %s' % _diag_str(msg.twist.covariance))
            self.get_logger().info('    twist 后: %s' % _diag_str(tc))
            self.get_logger().info('    pose  原: %s' % _diag_str(msg.pose.covariance))
            self.get_logger().info('    pose  后: %s' % _diag_str(pc))
            self.get_logger().info('    本次压过: twist=%s pose=%s' % (thr, phr))

    # ------------------------------------------------------------------
    def tick(self):
        now = time.time()
        age = (now - self.last_rx) if self.last_rx else float('nan')
        fmt = lambda d: ' '.join('%s:%d' % (k, v) for k, v in sorted(d.items())) or '—'
        self.get_logger().info(
            '心跳 %4.0fs | 收 %d 发 %d | 最近帧 %.2fs 前 | 压过 twist[%s] pose[%s] | 守卫触发 %d'
            % (now - self.t0, self.n_recv, self.n_pub, age,
               fmt(self.hit_twist), fmt(self.hit_pose), self.guard_fired))
        if self.n_recv == 0:
            self.get_logger().warn('★ 一条都没收到 —— 检查是否 source 了 /ros2_ws/scripts/ros_env.sh'
                                   '（漏 source 会跑在 domain 0）以及 %s 是否在发' % self.in_topic)
        elif age > self.stale_warn:
            self.get_logger().warn('★ 输入已静默 %.1fs（> %.1fs）' % (age, self.stale_warn))

    def final_report(self):
        self.get_logger().info('=' * 74)
        self.get_logger().info('收尾： 收 %d / 发 %d，运行 %.0fs'
                               % (self.n_recv, self.n_pub, time.time() - self.t0))
        fmt = lambda d: ' '.join('%s:%d' % (k, v) for k, v in sorted(d.items())) or '—'
        self.get_logger().info('  累计压过 twist[%s] pose[%s]  守卫触发 %d'
                               % (fmt(self.hit_twist), fmt(self.hit_pose), self.guard_fired))
        if self.n_recv and self.n_recv != self.n_pub:
            self.get_logger().warn('★ 收 %d ≠ 发 %d  ⇒ 有丢帧，需查明' % (self.n_recv, self.n_pub))
        self.get_logger().info('=' * 74)


def main():
    rclpy.init()
    node = OdomCovNorm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.final_report()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
