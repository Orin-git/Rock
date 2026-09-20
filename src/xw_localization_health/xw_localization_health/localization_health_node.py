#!/usr/bin/env python3
"""Gen2 localization health 0–3 + optional self-heal (spin + reinitialize).

0 good | 1 not ready | 2 drift (self-heal) | 3 needs intervention (latched until OK)

AMCL only republishes after motion (update_min_*). While NAV/FOLLOW/recovery is
armed, this node periodically calls request_nomotion_update so amcl_pose stays
live even when the robot is stopped. While odom is nearly static since the last
amcl_pose, that pose is still treated as usable (avoids idle→1).
A stopped robot is still judged: a live scan that fails the frozen 0.38 laser
gate at the current pose is status 3, without waiting for motion.
A live scan that fails the frozen 0.38 laser gate at that pose is not normal,
even if the robot has not moved. That check is low-rate and does not spin.

Phase1: detection is always active (incl. FOLLOW). Execution of spin/reinit is
gated by /xw/localization/recovery_enable so follow is never preempted by
health motion — supervisor stops follow first, then arms recovery.
"""

from __future__ import annotations

import json
import math
from copy import deepcopy
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from xw_global_reloc.laser_verify import (
    DistanceField,
    ray_consistency_at_pose,
    score_scan_at_pose,
)
from xw_interfaces.msg import RobotEvent


_MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class LocalizationHealthNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_localization_health')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_stale_sec', 1.0)
        self.declare_parameter('amcl_stale_sec', 2.0)
        # AMCL only publishes after update_min_*; while odom motion since the last
        # amcl_pose stays below these, reuse that pose (do not force status 1).
        self.declare_parameter('amcl_static_trans_m', 0.12)
        self.declare_parameter('amcl_static_yaw_rad', 0.12)
        self.declare_parameter('cov_xy_warn', 0.8)
        self.declare_parameter('cov_xy_bad', 2.5)
        self.declare_parameter('cov_yaw_warn', 0.35)
        self.declare_parameter('cov_yaw_bad', 0.8)
        # ---- ⑥-A（2026-09-18）：协方差**下界** ----
        # 上面四个 cov_* 全是**上界**（>= bad 才报）。而本轮的失效模式是**虚假的确定**：
        # 静止 + 强制更新 ⇒ AMCL 在零运动信息下反复「传感器更新 + 重采样」⇒ 粒子云
        # 单调收缩，cov 落到 ~1e-7 乃至 1e-14，四道闸门全部轻松通过，网页显示「正常」。
        # 闸门要的是「cov 大 = 我可能错了」，而塌缩产出的是「cov ≈ 0 = 我确信」——
        # 后者恰好是上界闸门唯一认得的「健康」形状。所以必须有下界。
        #
        # ⚠️ 关于符号 —— **更正我自己先前写在这段里的一句错话**（红线 12 留痕）：
        #    我原先写「任何 abs()/max(0,·) 都会把要观测的东西抹掉」。写单测时把它验了，
        #    对**这个下界比较**而言那句话是**错的**：实测塌缩值 c7 = -1.467e-14，
        #    取 abs 得 1.467e-14、取 max(0,·) 得 0.0，三者在 3e-5 这个阈值下
        #    **给出同一个判决**（都远小于阈值）。测试 `T1::test_sign_does_not_...
        #    __documented` 把这个「不变」钉住了。
        #    符号真正决定的是另外两件事：
        #      ① **遥测**：`cov.xy_raw` 必须带符号，负号才说明滤波器的方差已落到
        #         浮点噪声里（= 平台期，不会再降），而不是「正停在一个很小但真实的
        #         协方差上」。A10 那个 `-0.0` 就是这个意思 —— 5 位小数的打印精度
        #         让两种完全不同的状态长得一模一样。
        #      ② **「负值 = 关闭该轴」这个约定**：它只有在按原值比较时才成立。
        #         若写成 `lim != 0.0` 而不是 `lim > 0.0`，一个负的限值会被当成
        #         「启用」，于是 `xy < lim` 在负值区反而**成立** ⇒ 误判塌缩。
        #         测试 `T4::test_negative_limit_near_floor_disables_axis` 钉住这一条
        #         （限值取在 -1e-14，与实测塌缩值同量级 —— 取 -1.0 是分不开两者的）。
        #
        # 取值来源（Step 0 标定，2026-09-18 上午；**单次停车事件**，留痕见方案）：
        #   ⓐ 正常停在目标点 +20 s（建库每格停留就是这个量级）⇒ 1.77e-3 —— 不得触发
        #   ⓑ 停放 +300 s ⇒ 3.53e-7（其后 ≥190 s 稳定在 3.4~4.2e-7）    —— 必须触发
        #   ③ 平台期最大的注入尖峰 ⇒ cov_xy 1.14e-5 / cov_yaw 3.09e-6
        #      （增广 MCL 的随机注入，实测 33× / 170× 的瞬时跳动）
        # 阈值取在**尖峰天花板之上**，否则尖峰会把 cov_collapsed_sec 反复清零，
        # ⑥-C 的重播种将永远等不到保持期：
        #   cov_xy_min  = 3.0e-5 = 尖峰 ×2.6，ⓐ /59，ⓑ ×85
        #   cov_yaw_min = 8.0e-6 = 尖峰 ×2.6，ⓐ /64，ⓑ ×239
        #
        # ★ 2026-09-20 更正（红线 12：被实测否掉的推断要留痕）——
        #   上面那两条 margin 是**在 09-18 那一个位置上**算的，不能跨位置转移：
        #   那一处的平台期是 3.3e-8，而 09-20 实测的另一处平台期是 ~5e-6（高 150×）。
        #   ⇒ 8.0e-6 在该处只比平台期高 1.3×，ⓑ 的 239× 余量在那里根本不存在。
        #   09-20 实测（同一次停车、重播种后未被下一次重播种打断的一段，2 Hz）：
        #     +20.0 s  cov_aa = 4.331e-4   —— 建库每格停留这一档，**不得触发**
        #     +235.5 s cov_aa = 8.321e-6   —— 旧阈值 8e-6 到这时才被越过
        #     +256.0 s cov_collapsed 置位（hold 20 s 到点）⇒ 距「5 min 必须触发」只差 44 s
        #   新值取在两次实测的平台期(6.0e-6)与 +20 s 值(4.331e-4)的几何中值：
        #     cov_yaw_min = 5.0e-5 ⇒ 该处 +73 s 越阈、+93 s 置位
        #     ⓐ 93/20 = 4.7×  ｜ ⓑ 300/93 = 3.2×  ｜ ⓒ 93 s 落进 [40,150] s
        #   ⚠️ 已知代价（未测，登记）：旧值取在「注入尖峰 ×2.6」之上，是为了防尖峰
        #      反复清零 collapsed_sec；抬到 5e-5 后这层保护只剩 hold(20 s) 一道。
        #      部署后须用记录器验证：无重播种时 collapsed_sec 不得被清零。
        #   ⚠️ cov_xy_min = 3.0e-5 **不动**：09-20 那一处 cov_xx 全程 ≥1.1e-3
        #      （从未接近阈值），xy 支本次一次都没触发过，没有证据支持改它。
        # 负值 = 关闭该轴（cov 可以为负，所以不能用 0 当「关」）。
        self.declare_parameter('cov_xy_min', 3.0e-5)
        self.declare_parameter('cov_yaw_min', 5.0e-5)
        # 塌缩须**连续**保持这么久才置位（防单帧注入尖峰清零计时）。
        # 标定：cov 从健康 2e-2 衰减到阈值 3e-5 约需 75 s ⇒ 20 s 保持只加很小的延迟；
        # 而 20 s 的正常停车停留根本到不了阈值（那一档 cov ≈ 1.8e-3，差 59×）。
        self.declare_parameter('cov_collapse_hold_sec', 20.0)
        # ⑥-B：塌缩期间跳过强制更新（见 _maybe_force_amcl_update 内的说明）。
        self.declare_parameter('amcl_force_skip_when_collapsed', False)   # ← 2026-09-20 对照实验临时改判（原值 True）
        #
        # ★★ 2026-09-20 对照实验（红线 12：被实测质疑的假设要留痕）
        #   ⑥-B 的立论是「塌缩时每次强制更新只会让样本更贫化」（见 :567-568 原文）。
        #   09-20 改 `cov_yaw_min` 后重启，**意外构成 A/B**，方向与立论相反：
        #     +~1 s  ⑥-B 未生效（强制更新在跑）  cov_yaw = 6.0101e-06
        #     +~20 s 强制更新在跑                cov_yaw = 1.0725e-05  ← **升了**
        #     +20 s 起 ⑥-B 掐掉更新              冻死在 1.0725e-05
        #                                        amcl_age +1.0 s/s 无界爬升
        #                                        fu_skip  +2.0/s
        #                                        而 status 仍是 0
        #   ⇒ **一个数据点，不算数（红线 11）**，故把本参数临时置 False 做对照。
        #   · 若 cov 持续单调下跌 且 粒子云在缩  ⇒ **证实**贫化 ⇒ 恢复 True
        #   · 若 cov 稳住/回升    或 云展布稳住 ⇒ **证伪**    ⇒ ⑥-B 需重新论证
        #   ★ 主判据 = `/particle_cloud` 展布 + N_eff（协方差是派生量，只作旁证）。
        #   ⚠️ 实测：A 臂里 `/particle_cloud` 也是 **0 帧** —— nav2 的 amcl 在
        #      `laserReceived()` 里同一处发 `/amcl_pose` 与 `/particle_cloud`，
        #      不更新就两个都不发 ⇒ **A 臂对滤波器零可观测性**，主判据只能在 B 臂量。
        #   ⚠️ 这是**实验态，不是修复**。结论出来前不得当成已修。
        #   ⚠️ 已知代价：置 False 后塌缩期间 AMCL 会持续以 1 Hz 被强制更新，
        #      与 ⑥-C（尚不存在）「重播种前别搅」的意图相反。
        self.declare_parameter('pose_jump_m', 0.8)
        self.declare_parameter('outside_map_margin_m', 0.5)
        self.declare_parameter('status2_hold_sec', 4.0)
        self.declare_parameter('self_heal_timeout_sec', 25.0)
        self.declare_parameter('self_heal_spin_wz', 0.35)
        self.declare_parameter('self_heal_spin_sec', 4.0)
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('enable_self_heal', True)
        # Stopped robots still get a laser-vs-map check. Threshold stays 0.38.
        self.declare_parameter('laser_check_period_sec', 2.0)
        self.declare_parameter('min_laser_score', 0.38)
        self.declare_parameter('min_valid_beams', 20)
        self.declare_parameter('scan_fresh_sec', 1.5)
        # If true, heal during follow without supervisor gate (NOT recommended).
        self.declare_parameter('allow_self_heal_during_follow', False)
        # While NAV/FOLLOW/recovery is armed, force AMCL laser updates even when
        # the robot is static (Nav2 otherwise only publishes after update_min_*).
        self.declare_parameter('amcl_force_update_enable', True)
        self.declare_parameter('amcl_force_update_period_sec', 1.0)

        # ---- Stage C（2026-09-18）：射线一致性判据（三值规则）----
        # 旧口径 min_laser_score 量的是「端点离最近的占用格多近」，不问「这条射线
        # 该不该打这么远」⇒ 在杂物旁反而给高分（真位姿 0.1186、错位姿 0.42），
        # 与真实好坏方向相反。**调它的阈值不可能修好它**，所以保留它只做遥测对照，
        # 判决改由射线一致性驱动。
        # 三值规则（方案 Stage A 实测改写；**不是**两臂 OR —— 那条在 err≤0.30 m 的
        # 位姿上错报 44.4%，已否）：
        #   tr >= laser_max_through_ratio                     → 定罪
        #   tr <  T_up 且 lm >= laser_min_long_match_ratio    → 释放
        #   其余（含一切证据不足）                             → 弃权
        # 弃权 ≡ 释放 ≡ status 0（_raw_code 无弃权分支），但遥测里分得开。
        self.declare_parameter('laser_ray_enable', True)
        self.declare_parameter('laser_ray_tol_m', 0.25)
        self.declare_parameter('laser_ray_step_m', 0.025)
        self.declare_parameter('laser_ray_max_range_m', 16.0)
        # ⚠️ 必须与标定口径一致：Stage B 定 T_up 的全部测量用的是 -1.0（关掠射防护），
        # 而 laser_verify 的默认值是 0.30（G5）。阈值来自哪个口径就用哪个口径跑，
        # 否则阈值与它赖以成立的数据不匹配。
        self.declare_parameter('laser_ray_grazing_m', -1.0)
        self.declare_parameter('laser_min_evidence', 40)
        self.declare_parameter('laser_min_long_beams', 40)
        # T_up / T_down 取自 Stage B part 9（红线 11：只能来自标定）。tol=0.25 下
        # 实测三层：健康 1-5 max 0.0949 < 位置 8 max 0.221 < 位置 6 min 0.3498。
        # T_up=0.25 落在这个窗口内 ⇒ 位置 8（位置正确、地图远处不符）落进弃权。
        # ⚠️ 已知代价：T_up>0.221 时合成 1.00 m 位移检出由 94.4% 降到 83.0%。
        self.declare_parameter('laser_max_through_ratio', 0.25)
        self.declare_parameter('laser_min_long_match_ratio', 0.40)
        # G11：连续 K 次定罪才置 _laser_mismatch。0.5 Hz × K=3 = 6 s，
        # 在一个「闩锁本来就是持续状态」的节点里这是零成本。
        self.declare_parameter('laser_mismatch_k', 3)
        # G10：位姿必须「确实停着」才允许定罪，否则弃权。
        # 比 amcl_static_*（0.12 m / 0.12 rad）紧一个量级 —— 那一对是给「AMCL 静默
        # 是否算陈旧」用的，不是给判据用的。Stage B 实测：0.15 m 合成位移的 through
        # 最大已到 0.3338、5° 偏航 p50 0.227，用松阈值会把正常行驶偏差判成穿墙。
        self.declare_parameter('laser_pose_max_trans_m', 0.05)
        self.declare_parameter('laser_pose_max_yaw_rad', 0.02)
        # G12：判决陈旧 ⇒ 当未知，不得继承旧的 _laser_mismatch（旧代码里一次数据
        # 饥饿之后，陈旧判决可以无限期持住 status 3）。
        self.declare_parameter('laser_verdict_max_age_sec', 6.0)
        # C2：raw != 3 连续保持这么久 ⇒ 解除 _latched_3。旧代码只有 raw==0 才解，
        # 而 raw==1 会被立刻重新闩上 ⇒ 单向陷阱。
        self.declare_parameter('unlatch_hold_sec', 6.0)

        # ---- ⑥-C 自动本地重播种 ------------------------------------------------
        # 目的：把「塌缩后冻住、动不了」变成「能动、可被纠正」。
        # ★★ 它**不能修正错误位姿** —— 种子种在 *当前* 位姿上，净位移恒为零。
        #    （见 _maybe_reseed 的完整边界说明；用户 2026-09-20 已授权本机制。）
        self.declare_parameter('reseed_enable', True)
        # ★ 收窄触发：只允许「判据定罪」时开火（用户 2026-09-20 拍板）。
        #   为什么需要它（09-20 实测）：`cov_yaw_min=5e-5` 落在**健康停车**分布**内部** ——
        #   6 次开火的触发态**全部**是「机器停着 + 位姿正确 + status 0 + 0 次定罪」。
        #   ⇒ 建库每格停 ≈20 s（正好等于 hold 门槛）期间会持续开火。
        #   ⚠️ **「等 cov_yaw_min 标定完就置 False 回宽触发」这条路已被实测否掉
        #   （红线 12 留痕）；本门是常设设计，不是临时兜底。**
        #   依据：健康停车的粒子云是两个**离散模态**的混合，cov_yaw = p(1-p)·Δ²，
        #   配比 p 会一路游走到 0 ⇒ 停够久必然衰减到机器精度（09-20 两条独立实测：
        #   塌缩 172 s 时判据仍 abstain；另一台静止 51 min 也塌到 1e-14）。
        #   ⇒ **没有任何正阈值能把「健康停车」排除在外**：调阈值只能**推迟**触发时刻，
        #   不能**区分**健康与否。宽触发的真实代价已实测：33 发/小时
        #   （09-20 11:13–12:36 CST，中位间隔 75 s），而建库每格停 ≈20 s 会持续开火。
        #   ⇒ 触发条件必须来自**独立于 cov 的证据**，即判据定罪。
        #   `reseed_require_verdict=False` 保留作紧急回滚，但**默认 True 是常设值**。
        self.declare_parameter('reseed_require_verdict', True)
        # 真静止多久才允许开火。45 s 的依据（红线 11）：
        #   · 建库时每个目标点停 ≈20 s ⇒ 2.25× 余量，正常作业**不得**触发（验收④）
        #   · 而塌缩闩锁本身要求「衰减 77 s + hold 20 s ≈ 97 s」（09-20 实测 τ_yaw≈10.7 s）
        #     ⇒ 这道门在默认参数下**不咬**，它是防「有人调小 cov_collapse_hold_sec /
        #       调大 cov_yaw_min 之后 ⑥-C 变成见停就开火」的独立护栏，不是当前触发器。
        self.declare_parameter('reseed_static_sec', 45.0)
        # 「真静止」判据。比 amcl_static_*（0.12 m / 0.12 rad）紧一个量级 ——
        # 那一对回答的是「AMCL 静默是否算陈旧」，这里要回答的是「车真的停着吗」。
        # 09-18 实测：停机时 /odom 的 x/y 跨度**恰好 0.000e+00**、vx=wz≈0
        # ⇒ 这个锚点不会被里程计噪声推着走。
        self.declare_parameter('reseed_static_trans_m', 0.03)
        self.declare_parameter('reseed_static_yaw_rad', 0.02)
        # 两次重播种之间的最短间隔。必须 **≪ 97 s**，否则「冷却期还没过、cov 已经塌回去」
        # ⇒ 只在每次闩锁后救一次，而不是持续把门撑着。30 s ≈ 2.8×τ_yaw，取 60 s 就逼近
        # 闩锁点了。它同时是「种子被 AMCL 静默拒绝」这种病态情形的**唯一**速率上界
        # （此时 _cov_collapsed 不会被清，没有冷却就是 2 Hz 风暴）。
        # ⚠️ 正常运行时的速率**不由它决定**：种下去 → cov 抬到 0.25 → 再塌回来要 ~97 s
        #    ⇒ 停放时自然周期 ≈97 s ≈ 37 次/小时。**这是设计行为，不是缺陷。**
        self.declare_parameter('reseed_cooldown_sec', 30.0)
        # 载荷逐位照抄网页「确认初位姿」按钮（web_server.py 的 publish_initial_pose）：
        # cov[0]=cov[7]=0.25 (σ=0.5 m)、cov[35]=0.068 (σ≈15°)。
        # ⚠️ 两者不得各自为政 —— cov_yaw 是**区分种子来源的判别量**（09-20 实测：
        #    web=0.068 vs boot p2 / lost r_seed=0.15），改这里等于改变现场取证的可读性。
        self.declare_parameter('reseed_cov_xy', 0.25)
        self.declare_parameter('reseed_cov_yaw', 0.068)
        # 0 = 不限次数（默认）。非 0 时才是一次性逃生阀，用于「怀疑它在空转」的排查。
        self.declare_parameter('reseed_max_count', 0)

        self._cb = ReentrantCallbackGroup()
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._amcl_mono: Optional[float] = None
        self._odom_at_amcl: Optional[tuple] = None  # (x, y, yaw) in odom at last amcl
        self._map: Optional[OccupancyGrid] = None
        self._nav_en = False
        self._follow_en = False
        self._recovery_en = False
        self._phase2c_recovery = False
        self._status = 1
        self._latched_3 = False
        self._raw_bad_since: Optional[float] = None
        self._heal_started: Optional[float] = None
        self._heal_phase = ''
        self._last_xy: Optional[tuple] = None
        self._scan: Optional[LaserScan] = None
        self._scan_mono: Optional[float] = None
        self._field: Optional[DistanceField] = None
        self._field_map_id: Optional[tuple] = None
        self._laser_mismatch = False
        self._laser_score: Optional[float] = None
        self._last_laser_check = 0.0
        # 判决依据（R3 诊断用）。此前 _laser_score 是只写字段、全文件无读点，
        # 「3 是活判决还是冻住的值」完全不可见 —— 2026-09-14 为此翻了 40 分钟
        # python3_<pid>_<launch_ms>.log。
        self._laser_eval_mono: Optional[float] = None
        self._laser_beams = 0
        self._laser_ratio = 0.0
        self._laser_reason = ''
        self._last_raw = 1
        self._last_jump = False
        self._g_tf_ok = False
        self._g_amcl_fresh = False
        self._g_outside = False
        self._last_force_update = 0.0
        self._force_update_inflight = False

        # ---- Stage C 新判据状态 ----
        self._rc = None                      # 最近一次 RayConsistency（遥测用）
        self._ray_verdict = 'none'           # convict | release | abstain | none
        self._ray_abstain = ''               # 弃权原因（verdict==abstain 时有值）
        self._ray_consec_bad = 0             # G11 连续定罪计数
        self._ray_pose_guard = 'init'        # G10 位姿信任闸门的结论
        self._laser_convicted = False        # 新口径定罪（= consec_bad >= K）
        self._laser_old_mismatch = False     # 旧口径的结论，只做对照
        self._g_laser_fresh = False          # G12 新鲜度闸门
        self._raw_not3_since: Optional[float] = None   # C2 解闩锁计时起点

        # ---- ⑥（2026-09-18）塌缩防护状态 ----
        self._cov_collapsed = False          # ⑥-A：连续塌缩 ≥ hold 后的**置位**结果
        self._cov_collapsed_since = None     # 连续塌缩的计时起点（单调钟）
        self._cov_collapsed_sec = 0.0        # 已连续塌缩的秒数（遥测用，未到 hold 也报）
        self._force_skipped = 0              # ⑥-B：因塌缩而拦下的强制更新次数
        self._reseed_count = 0               # ⑥-C：重播种次数
        self._reseed_last_at = None          # ⑥-C：上次重播种的时钟时刻
        self._reseed_last_reason = ''        # ⑥-C：上次重播种的原因（实际发出去的那次）
        self._reseed_blocked = 'init'        # ⑥-C：本 tick 为什么没开火（遥测，验收④靠它）
        self._reseed_static_sec = 0.0        # ⑥-C：当前已连续静止秒数（遥测）
        self._odom_anchor = None             # ⑥-C：(x, y, yaw, t) 里程计静止锚点
        self._phase2c_loc = ''               # 兼容镜像 /xw/localization/phase2c_loc_state
        self._canonical_state = ''           # 权威 /xw/localization/phase2c_state 的 state

        latch_in = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        # Match Nav2 AMCL (transient_local) so a restart while idle still gets last pose.
        amcl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, amcl_qos
        )
        self.create_subscription(OccupancyGrid, 'map', self._on_map, _MAP_QOS)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Bool, '/xw/nav/enable', self._on_nav_en, latch_in)
        self.create_subscription(Bool, '/xw/follow/enable', self._on_follow_en, latch_in)
        self.create_subscription(
            Bool, '/xw/localization/recovery_enable', self._on_recovery_en, latch_in
        )
        # Phase2C-C3: Visual+Laser Reloc owns recovery — never spin+reinit in parallel.
        self.create_subscription(
            Bool, '/xw/localization/phase2c_recovery', self._on_phase2c_recovery, latch_in
        )
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self._on_initialpose, 10
        )
        # ⑥-C 的**安全闸门**：phase2c 处于 NEED_OPERATOR 时，`boot_localizer` 会把
        # 任何裸 `/initialpose` 当成「操作员提交的位姿」去跑 _run_operator_verify。
        # 两个话题都订阅，因为 boot 的 `_awaiting_operator()` 是两个都看：
        #   `if canonical_state == NEED_OPERATOR: return True`
        #   `if phase2c_loc      == NEED_OPERATOR: return True`
        # ⚠️ 触发条件是**两个中任一**为 NEED_OPERATOR 就闭嘴 —— 宁可少救一次。
        # 两者都是 String + TRANSIENT_LOCAL ⇒ 订阅即拿到当前值，不会因为「错过了
        # 那条 latched 消息」而在事故窗口里失明。
        self.create_subscription(
            String, '/xw/localization/phase2c_loc_state', self._on_phase2c_loc, latch_in
        )
        self.create_subscription(
            String, '/xw/localization/phase2c_state', self._on_canonical_state, latch_in
        )

        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_pub = self.create_publisher(Int8, '/xw/localization_status', latch)
        # 闩住的判决诊断。纯只读观测，不参与任何判定 —— 判 3 时能立刻看出
        # 「分数多少 / 多少束 / 判决多少秒前算的」，而不是只能看到裸的状态码。
        self._detail_pub = self.create_publisher(
            String, '/xw/localization/health_detail', latch
        )
        self._event_pub = self.create_publisher(RobotEvent, '/xw/event', 10)
        self._cmd_pub = self.create_publisher(Twist, '/xw/cmd/motion', 10)
        # ⑥-C：本节点此前**只有订阅者、没有发布者**。发在 'initialpose'（相对名，
        # 与上面那条订阅逐字相同）⇒ 解析结果必定一致，且会回灌进自己的
        # `_on_initialpose`（那是**要的**：播种后把塌缩计时和 status-3 闩锁一起归零）。
        # QoS 与其余 6 个 /initialpose 发布者一致（默认 RELIABLE / VOLATILE / depth 10），
        # 也是 nav2 amcl 的 `initialpose` 订阅（SystemDefaultsQoS）能收的形状。
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )

        self._reinit = self.create_client(
            Empty, 'reinitialize_global_localization', callback_group=self._cb
        )
        self._nomotion = self.create_client(
            Empty, 'request_nomotion_update', callback_group=self._cb
        )

        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / max(hz, 0.5), self._tick, callback_group=self._cb)
        self.get_logger().info(
            'localization_health ready (detect always; heal gated by recovery_enable; '
            'phase2c_recovery blocks spin+reinit; idle laser mismatch → status 3; '
            'nav-mode forces AMCL nomotion updates)'
        )

    @property
    def _nav_mode(self) -> bool:
        return self._nav_en or self._follow_en or self._recovery_en

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg
        self._amcl_mono = self._now()
        # Snapshot odom so silence while static is not treated as stale.
        self._odom_at_amcl = self._lookup_odom_pose()

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_mono = self._now()

    def _on_nav_en(self, msg: Bool) -> None:
        self._nav_en = bool(msg.data)

    def _on_follow_en(self, msg: Bool) -> None:
        was = self._follow_en
        self._follow_en = bool(msg.data)
        if self._follow_en and not was:
            # Cancel any in-progress heal motion; detection continues.
            self._abort_heal_motion('follow on → pause heal execution')
        elif was and not self._follow_en:
            self._raw_bad_since = None
            self.get_logger().info('follow off → heal may arm if recovery_enable/nav')

    def _on_recovery_en(self, msg: Bool) -> None:
        was = self._recovery_en
        self._recovery_en = bool(msg.data)
        if self._recovery_en and not was:
            self.get_logger().info('localization recovery armed (heal execution allowed)')
        elif was and not self._recovery_en:
            self._abort_heal_motion('recovery disarmed')

    def _on_phase2c_recovery(self, msg: Bool) -> None:
        """Phase2C ACTIVE → Reloc is sole owner; abort/forbid spin+reinit."""
        want = bool(msg.data)
        if want and not self._phase2c_recovery:
            self._abort_heal_motion('phase2c_recovery on → heal forbidden (Reloc owner)')
            self.get_logger().warn(
                'phase2c_recovery ACTIVE — spin+reinitialize_global_localization blocked'
            )
        self._phase2c_recovery = want

    def _on_initialpose(self, _msg: PoseWithCovarianceStamped) -> None:
        self._latched_3 = False
        self._heal_started = None
        self._heal_phase = ''
        self._raw_bad_since = None
        self._laser_mismatch = False
        self._laser_score = None
        # 旧位姿的判决作废：诊断里必须看出「还没按新位姿重新打分」，
        # 而不是继续显示上一处点位的分数。
        self._laser_eval_mono = None
        self._laser_reason = ''
        # ⑥（顺带修一个既有的真 bug）：不清 `_last_xy` 的话，新位姿一到，
        # `_pose_jump()` 会拿它和**上一个位置**的坐标比 —— 一次合法的重新定位
        # （尤其是 ⑥-C 自己发的、或操作员从几十米外拖过来的）会被报成 pose_jump
        # ⇒ status 2 ⇒ 可能引出那副已知会加重病情的自旋自愈。这里置 None，
        # 让 `_pose_jump()` 把它当作新的基准（它自己开头就有 `if self._last_xy is None
        # ⇒ 重设基准并 return False` 这个分支 —— 按方法名找，不按行号，本文件行号已漂过）。
        self._last_xy = None
        # 新的种子意味着滤波器被重新供能，旧的塌缩计时不再描述它。
        # （下个 tick 的 cov 也会很大而自然清零，这里显式复位以免中间有几帧假置位。）
        self._cov_collapsed = False
        self._cov_collapsed_since = None
        self._cov_collapsed_sec = 0.0
        self.get_logger().info('initialpose → clear status-3 latch')

    def _on_phase2c_loc(self, msg: String) -> None:
        """⑥-C 安全闸门用的兼容镜像（与 boot_localizer 的 `_phase2c_loc` 同源）。

        刻意**不做** boot 那条「canonical 是 incident 时忽略 READY」的抑制 —— 这里要的是
        两个来源的**并集**：任一为 NEED_OPERATOR 就闭嘴。宽松方向必须朝「少开火」。
        """
        self._phase2c_loc = (msg.data or '').strip()

    def _on_canonical_state(self, msg: String) -> None:
        """⑥-C 安全闸门用的权威态（/xw/localization/phase2c_state 的 state 字段）。

        载荷是 `{"state":..,"goals_blocked":..,"source":..,"generation":..}`。
        解析失败一律**当成空的**（= 不阻止开火）—— 因为这条闸门防的是一个窄窗口，
        而不是要求 phase2c 在线；相位不确定时不因此永久瘫掉 ⑥-C。
        真正的防重复靠 cooldown，而 cooldown 与这里无关。
        """
        try:
            ev = json.loads(msg.data or '{}')
        except (ValueError, TypeError):
            return
        if isinstance(ev, dict) and ev.get('state'):
            self._canonical_state = str(ev['state'])

    def _abort_heal_motion(self, reason: str) -> None:
        if self._heal_started is not None or self._heal_phase:
            self._heal_started = None
            self._heal_phase = ''
            self._stop_motion()
            self.get_logger().info(reason)

    def _tf_ok(self) -> bool:
        map_f = str(self.get_parameter('map_frame').value)
        odom_f = str(self.get_parameter('odom_frame').value)
        stale = float(self.get_parameter('tf_stale_sec').value)
        try:
            tf = self._tf.lookup_transform(
                map_f, odom_f, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            # stamp 0 means static/latest-only; treat as ok if lookup succeeded
            if tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0:
                return True
            return age < stale
        except TransformException:
            return False

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _lookup_odom_pose(self) -> Optional[tuple]:
        """Current base pose in odom: (x, y, yaw) or None if TF missing."""
        odom_f = str(self.get_parameter('odom_frame').value)
        base_f = str(self.get_parameter('base_frame').value)
        try:
            tf = self._tf.lookup_transform(
                odom_f, base_f, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        return (float(t.x), float(t.y), self._yaw_from_quat(tf.transform.rotation))

    def _odom_nearly_static_since_amcl(self) -> bool:
        """True if odom has not moved past AMCL update-scale thresholds since last pose."""
        if self._odom_at_amcl is None:
            self._odom_at_amcl = self._lookup_odom_pose()
        ref = self._odom_at_amcl
        cur = self._lookup_odom_pose()
        if ref is None or cur is None:
            return False
        dx = cur[0] - ref[0]
        dy = cur[1] - ref[1]
        dyaw = abs(math.atan2(math.sin(cur[2] - ref[2]), math.cos(cur[2] - ref[2])))
        lim_t = float(self.get_parameter('amcl_static_trans_m').value)
        lim_y = float(self.get_parameter('amcl_static_yaw_rad').value)
        return math.hypot(dx, dy) <= lim_t and dyaw <= lim_y

    def _odom_static_sec(self) -> Optional[float]:
        """⑥-C：距「上一次明显动了」过了多少秒。None = odom 不可用（**不累计**）。

        为什么不用现成的 `_odom_nearly_static_since_amcl()`：它的锚点 `_odom_at_amcl`
        在**每一条** amcl_pose 上刷新（`_on_amcl` 里重置），所以它量的是「最近一次 AMCL
        更新以来的运动」，窗口只有 ~1 s；而 ⑥-C 要问的是「停了多久」。阈值也不同 ——
        那一对的 0.12 m / 0.12 rad 是给「AMCL 静默算不算陈旧」用的。

        锚点只在**超出阈值**时前移，不逐帧跟随。依据（09-18 实测）：停机时 `/odom` 的
        x/y 跨度**恰好 0.000e+00**、vx=wz 是最小次正规数（实质 0）⇒ 不存在「噪声把锚点
        推着走、静止时间永远攒不起来」。车若真在缓慢蠕动，锚点前移正是我们想要的。

        TF 缺失时返回 None 并清锚点 —— 宁可不让它累计，也不要拿一段没有里程计的时间
        当「静止」。
        """
        cur = self._lookup_odom_pose()
        if cur is None:
            self._odom_anchor = None
            return None
        now = self._now()
        if self._odom_anchor is None:
            self._odom_anchor = (cur[0], cur[1], cur[2], now)
            return 0.0
        ax, ay, ayaw, at = self._odom_anchor
        dx = cur[0] - ax
        dy = cur[1] - ay
        dyaw = abs(math.atan2(math.sin(cur[2] - ayaw), math.cos(cur[2] - ayaw)))
        lim_t = float(self.get_parameter('reseed_static_trans_m').value)
        lim_y = float(self.get_parameter('reseed_static_yaw_rad').value)
        if math.hypot(dx, dy) > lim_t or dyaw > lim_y:
            self._odom_anchor = (cur[0], cur[1], cur[2], now)
            return 0.0
        return max(0.0, now - at)

    def _amcl_fresh(self) -> bool:
        """True if amcl_pose is recent, or robot is still nearly static since last pose.

        AMCL does not republish while stopped (update_min_d/a). Treating that silence
        as stale forced status=1 on idle; exempt when odom has barely moved and TF ok
        is already required by the caller.
        """
        if self._amcl is None or self._amcl_mono is None:
            return False
        stale = float(self.get_parameter('amcl_stale_sec').value)
        if (self._now() - self._amcl_mono) <= stale:
            return True
        return self._odom_nearly_static_since_amcl()

    def _cov_xy_yaw(self) -> tuple:
        if self._amcl is None:
            return 999.0, 999.0
        c = self._amcl.pose.covariance
        xy = max(float(c[0]), float(c[7]))
        yaw = float(c[35])
        return xy, yaw

    def _update_cov_collapse(self) -> None:
        """⑥-A：判定「协方差是否已塌缩」，每 tick 调一次 —— **单一写入者**。

        为什么不做进 `_raw_code()`：那里有 3 个 early return（TF 不 ok / amcl 缺失 /
        不新鲜），塌缩判定会跟着一起被跳过 —— 而塌缩恰恰常与「AMCL 静默」同时发生，
        那正是最需要它的时刻。
        为什么不做成惰性：`_publish_detail` 在 `_tick` 的 finally 里**无条件**跑，
        惰性的话它会读到没被刷新过的旧值，遥测就会说谎。

        比较按**有符号**写（cov 实测为负，见 declare_parameter 处）。负的限值 = 关闭该轴。

        注意：本函数**只写遥测状态，绝不改 `status`**。把「退化」路由进 status 2 等于
        让 `_self_heal_tick` 去自旋 + reinit —— 09-16 实测那条自愈把估计打得更坏
        （静止中 yaw 自己跳 +98.3°，through 从 0.3568 升到 0.5864，迄今最坏）。
        退化的出口是 ⑥-C 重播种，不是自旋。
        """
        if self._amcl is None:
            # 没有位姿就谈不上「确信地错」。保持上次判定，不在这里翻转 ——
            # 这里翻转会把「还没收到第一条 amcl」误报成「已恢复」。
            return
        c = self._amcl.pose.covariance
        xy = max(float(c[0]), float(c[7]))
        yaw = float(c[35])
        lim_xy = float(self.get_parameter('cov_xy_min').value)
        lim_yaw = float(self.get_parameter('cov_yaw_min').value)
        hit = ((lim_xy > 0.0 and xy < lim_xy)
               or (lim_yaw > 0.0 and yaw < lim_yaw))
        now = self._now()
        if not hit:
            self._cov_collapsed = False
            self._cov_collapsed_since = None
            self._cov_collapsed_sec = 0.0
            return
        if self._cov_collapsed_since is None:
            self._cov_collapsed_since = now
        self._cov_collapsed_sec = now - self._cov_collapsed_since
        hold = float(self.get_parameter('cov_collapse_hold_sec').value)
        was = self._cov_collapsed
        self._cov_collapsed = self._cov_collapsed_sec >= hold
        if self._cov_collapsed and not was:
            # ★ 逐轴打**真实的**比较符。上面的 `hit` 是 **OR** —— 只要一轴越阈就判塌缩，
            #   所以另一轴完全可以高出阈值几十倍。旧文本对两轴无条件写 `<`，
            #   实测打出过「cov_xy=2.021e-03 (< 3.0e-05)」（高出 **67 倍**）这种假陈述。
            #   三态分开：越阈 / 未越阈 / 阈值≤0（该项被关闭，不比较）。
            def _axis_label(v, lim):
                if lim <= 0.0:
                    return 'disabled'
                if v < lim:
                    return '< %.1e HIT' % lim
                return '>= %.1e ok' % lim
            self.get_logger().warn(
                'cov COLLAPSED (sustained %.1fs >= %.1fs): '
                'cov_xy=%.3e %s | cov_yaw=%.3e %s — 滤波器已不再探索。'
                '这**只是遥测**，status 不受影响；退化要走的出口是重播种，'
                '不是自旋自愈。'
                % (self._cov_collapsed_sec, hold, xy, _axis_label(xy, lim_xy),
                   yaw, _axis_label(yaw, lim_yaw))
            )

    def _outside_map(self) -> bool:
        if self._amcl is None or self._map is None:
            return False
        info = self._map.info
        x = self._amcl.pose.pose.position.x
        y = self._amcl.pose.pose.position.y
        margin = float(self.get_parameter('outside_map_margin_m').value)
        min_x = info.origin.position.x - margin
        min_y = info.origin.position.y - margin
        max_x = info.origin.position.x + info.width * info.resolution + margin
        max_y = info.origin.position.y + info.height * info.resolution + margin
        return not (min_x <= x <= max_x and min_y <= y <= max_y)

    def _pose_jump(self) -> bool:
        if self._amcl is None:
            return False
        x = self._amcl.pose.pose.position.x
        y = self._amcl.pose.pose.position.y
        if self._last_xy is None:
            self._last_xy = (x, y)
            return False
        dx = x - self._last_xy[0]
        dy = y - self._last_xy[1]
        self._last_xy = (x, y)
        lim = float(self.get_parameter('pose_jump_m').value)
        return math.hypot(dx, dy) > lim

    def _maybe_force_amcl_update(self) -> None:
        """Keep amcl_pose live while NAV/FOLLOW/recovery is armed.

        Nav2 AMCL only republishes after update_min_* motion. Sitting still at a
        waypoint otherwise leaves status=1 (stale pose) for minutes even when the
        laser/map overlay looks fine. Periodic request_nomotion_update forces a
        laser update without global reinit.
        """
        if not bool(self.get_parameter('amcl_force_update_enable').value):
            return
        if not self._nav_mode:
            return
        if self._amcl is None:
            return
        period = float(self.get_parameter('amcl_force_update_period_sec').value)
        period = max(0.2, period)
        now = self._now()
        if now - self._last_force_update < period:
            return
        # Skip if a fresh amcl_pose already arrived within the period.
        if self._amcl_mono is not None and (now - self._amcl_mono) < period:
            return
        if self._force_update_inflight:
            return
        # ⑥-B：位姿已经塌缩时**不要**继续强制更新。
        # ⚠️ 2026-09-20：下面这条立论正被对照实验检验，本参数的声明默认值已
        #    临时改为 False（留痕见 declare_parameter 处）⇒ **本节默认不执行**。
        #    结论出来前不要依据本段的理由做任何决定。
        # 每一次强制更新都只是在零运动信息下再做一次「传感器更新 + 重采样」，
        # 只会让样本更贫化 —— 此时该做的是重播种（⑥-C），不是继续搅。
        # 放在这里（而不是函数开头）是为了让计数器数的是「本次确实要发、但被拦下」，
        # 而不是每个 tick 都 +1。
        # ⚠️ 刻意**不更新** `_last_force_update`：跳过不消耗周期记账，下一 tick 仍会
        #    尝试 —— 塌缩一旦解除就立刻恢复原节奏，不需要等一个完整周期。
        if (bool(self.get_parameter('amcl_force_skip_when_collapsed').value)
                and self._cov_collapsed):
            self._force_skipped += 1
            return
        if not self._nomotion.service_is_ready():
            return
        self._last_force_update = now
        self._force_update_inflight = True
        fut = self._nomotion.call_async(Empty.Request())

        def _done(f) -> None:  # noqa: ANN001
            self._force_update_inflight = False
            try:
                f.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'request_nomotion_update failed: {exc}')

        fut.add_done_callback(_done)

    def _maybe_reseed(self) -> None:
        """⑥-C：塌缩已闩 + 车真停着 ⇒ 用**当前滤波位姿**重播种，把单向门撬开。

        ★★ 诚实的边界（必须跟这段代码一起读，不得含糊）：
          · 它**无法修正错误位姿** —— 种子种在 *当前* 位姿上，**净位移恒等于零**。
            位置 6 那次「按了网页按钮也没用」正是这个道理：种子 = AMCL 自己那个错位姿，
            重新撒开的云收敛回同一个局部极值。
          · 它只做一件事：把「冻住、动不了」变成「能动、可被纠正」。
          · **在「位置 8 型」区域（long_match ≈0.39 ≪ 健康带 0.58~0.68）连「能动」都
            买不到** —— 那里 0.2 m 尺度上似然是平的，判据自己都没有偏好。
          ⇒ 所以 ⑥-C **不是 ③（网页按钮）的替代品，是给 ③ 争取时间。**

        ★ 它真正的价值要在 ⑥-B（amcl_force_skip_when_collapsed）打开之后才兑现：
          那时 cov_collapsed ⇒ 跳过强制更新 ⇒ AMCL 连 /particle_cloud 都不再发
          （**零可观测性**），⑥-C 是唯一的出口。当前 ⑥-B 维持 False，所以本机制此刻
          是「让滤波器活着」，不是「救活一个已经死掉的滤波器」。

        ★ 载荷不是想当然的（09-20 实测）：这个协方差 (0.25 / 0.068) 的种子**确实推得动
          位姿** —— 三次操作员注入分别位移 6 mm / 154 mm / 200 mm；而 AMCL 自己撒遍
          全图那一次的协方差是 1.31e+01（σ≈3.6 m），对发布位姿的作用是 **6.23e-14 m**。
          ⇒ 机制不是死的，`打不中` 是那个**过大**的载荷，不是这个。

        ★ 防重复**不能**靠 `_cov_collapsed`：`_on_initialpose` 在收到我们自己的种子时
          会立刻把它清零（那是要的行为 —— 计时归零），所以必须另有一个冷却计时器。
        """
        self._reseed_static_sec = self._odom_static_sec() or 0.0
        now = self._now()
        cooldown = float(self.get_parameter('reseed_cooldown_sec').value)
        want_static = float(self.get_parameter('reseed_static_sec').value)
        max_count = int(self.get_parameter('reseed_max_count').value)

        blocked = ''
        if not bool(self.get_parameter('reseed_enable').value):
            blocked = 'disabled'
        elif self._amcl is None:
            blocked = 'no_amcl'
        elif not self._cov_collapsed:
            blocked = 'not_collapsed_%.0fs' % self._cov_collapsed_sec
        elif (bool(self.get_parameter('reseed_require_verdict').value)
              and not (self._laser_verdict_fresh()
                       and self._ray_verdict == 'convict')):
            # ★ ⑥-C 收窄（用户 2026-09-20 拍板）：**判据定罪才开火**。
            #   依据（09-20 实测）：`cov_yaw_min=5e-5` 落在「健康停车」分布**内部** ——
            #   全部 6 次开火的触发态都是「机器停着 + 位姿正确 + status 0 + 0 次定罪」，
            #   停车十几分钟后必然越阈 ⇒ 建库（每格停 ≈20 s ≈ hold 门槛）期间持续开火。
            #   ⚠️ **旧注释在这里把方向写反了，09-20 实测更正（红线 12 留痕）**：
            #   旧文说「这是阈值标定问题（红线 11），标定完成后可安全置 False 回到
            #   宽触发」—— **那条路是死的**。cov_yaw 是双模态粒子云的混合方差
            #   p(1-p)·Δ²，配比会一路游走到 0 ⇒ 健康停车**必然**衰减到机器精度
            #   （09-20 两条独立实测：172 s / 51 min）。任何正阈值最终都会被越过 ⇒
            #   标定只能**推迟**触发、不能**区分**健康与否。本门是**常设设计**，
            #   `False` 只作紧急回滚用。完整依据见 `reseed_require_verdict` 声明处。
            #   · 判决必须**新鲜**（G12）：过期判决不得继承，与三值规则「没证据 ⇒ 弃权」
            #     同源。
            #   · 用**单次** `_ray_verdict == 'convict'`（⟺ `through_ratio >= T_up`），
            #     不用 `_laser_convicted`（K=3）。后者的语义是「连续 K 次」去抖，且与
            #     `_laser_mismatch` 同源；若实测发现单次判决闪断导致误开火，换成
            #     `self._laser_convicted` 即可（一行）。
            blocked = 'no_convict:%s(tr=%s)' % (
                self._ray_verdict or 'none',
                None if self._rc is None else round(float(self._rc.through_ratio), 3))
        elif self._heal_started is not None or self._heal_phase:
            # 与自旋自愈互斥。**只看自旋状态本身，不看 `_latched_3`。**
            # ⚠️ 09-20 实测更正（旧注释的方向是反的）：`_self_heal_tick` 的**唯一**调用点
            #   是 `_tick_body` 的 raw==2 分支，它**不读** `_latched_3`；而 `_latched_3`
            #   由 raw==3、自旋超时（:1086）或空闲漂移（:1362）置位 —— 两者不是同一件事。
            #   同一行里的 `_heal_started`/`_heal_phase` 已经**精确**表达了「自旋正在跑」。
            # ⚠️ 旧门把 `_latched_3` 也列进来，造成**死锁**（09-20 定位）：位姿错 ⇒ 判据
            #   连续 K=3 定罪（`laser_check_period_sec=2.0` ⇒ 约 6 s）⇒ `_laser_mismatch`
            #   ⇒ `_raw_code()` 返回 3 ⇒ `_latched_3` 置位；而 cov 要 ~77 s 才越阈、
            #   再过 hold 20 s 才置 `_cov_collapsed` ⇒ **门在 t≈6 s 就永久关闭，
            #   ⑥-C 恰恰在它最该开火的场景（位姿错 + 冻住）永不开火。**
            #   拿掉它之后，`_latched_3` 期间反而**正是**⑥-C 该开火的时刻。
            #   安全性：⑥-C 的种子种在**当前位姿**上，净位移恒为零，不会让位姿更坏；
            #   且 status 3 期间机器人本就停着（`_stop_motion`）。
            blocked = 'self_heal_active'
        elif not self._nav_mode:
            # 只有 NAV/FOLLOW/recovery 期间才播种。塌缩的**成因**（1 Hz 强制更新）
            # 本就在这个模式下；离开这个模式去改 /initialpose 会越过用户的预期。
            blocked = 'not_nav_mode'
        elif self._phase2c_recovery:
            # 视觉+激光 Reloc 正在拥有恢复权 —— 不要跟它抢。
            blocked = 'phase2c_recovery'
        elif (self._phase2c_loc == 'NEED_OPERATOR'
              or self._canonical_state == 'NEED_OPERATOR'):
            # ★ 安全闸门。`boot_localizer._awaiting_operator()` 在 canonical 或
            #   phase2c_loc 为 NEED_OPERATOR 时返回 True，此时任何裸 /initialpose 都会
            #   被它当成「操作员提交的位姿」去起线程跑 `_operator_pose_entry`。
            #   实际后果有限（它等 1.0 s 看 owner 是不是 operator，我们不发 owner ⇒
            #   它只打一条 'operator /initialpose ignored' 的 WARN 就放弃），
            #   但有一个窄窗口是真的危险：**操作员刚按过网页按钮**的 5 s 内
            #   (`operator_owner_fresh_sec=5.0`)，owner 仍是 operator ⇒ 它会真的
            #   跑 `_run_operator_verify(当前位姿)`。宁可少救一次，也不要伪造一次
            #   操作员提交。2026-09-20 用户已批准的正是「不声明 owner + 显式排除
            #   NEED_OPERATOR 窗口」。
            blocked = 'need_operator'
        elif self._reseed_static_sec < want_static:
            blocked = 'moving_%.1fs' % self._reseed_static_sec
        elif self._reseed_last_at is not None and (now - self._reseed_last_at) < cooldown:
            blocked = 'cooldown_%.1fs' % (now - self._reseed_last_at)
        elif max_count > 0 and self._reseed_count >= max_count:
            blocked = 'max_count_%d' % max_count
        self._reseed_blocked = blocked
        if blocked:
            return

        cov_xy = float(self.get_parameter('reseed_cov_xy').value)
        cov_yaw = float(self.get_parameter('reseed_cov_yaw').value)
        out = PoseWithCovarianceStamped()
        out.header.frame_id = str(self._amcl.header.frame_id or 'map')
        # stamp 用**零**，理由同 web_server.py：EKF 以 20 Hz 发 odom→base_link，
        # now() 永远晚于最新样本，tf2 的外推检查会**静默拒绝**整个种子。
        out.header.stamp = rclpy.time.Time().to_msg()
        out.pose.pose = deepcopy(self._amcl.pose.pose)   # 位置与姿态逐位复制
        cov = [0.0] * 36
        cov[0] = cov_xy
        cov[7] = cov_xy
        cov[35] = cov_yaw
        out.pose.covariance = cov

        self._reseed_count += 1
        self._reseed_last_at = now
        # 带上**开火时的判据读数**供审计：收窄后「这一次为什么能开火」的答案就是
        # 那次定罪，不留痕的话事后只能去翻 rosout。
        self._reseed_last_reason = 'collapsed+parked+convict(tr=%s)' % (
            None if self._rc is None else round(float(self._rc.through_ratio), 3))
        self._initialpose_pub.publish(out)

        xy, yaw = self._cov_xy_yaw()
        self.get_logger().warn(
            'loc_reseed #%d → seed at CURRENT pose (collapsed %.0fs, parked %.0fs; '
            'cov xy %.3e→%.3f yaw %.3e→%.3f; verdict=%s tr=%s). '
            'It CANNOT correct a wrong pose — it only makes the filter live again.'
            % (self._reseed_count, self._cov_collapsed_sec, self._reseed_static_sec,
               xy, cov_xy, yaw, cov_yaw, self._ray_verdict,
               None if self._rc is None else round(float(self._rc.through_ratio), 3))
        )
        self._emit(1, 'loc_reseed', json.dumps({
            'count': int(self._reseed_count),
            'collapsed_sec': round(float(self._cov_collapsed_sec), 1),
            'static_sec': round(float(self._reseed_static_sec), 1),
            'cov_xy_before': xy,
            'cov_yaw_before': yaw,
            'cov_xy_seed': cov_xy,
            'cov_yaw_seed': cov_yaw,
            'phase2c': self._phase2c_loc,
        }))

    def _laser_pose_trust(self) -> tuple:
        """G10：位姿必须「确实停着」才允许定罪。返回 (ok, reason)。

        旧代码从不检查运动。AMCL 受 update_min_d/a 门控，行驶中 amcl_pose 可比扫描
        旧一整个更新周期（至多 0.25 m / 0.2 rad）。更糟的是自愈自旋：_self_heal_tick
        以 0.35 rad/s 转，而激光校验在自旋期间照常跑 ⇒ **自旋自己制造出被判成 LOST
        的位姿偏差**。旧判据对此近乎免疫（3° 只动 0.009），新判据会把它放大 0.2~0.35。

        阈值不能借 amcl_static_trans_m/amcl_static_yaw_rad（0.12/0.12）—— 那一对是
        给「AMCL 静默是否算陈旧」用的。Stage B 实测：0.15 m 合成位移的 through 最大
        已到 0.3338（> T_up），5° 偏航 p50 0.227 ⇒ 用松阈值会把正常行驶偏差判成穿墙。
        """
        if self._heal_started is not None or self._heal_phase:
            return False, 'heal_spin'
        ref = self._odom_at_amcl
        if ref is None:
            ref = self._lookup_odom_pose()
        cur = self._lookup_odom_pose()
        if ref is None or cur is None:
            return False, 'no_odom'
        dx = cur[0] - ref[0]
        dy = cur[1] - ref[1]
        dyaw = abs(math.atan2(math.sin(cur[2] - ref[2]), math.cos(cur[2] - ref[2])))
        if math.hypot(dx, dy) > float(self.get_parameter('laser_pose_max_trans_m').value):
            return False, 'moving_trans'
        if dyaw > float(self.get_parameter('laser_pose_max_yaw_rad').value):
            return False, 'moving_yaw'
        return True, 'ok'

    def _laser_verdict_fresh(self) -> bool:
        """G12：判决必须够新才能参与判决。None 或过旧 ⇒ 当未知，不继承。"""
        if self._laser_eval_mono is None:
            return False
        age = self._now() - self._laser_eval_mono
        return age <= float(self.get_parameter('laser_verdict_max_age_sec').value)

    def _maybe_laser_check(self) -> None:
        """Score the live scan at the current pose. No motion required.

        旧口径（端点邻近度）继续算，但只进遥测与 rosout；判决改由射线一致性三值规则
        驱动（Stage C）。两者同时报出来 —— 一旦出问题能立刻对照，而不是只能看到裸状态码。
        """
        period = float(self.get_parameter('laser_check_period_sec').value)
        now = self._now()
        if now - self._last_laser_check < period:
            return
        # G12：窗口只能在真正做了评估之后才被消费。旧代码在早退**之前**就写了
        # 时间戳，一次数据饥饿吃掉整个窗口，而 _laser_mismatch 保留旧值继续参与判决。
        if self._amcl is None or self._map is None or self._scan is None or self._scan_mono is None:
            return
        if now - self._scan_mono > float(self.get_parameter('scan_fresh_sec').value):
            return
        self._last_laser_check = now
        ray_on = bool(self.get_parameter('laser_ray_enable').value)
        t_up = float(self.get_parameter('laser_max_through_ratio').value)
        t_down = float(self.get_parameter('laser_min_long_match_ratio').value)
        try:
            info = self._map.info
            map_id = (int(info.width), int(info.height), float(info.resolution), float(info.origin.position.x), float(info.origin.position.y))
            if self._field is None or self._field_map_id != map_id:
                self._field = DistanceField(self._map)
                self._field_map_id = map_id
            p = self._amcl.pose.pose
            yaw = self._yaw_from_quat(p.orientation)
            px, py = float(p.position.x), float(p.position.y)
            scored = score_scan_at_pose(
                self._field,
                self._scan,
                px,
                py,
                yaw,
                min_valid_beams=int(self.get_parameter('min_valid_beams').value),
                min_laser_score=float(self.get_parameter('min_laser_score').value),
            )
            rc = None
            if ray_on:
                rc = ray_consistency_at_pose(
                    self._field,
                    self._scan,
                    px,
                    py,
                    yaw,
                    tol_m=float(self.get_parameter('laser_ray_tol_m').value),
                    step_m=float(self.get_parameter('laser_ray_step_m').value),
                    max_range_m=float(self.get_parameter('laser_ray_max_range_m').value),
                    grazing_m=float(self.get_parameter('laser_ray_grazing_m').value),
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'laser mismatch check failed: {exc}')
            return

        # ---- 旧口径：只做遥测对照，不再驱动判决 ----
        self._laser_score = float(scored.laser_score)
        self._laser_eval_mono = self._now()
        self._laser_beams = int(scored.valid_beams)
        self._laser_ratio = float(scored.matched_ratio)
        self._laser_reason = str(scored.reason)
        self._laser_old_mismatch = (
            (not bool(scored.accepted)) and str(scored.reason) != 'few_beams'
        )

        # ---- 新口径：三值规则 ----
        self._rc = rc
        verdict = 'abstain'
        abstain = 'ray_disabled'
        if rc is not None:
            pose_ok, pose_reason = self._laser_pose_trust()
            self._ray_pose_guard = pose_reason
            min_ev = int(self.get_parameter('laser_min_evidence').value)
            min_lb = int(self.get_parameter('laser_min_long_beams').value)
            if rc.origin_blocked:
                # G4：原点落在占用格/贴墙 ⇒ 第 1 步就命中 ⇒ through 会假性逼近 100%
                abstain = f'origin_blocked:{rc.reason}'
            elif rc.n_evidence < min_ev:
                abstain = f'few_evidence:{rc.n_evidence}<{min_ev}'
            elif rc.n_long < min_lb:
                abstain = f'few_long_beams:{rc.n_long}<{min_lb}'
            elif not rc.structure_ok:
                abstain = 'no_structure'
            elif not pose_ok:
                abstain = f'pose_{pose_reason}'
            elif rc.through_ratio >= t_up:
                verdict = 'convict'
            elif rc.long_match_ratio >= t_down:
                verdict = 'release'
            else:
                # 正向证据缺失。**它不足以定罪**（那正是被否掉的第二臂），
                # 只足以「拒绝释放」⇒ 落进弃权：不动状态、不置闩锁，但也不背书。
                # 位置 8（位置正确、地图远处不符）就落在这里 —— 旧判据在此定罪，
                # 而它 lm=0.099 也够不上被释放，弃权才是诚实的答案。
                verdict = 'abstain'
                abstain = f'weak_positive:lm={float(rc.long_match_ratio):.3f}<{t_down}'
        self._ray_verdict = verdict
        self._ray_abstain = abstain if verdict == 'abstain' else ''

        # G11：连续 K 次定罪才置位；任何非定罪（含弃权）都清零。
        if verdict == 'convict':
            self._ray_consec_bad += 1
        else:
            self._ray_consec_bad = 0
        k = max(1, int(self.get_parameter('laser_mismatch_k').value))
        self._laser_convicted = self._ray_consec_bad >= k

        if self._laser_convicted and not self._laser_mismatch:
            self.get_logger().warn(
                f'ray CONVICT consec={self._ray_consec_bad}/{k} '
                f'through={rc.through_ratio:.4f}>=T_up({t_up:.2f}) '
                f'evidence={rc.n_evidence} long={rc.n_long} run={rc.max_through_run} '
                f'old_score={scored.laser_score:.3f} old_reason={scored.reason}'
            )
        elif self._laser_mismatch and not self._laser_convicted:
            self.get_logger().warn(
                f'ray conviction CLEARED verdict={verdict} consec={self._ray_consec_bad} '
                f'through={None if rc is None else round(rc.through_ratio, 4)} abstain={self._ray_abstain}'
            )
        self._laser_mismatch = self._laser_convicted

    def _raw_code(self) -> int:
        """Immediate health without latch/heal. Always evaluated (incl. FOLLOW)."""
        self._g_tf_ok = self._tf_ok()
        self._g_amcl_fresh = self._amcl_fresh()
        if not self._g_tf_ok or self._amcl is None or not self._g_amcl_fresh:
            return 1
        xy, yaw = self._cov_xy_yaw()
        # G12：陈旧判决不得无限期持住 status 3。旧代码里一次数据饥饿之后，
        # _laser_mismatch 保留旧值继续参与判决 ⇒ 陈旧判决能把 status 3 一直撑下去。
        self._g_laser_fresh = self._laser_verdict_fresh()
        if self._laser_mismatch and self._g_laser_fresh:
            return 3
        self._g_outside = self._outside_map()
        if self._g_outside:
            return 3
        # _pose_jump 会推进 _last_xy，只能在这里调一次；诊断读存下来的结果。
        self._last_jump = self._pose_jump()
        if self._last_jump:
            return 2
        if xy >= float(self.get_parameter('cov_xy_bad').value) or yaw >= float(
            self.get_parameter('cov_yaw_bad').value
        ):
            return 2
        if xy >= float(self.get_parameter('cov_xy_warn').value) or yaw >= float(
            self.get_parameter('cov_yaw_warn').value
        ):
            return 2
        return 0

    def _publish_status(self, code: int) -> None:
        msg = Int8()
        msg.data = int(code)
        self._status_pub.publish(msg)

    def _emit(self, severity: int, etype: str, body: str) -> None:
        ev = RobotEvent()
        ev.stamp = self.get_clock().now().to_msg()
        ev.severity = severity
        ev.type = etype
        ev.body = body
        ev.capability = 'localization'
        self._event_pub.publish(ev)

    def _stop_motion(self) -> None:
        self._cmd_pub.publish(Twist())

    def _heal_execution_allowed(self) -> bool:
        """Spin/reinit only when not fighting follow, unless explicitly allowed.

        Phase2C-C3: when /xw/localization/phase2c_recovery is true, heal is always
        forbidden so Visual+Laser Reloc remains the single recovery owner.
        """
        if self._phase2c_recovery:
            return False
        if not bool(self.get_parameter('enable_self_heal').value):
            return False
        if self._follow_en and not bool(self.get_parameter('allow_self_heal_during_follow').value):
            # Supervisor must stop follow and set recovery_enable first.
            return bool(self._recovery_en)
        if self._recovery_en:
            return True
        return bool(self._nav_en and not self._follow_en)

    def _self_heal_tick(self) -> None:
        if not self._heal_execution_allowed():
            self._abort_heal_motion('heal not allowed')
            return
        if not self._nav_mode:
            self._heal_started = None
            self._heal_phase = ''
            self._stop_motion()
            return
        now = self._now()
        if self._heal_started is None:
            self._heal_started = now
            self._heal_phase = 'spin'
            self._emit(1, 'loc_self_heal', 'status2 start spin+reinit')
            if self._reinit.service_is_ready():
                self._reinit.call_async(Empty.Request())
            return

        elapsed = now - self._heal_started
        timeout = float(self.get_parameter('self_heal_timeout_sec').value)
        spin_sec = float(self.get_parameter('self_heal_spin_sec').value)
        wz = float(self.get_parameter('self_heal_spin_wz').value)

        if elapsed > timeout:
            self._latched_3 = True
            self._heal_started = None
            self._heal_phase = ''
            self._stop_motion()
            self.get_logger().warn(
                f'status-3 LATCH SET (self-heal timeout {elapsed:.1f}s > {timeout:.1f}s) '
                f'raw={self._last_raw} through='
                f'{None if self._rc is None else round(self._rc.through_ratio, 4)} '
                f'old_score={self._laser_score}'
            )
            self._emit(2, 'loc_needs_attention', 'self-heal timeout → status 3')
            return

        if self._heal_phase == 'spin':
            tw = Twist()
            tw.angular.z = wz
            self._cmd_pub.publish(tw)
            if elapsed >= spin_sec:
                self._heal_phase = 'wait'
                self._stop_motion()
                if self._reinit.service_is_ready():
                    self._reinit.call_async(Empty.Request())
        else:
            self._stop_motion()

    def _publish_detail(self) -> None:
        """把判决依据发到闩住话题。只读诊断，不影响任何判定。"""
        now = self._now()
        xy, yaw = self._cov_xy_yaw()
        d = {
            'stamp': round(now, 3),
            'status': int(self._status),
            'raw': int(self._last_raw),
            'latched_3': bool(self._latched_3),
            'laser': {
                # mismatch = 新口径的定罪（驱动判决）。old_* 是旧口径，只做对照 ——
                # 两者方向相反过一次（真位姿 old 0.42 / 新判据定罪），留着是为了
                # 以后再出问题时能一眼看出「是判据换了还是现场变了」。
                'mismatch': bool(self._laser_mismatch),
                'convicted': bool(self._laser_convicted),
                'consec_bad': int(self._ray_consec_bad),
                'verdict': str(self._ray_verdict),
                'abstain_reason': str(self._ray_abstain),
                'old_mismatch': bool(self._laser_old_mismatch),
                'score': self._laser_score,
                'beams': int(self._laser_beams),
                'matched_ratio': round(float(self._laser_ratio), 4),
                'reason': self._laser_reason,
                # None = 上一次位姿变更后还没重新打分；数值大 = 判决陈旧。
                'verdict_age_sec': (
                    None if self._laser_eval_mono is None
                    else round(now - self._laser_eval_mono, 1)
                ),
            },
            # 新判据的全部原始计数 —— 判决出问题时不靠猜，直接看是哪一步。
            'ray': (
                None if self._rc is None else {
                    'through_ratio': round(float(self._rc.through_ratio), 4),
                    'long_match_ratio': round(float(self._rc.long_match_ratio), 4),
                    'n_evidence': int(self._rc.n_evidence),
                    'n_long': int(self._rc.n_long),
                    'n_match': int(self._rc.n_match),
                    'n_early': int(self._rc.n_early),
                    'n_through': int(self._rc.n_through),
                    'n_grazing': int(self._rc.n_grazing),
                    'n_nohit': int(self._rc.n_nohit),
                    'n_oob': int(self._rc.n_oob),
                    'max_through_run': int(self._rc.max_through_run),
                    'origin_blocked': bool(self._rc.origin_blocked),
                    'origin_dist_m': round(float(self._rc.origin_dist_m), 3),
                    'structure_ok': bool(self._rc.structure_ok),
                    'reason': str(self._rc.reason),
                    'runtime_ms': round(float(self._rc.runtime_sec) * 1e3, 2),
                    't_up': float(self.get_parameter('laser_max_through_ratio').value),
                    't_down': float(self.get_parameter('laser_min_long_match_ratio').value),
                    'k': int(self.get_parameter('laser_mismatch_k').value),
                }
            ),
            'pose_guard': str(self._ray_pose_guard),
            'cov': {
                'xy': round(xy, 4), 'yaw': round(yaw, 4),
                # ⑥-A：塌缩（**下界**判据）。上面两项按 4 位小数打印，塌缩时它们
                # 会显示成 `-0.0` —— 那只是打印精度的假象（A10）。这两个字段才是
                # 「到底塌没塌」的可读答案；xy_raw/yaw_raw 给全精度，便于事后拟合。
                'collapsed': bool(self._cov_collapsed),
                'collapsed_sec': round(self._cov_collapsed_sec, 1),
                'xy_raw': float(xy), 'yaw_raw': float(yaw),
                'xy_min': float(self.get_parameter('cov_xy_min').value),
                'yaw_min': float(self.get_parameter('cov_yaw_min').value),
            },
            # ⑥-B：强制更新的记账。age_sec = 距上次**真正发出**的强制更新多少秒；
            # 它与 ages.amcl_sec 的差就是「跳过」造成的空档，塌缩防护是否在起作用
            # 一眼可见。skipped_degenerate 单调递增，不因塌缩解除而清零。
            'force_update': {
                'age_sec': (
                    None if not self._last_force_update
                    else round(now - self._last_force_update, 1)
                ),
                'period_sec': float(
                    self.get_parameter('amcl_force_update_period_sec').value),
                'skipped_degenerate': int(self._force_skipped),
            },
            # ⑥-C：重播种的记账。⑥-C 实装前恒为 0 / None / ''。
            'reseed': {
                'count': int(self._reseed_count),
                'last_at': (
                    None if self._reseed_last_at is None
                    else round(self._reseed_last_at, 1)
                ),
                'last_reason': str(self._reseed_last_reason),
                # ⑥-C 的验收接口（只读、零订阅者、无 schema）：
                #   static_sec = 当前已连续静止秒数 —— 验收④「正常停在目标点 20 s
                #                不得触发」就是看它有没有到 reseed_static_sec
                #   blocked    = 本 tick 为什么没开火（'moving_18.3s' / 'cooldown_…'
                #                / 'need_operator' / …）；空串 = 已开火
                'static_sec': round(float(self._reseed_static_sec), 1),
                'blocked': str(self._reseed_blocked),
                'phase2c': str(self._phase2c_loc),
                'canonical': str(self._canonical_state),
            },
            'ages': {
                'scan_sec': (
                    None if self._scan_mono is None
                    else round(now - self._scan_mono, 1)
                ),
                'amcl_sec': (
                    None if self._amcl_mono is None
                    else round(now - self._amcl_mono, 1)
                ),
            },
            'gates': {
                'tf_ok': bool(self._g_tf_ok),
                'amcl_fresh': bool(self._g_amcl_fresh),
                'laser_verdict_fresh': bool(self._g_laser_fresh),
                'outside_map': bool(self._g_outside),
                'pose_jump': bool(self._last_jump),
            },
            # 解闩锁倒计时：>0 = 正在计时（raw 已经不是 3 了），到
            # unlatch_hold_sec 就解锁。旧代码没有这个数，也永远到不了 0。
            'unlatch_in_sec': (
                None if (not self._latched_3 or self._raw_not3_since is None)
                else round(
                    max(0.0, float(self.get_parameter('unlatch_hold_sec').value)
                        - (now - self._raw_not3_since)), 1
                )
            ),
            'map_id': self._field_map_id,
        }
        try:
            self._detail_pub.publish(String(data=json.dumps(d, default=str)))
        except Exception:  # noqa: BLE001 — 诊断绝不能让主循环挂掉
            pass

    def _tick(self) -> None:
        try:
            self._tick_body()
        finally:
            self._publish_detail()

    def _tick_body(self) -> None:
        # Detection always runs (FOLLOW included). Execution gated separately.
        if self._follow_en and not self._heal_execution_allowed():
            # Ensure we never leave a heal spin running under follow.
            if self._heal_started is not None or self._heal_phase:
                self._abort_heal_motion('follow active → stop heal motion')

        # ⑥-A 必须先于 ⑥-B：`_maybe_force_amcl_update` 要读本 tick 的 `_cov_collapsed`，
        # 而 `_publish_detail`（在 `_tick` 的 finally 里）要读它和 `_cov_collapsed_sec`。
        self._update_cov_collapse()
        self._maybe_force_amcl_update()
        self._maybe_laser_check()
        # ⑥-C：必须在下面 raw==0/1/3 三条 early-return **之前**调用，否则健康态
        # （raw==0）下永远执行不到。放在 ⑥-B 之后：本 tick 的强制更新按**播种前**的
        # `_cov_collapsed` 决定，播种的效果从下一 tick 起算（我们自己的种子要过
        # `_on_initialpose` 才会把 `_cov_collapsed` 清零）。
        self._maybe_reseed()
        raw = self._raw_code()
        self._last_raw = raw
        now = self._now()

        # ---- C2 解闩锁 ----
        # 旧代码只有 raw==0 才清 _latched_3，而 raw==1（TF/AMCL 瞬时失配，比
        # 「需要重定位」轻）只 return、不清闩锁；紧接着的 `_latched_3 or raw == 3`
        # 分支只要 raw 离开 1 就立刻重新闩上 ⇒ **单向陷阱**：位姿没修好就永远出不来，
        # 而修位姿又常常要先用操作员那个默认空转的按钮（两个缺陷互相咬死）。
        # 现在：raw != 3 连续保持 unlatch_hold_sec ⇒ 解锁，状态降为 raw。
        # 安全性：status 1 仍然 != 0 ⇒ 建库侧要的「loc_status==0 连续 20 s」照样
        # 等不到，什么都没被藏起来；区别只是恢复链路重新可用。
        if raw == 3:
            self._raw_not3_since = None
        elif self._raw_not3_since is None:
            self._raw_not3_since = now
        # 只对 raw==1 解锁。**raw==2 故意不在这里解**：解锁后控制流会落到下面
        # 「raw==2 软保持」那段既有代码，而它在前 status2_hold_sec 内**故意发 status 0**
        # （防抖设计）⇒ 解锁可能瞬时报「一切正常」，把这套安全论证（「解锁不等于放行」）
        # 破坏掉。raw==2 的出路本来就是 initialpose + 自愈，不靠这条。
        if self._latched_3 and raw == 1:
            held = now - float(self._raw_not3_since)
            if held >= float(self.get_parameter('unlatch_hold_sec').value):
                self._latched_3 = False
                self.get_logger().warn(
                    f'status-3 latch CLEARED: raw=1 held {held:.1f}s '
                    f'(>= unlatch_hold_sec) → status 1'
                )
                self._emit(0, 'loc_unlatched', f'raw=1 held {held:.1f}s')

        if raw == 0:
            # FIX-H3 的最后一处（⑥-D）：这条路上原本一声不响地清闩锁。
            # 而它是**唯一**能清闩锁的路（`raw==1` 那条要等 unlatch_hold_sec），
            # 也就是「08-16 那台机器到底是怎么自己好的」事后唯一的证据点 ——
            # 旧代码在这里什么都不留，只能去翻 40 分钟的 python3_*.log 反推。
            # ⚠️ 只在**确实闩着**的时候报：raw==0 是绝大多数 tick 的常态，
            #    无条件打印会把这个 WARN 淹进噪声里，等于没加。
            if self._latched_3:
                self.get_logger().warn(
                    f'status-3 latch CLEARED (raw=0) '
                    f'through='
                    f'{None if self._rc is None else round(self._rc.through_ratio, 4)} '
                    f'old_score={self._laser_score} '
                    f'verdict={self._ray_verdict} abstain={self._ray_abstain}'
                )
            self._raw_bad_since = None
            self._heal_started = None
            self._heal_phase = ''
            self._latched_3 = False
            self._status = 0
            self._stop_motion()
            self._publish_status(0)
            return

        if raw == 1:
            self._status = 1
            self._publish_status(1)
            return

        if self._latched_3 or raw == 3:
            if not self._latched_3:
                # FIX-H3：置位必须留痕。旧代码这条路上 rosout 一个字都没有，
                # 事后只能靠翻 40 分钟的 python3_<pid>_*.log 反推。
                self.get_logger().warn(
                    f'status-3 LATCH SET raw=3 laser_convicted={self._laser_convicted} '
                    f'through={None if self._rc is None else round(self._rc.through_ratio, 4)} '
                    f'verdict={self._ray_verdict} outside_map={self._g_outside} '
                    f'pose_jump={self._last_jump}'
                )
            self._latched_3 = True
            self._status = 3
            self._stop_motion()
            self._publish_status(3)
            return

        # raw == 2
        if self._raw_bad_since is None:
            self._raw_bad_since = now
        hold = float(self.get_parameter('status2_hold_sec').value)
        if now - self._raw_bad_since < hold:
            # Avoid flicker: hold soft-ok until sustained, then surface 2.
            self._status = 0 if not self._nav_mode else 2
            # During follow before recovery: still publish 0 until hold expires
            # so supervisor only reacts to sustained degradation.
            if self._follow_en and not self._recovery_en:
                self._status = 0
            self._publish_status(self._status)
            return

        self._status = 2
        self._publish_status(2)
        if self._heal_execution_allowed():
            self._self_heal_tick()
        elif not self._nav_mode:
            # Non-nav sustained drift → latch 3 (needs attention)
            if now - self._raw_bad_since > hold + 10.0:
                self._latched_3 = True
                self._status = 3
                self._publish_status(3)
                self.get_logger().warn(
                    f'status-3 LATCH SET (idle drift sustained '
                    f'{now - self._raw_bad_since:.1f}s) raw={self._last_raw} '
                    f'through='
                    f'{None if self._rc is None else round(self._rc.through_ratio, 4)} '
                    f'old_score={self._laser_score}'
                )
                self._emit(2, 'loc_needs_attention', 'drift while idle')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationHealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
