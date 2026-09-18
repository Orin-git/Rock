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
        # 负值 = 关闭该轴（cov 可以为负，所以不能用 0 当「关」）。
        self.declare_parameter('cov_xy_min', 3.0e-5)
        self.declare_parameter('cov_yaw_min', 8.0e-6)
        # 塌缩须**连续**保持这么久才置位（防单帧注入尖峰清零计时）。
        # 标定：cov 从健康 2e-2 衰减到阈值 3e-5 约需 75 s ⇒ 20 s 保持只加很小的延迟；
        # 而 20 s 的正常停车停留根本到不了阈值（那一档 cov ≈ 1.8e-3，差 59×）。
        self.declare_parameter('cov_collapse_hold_sec', 20.0)
        # ⑥-B：塌缩期间跳过强制更新（见 _maybe_force_amcl_update 内的说明）。
        self.declare_parameter('amcl_force_skip_when_collapsed', True)
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
        self._reseed_count = 0               # ⑥-C：重播种次数（⑥-C 实装前恒为 0）
        self._reseed_last_at = None          # ⑥-C：上次重播种的单调钟时刻
        self._reseed_last_reason = ''        # ⑥-C：上次重播种的原因

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
            self.get_logger().warn(
                'cov COLLAPSED (sustained %.1fs >= %.1fs): cov_xy=%.3e (< %.1e) '
                'cov_yaw=%.3e (< %.1e) — 滤波器已不再探索。这**只是遥测**，'
                'status 不受影响；退化要走的出口是重播种，不是自旋自愈。'
                % (self._cov_collapsed_sec, hold, xy, lim_xy, yaw, lim_yaw)
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
