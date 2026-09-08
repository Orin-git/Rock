#!/usr/bin/env python3
"""Gen2 localization health 0–3 + optional self-heal (spin + reinitialize).

0 good | 1 not ready | 2 drift (self-heal) | 3 needs intervention (latched until OK)

AMCL only republishes after motion (update_min_*). While odom is nearly static
since the last amcl_pose, that pose is still treated as usable (avoids idle→1).

Phase1: detection is always active (incl. FOLLOW). Execution of spin/reinit is
gated by /xw/localization/recovery_enable so follow is never preempted by
health motion — supervisor stops follow first, then arms recovery.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

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
        self.declare_parameter('pose_jump_m', 0.8)
        self.declare_parameter('outside_map_margin_m', 0.5)
        self.declare_parameter('status2_hold_sec', 4.0)
        self.declare_parameter('self_heal_timeout_sec', 25.0)
        self.declare_parameter('self_heal_spin_wz', 0.35)
        self.declare_parameter('self_heal_spin_sec', 4.0)
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('enable_self_heal', True)
        # If true, heal during follow without supervisor gate (NOT recommended).
        self.declare_parameter('allow_self_heal_during_follow', False)

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
        self._event_pub = self.create_publisher(RobotEvent, '/xw/event', 10)
        self._cmd_pub = self.create_publisher(Twist, '/xw/cmd/motion', 10)

        self._reinit = self.create_client(
            Empty, 'reinitialize_global_localization', callback_group=self._cb
        )

        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / max(hz, 0.5), self._tick, callback_group=self._cb)
        self.get_logger().info(
            'localization_health ready (detect always; heal gated by recovery_enable; '
            'phase2c_recovery blocks spin+reinit)'
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

    def _raw_code(self) -> int:
        """Immediate health without latch/heal. Always evaluated (incl. FOLLOW)."""
        if not self._tf_ok() or self._amcl is None or not self._amcl_fresh():
            return 1
        xy, yaw = self._cov_xy_yaw()
        if self._outside_map():
            return 3
        if self._pose_jump():
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

    def _tick(self) -> None:
        # Detection always runs (FOLLOW included). Execution gated separately.
        if self._follow_en and not self._heal_execution_allowed():
            # Ensure we never leave a heal spin running under follow.
            if self._heal_started is not None or self._heal_phase:
                self._abort_heal_motion('follow active → stop heal motion')

        raw = self._raw_code()
        now = self._now()

        if raw == 0:
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
