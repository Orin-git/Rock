#!/usr/bin/env python3
"""Gen2 localization health 0–3 + optional self-heal (spin + reinitialize).

0 good | 1 not ready | 2 drift (self-heal) | 3 needs intervention (latched until OK)
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

        self._cb = ReentrantCallbackGroup()
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._map: Optional[OccupancyGrid] = None
        self._nav_en = False
        self._follow_en = False
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
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10
        )
        self.create_subscription(OccupancyGrid, 'map', self._on_map, _MAP_QOS)
        self.create_subscription(Bool, '/xw/nav/enable', self._on_nav_en, latch_in)
        self.create_subscription(Bool, '/xw/follow/enable', self._on_follow_en, latch_in)
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
        self.get_logger().info('localization_health ready (0–3 + self-heal)')

    @property
    def _nav_mode(self) -> bool:
        return self._nav_en or self._follow_en

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _on_nav_en(self, msg: Bool) -> None:
        self._nav_en = bool(msg.data)

    def _on_follow_en(self, msg: Bool) -> None:
        was = self._follow_en
        self._follow_en = bool(msg.data)
        if self._follow_en and not was:
            # Visual follow must not fight AMCL self-heal spin (/xw/cmd/motion
            # priority > follow). Freeze heal and clear any in-progress spin.
            self._heal_started = None
            self._heal_phase = ''
            self._raw_bad_since = None
            self._stop_motion()
            self.get_logger().info('follow on → pause loc self-heal (hold map pose)')
        elif was and not self._follow_en:
            self._raw_bad_since = None
            self._heal_started = None
            self._heal_phase = ''
            self.get_logger().info('follow off → loc self-heal armed again')

    def _on_initialpose(self, _msg: PoseWithCovarianceStamped) -> None:
        self._latched_3 = False
        self._heal_started = None
        self._heal_phase = ''
        self._raw_bad_since = None
        self.get_logger().info('initialpose → clear status-3 latch')

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
        """Immediate health without latch/heal."""
        if not self._tf_ok() or self._amcl is None:
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

    def _self_heal_tick(self) -> None:
        if not bool(self.get_parameter('enable_self_heal').value):
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
        # Body-follow is visual-servo (no map). People in /scan + continuous
        # motion inflate AMCL cov; self-heal spin would steal cmd from follow
        # and make the map pose look like it "drifted". Hold last good status.
        if self._follow_en:
            if self._heal_started is not None or self._heal_phase:
                self._heal_started = None
                self._heal_phase = ''
                self._stop_motion()
            self._raw_bad_since = None
            if not self._latched_3:
                self._status = 0 if self._tf_ok() else 1
            self._publish_status(self._status)
            return

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
            # still report 0 until hold expires (avoid flicker)
            self._status = 0 if not self._nav_mode else 2
            self._publish_status(self._status)
            return

        self._status = 2
        self._publish_status(2)
        if self._nav_mode:
            self._self_heal_tick()
        else:
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
