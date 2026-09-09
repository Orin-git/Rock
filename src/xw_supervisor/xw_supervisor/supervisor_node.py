#!/usr/bin/env python3
"""Mode FSM: sole business gate for Gen2 sessions.

Motion modes (mapping/nav) are mutually exclusive for the Nav2/SLAM stack.
Body-follow is an orthogonal task latch (/xw/follow/enable) that requires Nav2
to stay up: enabling follow cancels point/patrol goals but does NOT stop Nav2.
Auto-recharge is orthogonal on nav (/xw/recharge/enable).
Autonomous mapping (frontier) is orthogonal on MAPPING (/xw/explore/enable):
SLAM stays up; a separate explore Nav2 (no AMCL) is managed by xw_explore.
Fall detection is also orthogonal (/xw/fall/enable).

Session lifecycle is commanded via topics (no nested service calls)
to avoid client/server deadlocks inside the same executor.
"""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import SetBool

from xw_interfaces.msg import PowerState, RobotEvent, RobotState, TaskProgress
from xw_interfaces.srv import GetState, SetMode, SetRunMode

MODE_NAMES = {
    0: 'IDLE',
    1: 'MAPPING',
    2: 'NAVIGATING',
    3: 'FOLLOWING',
    4: 'FALL_DETECT',
}

# Soft localization recovery overlay (not a SetMode id — keeps API stable).
LOC_RECOVERY_DETAIL = 'LOCALIZATION_RECOVERY'
NEED_INITIAL_POSE_DETAIL = 'NEED_INITIAL_POSE'

LOC_STATUS_NAMES = {
    0: 'ok',
    1: 'not_ready',
    2: 'drift',
    3: 'needs_attention',
}

# Stack sessions only — follow/fall are orthogonal latches.
MOTION_SESSION = {
    1: '/xw/slam/enable',
    2: '/xw/nav/enable',
}


class SupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_supervisor')
        self.declare_parameter('run_mode', 1)  # 0 production 1 developer
        self.declare_parameter('profile', 'normal')
        # Fall is an orthogonal background latch; default ON so dual-cam RGB+NPU stay warm.
        self.declare_parameter('fall_enable_default', True)
        # Phase2C-C4B production master switch. When true: suppress heal spin+reinit,
        # mirror BOOT/LOST states, keep Reloc ownership semantics.
        self.declare_parameter('phase2c_localization_enabled', False)
        # Phase2C-C1: LOST → cancel nav / block goals / snapshot. Default OFF (no prod change).
        # Does NOT call Relocalizer. Ownership is /xw/localization/phase2c_recovery
        # (independent of recovery_enable, which IDLE clears).
        self.declare_parameter('phase2c_lost_cancel_enabled', False)
        # Phase2C-C3: LOST → STOP → Reloc owned by xw_lost_recovery. Default OFF.
        # When ON (or master ON): suppress health spin+reinit arming; mirror snapshot;
        # do not steal phase2c_recovery ownership from lost_recovery; IDLE must not mid-cut.
        self.declare_parameter('phase2c_lost_recovery_enabled', False)

        self._cb = ReentrantCallbackGroup()
        self._mode = 0
        self._estop = False
        self._safety_ok = True
        self._loc_status = 1  # not ready until health publishes
        self._power = PowerState()
        self._active_map = ''
        self._detail = 'boot'
        self._fall_en = bool(self.get_parameter('fall_enable_default').value)
        self._follow_en = False
        self._recharge_en = False
        self._explore_en = False
        self._explore_map = ''
        self._loc_recovery_en = False
        self._prev_loc_status = 1
        # Phase2C independent recovery latch (survives semantic distinction from recovery_enable).
        self._phase2c_recovery_active = False
        self._phase2c_task_snapshot = ''
        self._phase2c_loc_state = 'READY'
        self._phase2c_goals_blocked = False
        self._canonical_gen = 0
        self._boot_state = ''
        self._last_goal_xy = None  # optional; filled if progress observed later

        # Keep robot_state VOLATILE (high rate) so CLI/Foxglove default QoS always sees updates.
        self._state_pub = self.create_publisher(RobotState, '/xw/robot_state', 10)
        self._event_pub = self.create_publisher(RobotEvent, '/xw/event', 10)
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)

        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._session_pubs = {
            mode: self.create_publisher(Bool, topic, latch)
            for mode, topic in MOTION_SESSION.items()
        }
        self._follow_pub = self.create_publisher(Bool, '/xw/follow/enable', latch)
        self._fall_pub = self.create_publisher(Bool, '/xw/fall/enable', latch)
        self._recharge_pub = self.create_publisher(Bool, '/xw/recharge/enable', latch)
        self._explore_pub = self.create_publisher(Bool, '/xw/explore/enable', latch)
        self._loc_recovery_pub = self.create_publisher(
            Bool, '/xw/localization/recovery_enable', latch
        )
        self._phase2c_recovery_pub = self.create_publisher(
            Bool, '/xw/localization/phase2c_recovery', latch
        )
        self._goals_blocked_pub = self.create_publisher(Bool, '/xw/nav/goals_blocked', latch)
        self._phase2c_loc_pub = self.create_publisher(
            String, '/xw/localization/phase2c_loc_state', latch
        )
        self._nav_cancel_pub = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self._task_snapshot_pub = self.create_publisher(
            String, '/xw/localization/phase2c_task_snapshot', latch
        )
        self._explore_map_pub = self.create_publisher(String, '/xw/explore/map_name', latch)
        self._nav_map_pub = self.create_publisher(String, '/xw/nav/map_name', latch)

        self._set_pc_nav = self.create_client(SetBool, '/xw/camera/set_pointcloud_nav')

        self.create_subscription(Bool, '/xw/chassis/motor_disabled', self._on_motor_disabled, 10)
        self.create_subscription(Bool, 'safety_status', self._on_safety, 10)
        self.create_subscription(PowerState, '/xw/power', self._on_power, 10)
        self.create_subscription(
            Int8, '/xw/localization_status', self._on_loc_status, latch
        )
        self.create_subscription(
            Bool, '/xw/follow/exit_loc_ok', self._on_follow_exit_loc, latch
        )
        self.create_subscription(
            Bool, '/xw/explore/request_disable', self._on_explore_request_disable, 10
        )
        # C3: Supervisor owns snapshot mirror; lost_recovery owns Reloc + STOP latch publishes.
        self.create_subscription(
            String,
            '/xw/localization/phase2c_task_snapshot',
            self._on_phase2c_snapshot,
            latch,
        )
        self.create_subscription(
            String,
            '/xw/localization/phase2c_loc_state',
            self._on_phase2c_loc_state,
            latch,
        )
        self.create_subscription(
            Bool,
            '/xw/localization/phase2c_recovery',
            self._on_phase2c_recovery_ext,
            latch,
        )
        self.create_subscription(String, '/xw/boot/status', self._on_boot_status, latch)
        self.create_subscription(
            String, '/xw/localization/phase2c_event', self._on_canonical, latch
        )
        self._canonical_state_pub = self.create_publisher(
            String, '/xw/localization/phase2c_state', latch
        )

        self.create_service(SetMode, '/xw/supervisor/set_mode', self._on_set_mode, callback_group=self._cb)
        self.create_service(SetRunMode, '/xw/supervisor/set_run_mode', self._on_set_run_mode, callback_group=self._cb)
        self.create_service(GetState, '/xw/supervisor/get_state', self._on_get_state, callback_group=self._cb)
        self.create_service(SetBool, '/xw/supervisor/set_fall', self._on_set_fall, callback_group=self._cb)
        self.create_service(SetBool, '/xw/supervisor/set_follow', self._on_set_follow, callback_group=self._cb)
        self.create_service(SetBool, '/xw/supervisor/set_recharge', self._on_set_recharge, callback_group=self._cb)
        self.create_service(SetBool, '/xw/supervisor/set_explore', self._on_set_explore, callback_group=self._cb)

        self.create_timer(0.5, self._publish_state)
        for mode in MOTION_SESSION:
            self._session_pubs[mode].publish(Bool(data=False))
        self._publish_follow()
        self._publish_fall()
        self._publish_recharge()
        self._publish_explore()
        self._publish_loc_recovery()
        self._publish_phase2c_recovery()
        self._publish_goals_blocked()
        lost_cancel = bool(self.get_parameter('phase2c_lost_cancel_enabled').value)
        lost_rec = bool(self.get_parameter('phase2c_lost_recovery_enabled').value)
        master = bool(self.get_parameter('phase2c_localization_enabled').value)
        self.get_logger().info(
            'supervisor ready (follow/recharge on nav; explore on mapping; '
            f'fall orthogonal default={"on" if self._fall_en else "off"}; '
            f'loc recovery gated; phase2c_master={master}; '
            f'phase2c_lost_cancel={lost_cancel}; '
            f'phase2c_lost_recovery={lost_rec})'
        )

    def _c3_lost_recovery_on(self) -> bool:
        return bool(self.get_parameter('phase2c_localization_enabled').value) or bool(
            self.get_parameter('phase2c_lost_recovery_enabled').value
        )

    def _on_boot_status(self, msg: String) -> None:
        try:
            d = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            d = {}
        self._boot_state = str(d.get('state') or '')
        if self._boot_state in (
            'WAIT_SENSORS',
            'PRE_LOCALIZATION_READY',
            'TRY_CHARGER',
            'TRY_LAST_GOOD',
            'TRY_VISUAL_LASER',
            'AMCL_VERIFY',
            'POST_SEED_AMCL_READY',
            'WAIT_AMCL_SETTLE',
        ):
            self._detail = 'BOOT_LOCALIZING'
        elif self._boot_state == 'VERIFYING_OPERATOR_POSE':
            self._detail = 'VERIFYING_OPERATOR_POSE'
        elif (
            self._boot_state == 'READY'
            and self._phase2c_loc_state in ('', 'READY')
            and self._canonical_gen == 0
        ):
            if self._detail.startswith('BOOT_') or self._detail in (
                'BOOT_LOCALIZING',
                'NEED_OPERATOR',
                'VERIFYING_OPERATOR_POSE',
            ):
                self._detail = 'READY'
        elif self._boot_state in ('UNKNOWN', 'SENSOR_TIMEOUT'):
            self._detail = 'NEED_OPERATOR'

    def _on_phase2c_snapshot(self, msg: String) -> None:
        """Supervisor owns the mirrored task snapshot (C3 Reloc path)."""
        if msg.data:
            self._phase2c_task_snapshot = msg.data

    def _apply_canonical(self, state: str, goals_blocked: bool, generation: int) -> None:
        if int(generation) <= int(self._canonical_gen):
            return
        self._canonical_gen = int(generation)
        self._phase2c_loc_state = str(state or 'READY')
        self._phase2c_goals_blocked = bool(goals_blocked)
        self._canonical_state_pub.publish(
            String(
                data=json.dumps(
                    {
                        'state': self._phase2c_loc_state,
                        'goals_blocked': self._phase2c_goals_blocked,
                        'source': 'supervisor',
                        'generation': int(self._canonical_gen),
                    },
                    separators=(',', ':'),
                )
            )
        )
        self._phase2c_loc_pub.publish(String(data=self._phase2c_loc_state))
        self._goals_blocked_pub.publish(Bool(data=self._phase2c_goals_blocked))
        self._on_phase2c_loc_state(String(data=self._phase2c_loc_state))

    def _on_canonical(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict) or not data.get('state'):
            return
        try:
            gen = int(data.get('generation') or 0)
        except (TypeError, ValueError):
            return
        self._apply_canonical(
            str(data.get('state') or 'READY'),
            bool(data.get('goals_blocked')),
            gen,
        )

    def _on_phase2c_loc_state(self, msg: String) -> None:
        incoming = (msg.data or 'READY').strip() or 'READY'
        # Old latched READY from boot/lost must not cover a newer incident.
        if self._canonical_gen and incoming == 'READY' and self._phase2c_loc_state not in (
            '',
            'READY',
        ):
            return
        if self._canonical_gen and incoming != self._phase2c_loc_state:
            return
        self._phase2c_loc_state = incoming
        # Mirror Phase2C logical states into RobotState.detail.
        st = self._phase2c_loc_state
        if st == 'BOOT_LOCALIZING':
            self._detail = 'BOOT_LOCALIZING'
        elif st == 'LOST':
            self._detail = 'LOST'
        elif st == 'RECOVERING':
            self._detail = LOC_RECOVERY_DETAIL
        elif st == 'VERIFYING_OPERATOR_POSE':
            self._detail = 'VERIFYING_OPERATOR_POSE'
        elif st in ('UNKNOWN', 'NEED_OPERATOR'):
            self._detail = 'NEED_OPERATOR'
        elif st == 'DEGRADED':
            self._detail = 'DEGRADED'
        elif st == 'READY' and self._detail in (
            'BOOT_LOCALIZING',
            LOC_RECOVERY_DETAIL,
            'LOST',
            'NEED_OPERATOR',
            'VERIFYING_OPERATOR_POSE',
            'DEGRADED',
        ):
            self._detail = 'READY'

    def _on_phase2c_recovery_ext(self, msg: Bool) -> None:
        """Mirror external Phase2C latch (xw_lost_recovery) without republishing."""
        if not self._c3_lost_recovery_on():
            return
        self._phase2c_recovery_active = bool(msg.data)

    def _on_motor_disabled(self, msg: Bool) -> None:
        """MCU Flag_Stop → RobotState.emergency_stop (UI only).

        Nav/SLAM stack sessions stay up; motion arbitration blocks drive when disabled.
        """
        was = self._estop
        self._estop = bool(msg.data)
        if self._estop != was:
            if self._estop:
                self._emit_event(2, 'motor_disabled', 'mcu_flag_stop')
            self._publish_state()

    def _on_safety(self, msg: Bool) -> None:
        self._safety_ok = bool(msg.data)

    def _on_loc_status(self, msg: Int8) -> None:
        prev = self._prev_loc_status
        self._loc_status = int(msg.data)
        self._prev_loc_status = self._loc_status
        c3 = self._c3_lost_recovery_on()
        # FOLLOW + sustained DEGRADED/LOST → stop follow; arm heal ONLY if C3 off.
        if self._follow_en and self._loc_status in (2, 3) and prev != self._loc_status:
            if c3:
                # Stop follow; do NOT arm recovery_enable (spin+reinit). Reloc owns heal.
                self._follow_en = False
                self._publish_follow()
                self._loc_recovery_en = False
                self._publish_loc_recovery()
                self._detail = f'follow stopped for Phase2C Reloc (loc={self._loc_status})'
                self._emit_event(2, 'follow_stop_phase2c', self._detail)
                self._publish_state()
            else:
                self._enter_localization_recovery(
                    f'follow interrupted by loc status={self._loc_status}'
                )
        elif self._loc_recovery_en and self._loc_status == 0:
            self._exit_localization_recovery('loc converged → READY')
        elif self._loc_recovery_en and self._loc_status == 3:
            self._fail_localization_recovery('self-heal failed → NEED_INITIAL_POSE')

        # Phase2C-C1 (flag default OFF): cancel nav / block goals. No Reloc call.
        # Skipped when C3 lost_recovery owns STOP+Reloc.
        if (not c3) and bool(self.get_parameter('phase2c_lost_cancel_enabled').value):
            if (
                self._loc_status in (2, 3)
                and prev != self._loc_status
                and (self._mode in (2, 3) or self._nav_active_capability())
            ):
                self._phase2c_enter_lost_cancel(
                    f'loc status={self._loc_status} (phase2c_lost_cancel)'
                )
            elif self._phase2c_recovery_active and self._loc_status == 0:
                # Note: status==0 is NOT full READY proof (latch clear ≠ READY).
                # C1 only releases goal block; C2+ must require stable AMCL window.
                self._phase2c_exit_lost_cancel('loc status=0 (release goal block only)')

    def _on_follow_exit_loc(self, msg: Bool) -> None:
        """Follow session exit handshake: False → localization recovery."""
        if msg.data:
            return
        if self._mode in (2, 3) or self._nav_active_capability():
            if self._c3_lost_recovery_on():
                if self._follow_en:
                    self._follow_en = False
                    self._publish_follow()
                self._loc_recovery_en = False
                self._publish_loc_recovery()
                return
            self._enter_localization_recovery('follow exit loc handshake failed')
            if bool(self.get_parameter('phase2c_lost_cancel_enabled').value):
                self._phase2c_enter_lost_cancel('follow exit loc handshake failed')

    def _nav_active_capability(self) -> bool:
        return self._mode in (2, 3) or bool(self._active_map)

    def _publish_loc_recovery(self) -> None:
        self._loc_recovery_pub.publish(Bool(data=bool(self._loc_recovery_en)))

    def _publish_phase2c_recovery(self) -> None:
        # C3: xw_lost_recovery owns the latch publisher — do not overwrite.
        if self._c3_lost_recovery_on():
            return
        self._phase2c_recovery_pub.publish(Bool(data=bool(self._phase2c_recovery_active)))

    def _publish_goals_blocked(self) -> None:
        if self._c3_lost_recovery_on():
            # Authority publisher: newest canonical incident, not a second latch.
            self._goals_blocked_pub.publish(Bool(data=bool(self._phase2c_goals_blocked)))
            if self._phase2c_loc_state:
                self._phase2c_loc_pub.publish(String(data=self._phase2c_loc_state))
            if self._canonical_gen:
                self._canonical_state_pub.publish(
                    String(
                        data=json.dumps(
                            {
                                'state': self._phase2c_loc_state,
                                'goals_blocked': self._phase2c_goals_blocked,
                                'source': 'supervisor',
                                'generation': int(self._canonical_gen),
                            },
                            separators=(',', ':'),
                        )
                    )
                )
            return
        self._goals_blocked_pub.publish(Bool(data=bool(self._phase2c_recovery_active)))

    def _phase2c_enter_lost_cancel(self, reason: str) -> None:
        """Stop motion planning; keep Safety; snapshot task. No Relocalizer."""
        import time as _time

        if self._c3_lost_recovery_on():
            # C3 path: xw_lost_recovery performs STOP + snapshot + Reloc.
            return

        if self._follow_en:
            task_type = 'follow'
        elif self._recharge_en:
            task_type = 'recharge'
        elif self._mode in (2, 3):
            task_type = 'navigate'
        else:
            task_type = 'none'
        snap = {
            'task_type': task_type,
            'nav_goal': self._last_goal_xy,
            'follow_was_on': bool(self._follow_en),
            'recharge_was_on': bool(self._recharge_en),
            'map_name': self._active_map,
            'reason': reason,
            'stamp': _time.time(),
            'note': 'Phase2C-C1 snapshot; Reloc NOT auto-invoked',
        }
        self._phase2c_task_snapshot = json.dumps(snap, separators=(',', ':'))
        self._task_snapshot_pub.publish(String(data=self._phase2c_task_snapshot))

        if self._follow_en:
            self._follow_en = False
            self._publish_follow()
        if self._recharge_en:
            self._recharge_en = False
            self._publish_recharge()
        if self._mode == 3:
            self._mode = 2
            self._set_session(2, True)

        # Cancel active Nav2 goal (soft). Safety gate remains in bringup.
        # Publish twice — /xw/nav/cancel is volatile; ensures late nav_session sees it.
        self._nav_cancel_pub.publish(Bool(data=True))
        self._nav_cancel_pub.publish(Bool(data=True))

        already = self._phase2c_recovery_active
        self._phase2c_recovery_active = True
        self._publish_phase2c_recovery()
        self._publish_goals_blocked()
        self._detail = f'PHASE2C_LOST_CANCEL: {reason}'
        if not already:
            self._emit_event(2, 'phase2c_lost_cancel', reason)
            self.get_logger().warn(
                f'PHASE2C_LOST_CANCEL: {reason} (goals blocked; no Reloc)'
            )
        self._publish_state()

    def _phase2c_exit_lost_cancel(self, reason: str) -> None:
        if not self._phase2c_recovery_active:
            return
        self._phase2c_recovery_active = False
        self._publish_phase2c_recovery()
        self._publish_goals_blocked()
        self._detail = f'PHASE2C_LOST_CANCEL_CLEARED: {reason}'
        self._emit_event(1, 'phase2c_lost_cancel_cleared', reason)
        self.get_logger().info(f'PHASE2C_LOST_CANCEL cleared: {reason}')
        self._publish_state()

    def _enter_localization_recovery(self, reason: str) -> None:
        # C3: Reloc owns recovery — never arm spin+reinit via recovery_enable.
        if self._c3_lost_recovery_on():
            if self._follow_en:
                self._follow_en = False
                self._publish_follow()
            if self._loc_recovery_en:
                self._loc_recovery_en = False
                self._publish_loc_recovery()
            self._detail = f'Phase2C Reloc path (heal suppressed): {reason}'
            self._publish_state()
            self.get_logger().warn(self._detail)
            return
        if self._loc_recovery_en and not self._follow_en:
            # Already recovering and follow already stopped.
            self._detail = f'{LOC_RECOVERY_DETAIL}: {reason}'
            self._publish_state()
            return
        if self._follow_en:
            self._follow_en = False
            self._publish_follow()
        if self._mode == 3:
            self._mode = 2
            self._set_session(2, True)
        self._loc_recovery_en = True
        self._publish_loc_recovery()
        self._detail = f'{LOC_RECOVERY_DETAIL}: {reason}'
        self._emit_event(2, 'localization_recovery', reason)
        self._publish_state()
        self.get_logger().warn(f'LOCALIZATION_RECOVERY: {reason}')

    def _exit_localization_recovery(self, reason: str) -> None:
        if not self._loc_recovery_en:
            return
        self._loc_recovery_en = False
        self._publish_loc_recovery()
        self._detail = f'READY: {reason}'
        self._emit_event(1, 'localization_ready', reason)
        self._publish_state()
        self.get_logger().info(f'localization recovery cleared: {reason}')

    def _fail_localization_recovery(self, reason: str) -> None:
        self._loc_recovery_en = False
        self._publish_loc_recovery()
        self._detail = f'{NEED_INITIAL_POSE_DETAIL}: {reason}'
        self._emit_event(2, 'need_initial_pose', reason)
        self._publish_state()
        self.get_logger().error(f'NEED_INITIAL_POSE: {reason}')

    def _on_power(self, msg: PowerState) -> None:
        self._power = msg

    def _build_state(self) -> RobotState:
        s = RobotState()
        s.stamp = self.get_clock().now().to_msg()
        s.mode = self._mode
        s.mode_name = MODE_NAMES.get(self._mode, 'UNKNOWN')
        s.run_mode = int(self.get_parameter('run_mode').value)
        s.emergency_stop = self._estop
        s.safety_ok = self._safety_ok
        s.localization_status = int(self._loc_status)
        s.localization_ok = self._loc_status == 0
        s.active_map = self._active_map
        s.profile = str(self.get_parameter('profile').value)
        s.power = self._power
        tags = []
        tags.append('follow=on' if self._follow_en else 'follow=off')
        tags.append('recharge=on' if self._recharge_en else 'recharge=off')
        tags.append('explore=on' if self._explore_en else 'explore=off')
        tags.append('fall=on' if self._fall_en else 'fall=off')
        tags.append(f'loc={LOC_STATUS_NAMES.get(self._loc_status, str(self._loc_status))}')
        if self._phase2c_recovery_active:
            tags.append('phase2c_recovery=on')
        if self._c3_lost_recovery_on() and self._phase2c_loc_state not in ('', 'READY'):
            tags.append(f'phase2c_loc={self._phase2c_loc_state}')
        base = self._detail or ''
        tag_s = ' '.join(tags)
        s.detail = f'{base} | {tag_s}' if base else tag_s
        return s

    def _publish_state(self) -> None:
        self._state_pub.publish(self._build_state())

    def _on_get_state(self, req: GetState.Request, res: GetState.Response):
        res.success = True
        res.message = 'ok'
        res.state = self._build_state()
        return res

    def _set_session(self, mode: int, active: bool) -> None:
        pub = self._session_pubs.get(mode)
        if pub is not None:
            pub.publish(Bool(data=active))

    def _disable_motion_sessions(self) -> None:
        for mode in MOTION_SESSION:
            self._set_session(mode, False)

    def _publish_fall(self) -> None:
        self._fall_pub.publish(Bool(data=bool(self._fall_en)))

    def _publish_follow(self) -> None:
        self._follow_pub.publish(Bool(data=bool(self._follow_en)))

    def _publish_recharge(self) -> None:
        self._recharge_pub.publish(Bool(data=bool(self._recharge_en)))

    def _publish_explore(self) -> None:
        self._explore_pub.publish(Bool(data=bool(self._explore_en)))

    def _clear_recharge(self) -> None:
        if self._recharge_en:
            self._recharge_en = False
            self._publish_recharge()

    def _clear_explore(self) -> None:
        if self._explore_en:
            self._explore_en = False
            self._publish_explore()

    def _on_explore_request_disable(self, msg: Bool) -> None:
        """Frontier finished / session asks to drop the orthogonal latch."""
        if msg.data and self._explore_en:
            self._clear_explore()
            self._detail = 'explore finished (latch cleared)'
            self._publish_state()

    def _set_fall(self, active: bool) -> None:
        self._fall_en = bool(active)
        self._publish_fall()

    def _set_explore(self, active: bool, map_name: str = '') -> tuple[bool, str]:
        """Toggle frontier explore while staying in MAPPING (SLAM kept)."""
        want = bool(active)
        if want:
            name = (map_name or self._explore_map or '').strip()
            if name:
                self._explore_map = name
                self._explore_map_pub.publish(String(data=name))
            if self._mode != 1:
                # Enter mapping; disables nav/follow/recharge
                self._apply_mode(1, 'explore → mapping')
            self._clear_recharge()
            if self._follow_en:
                self._follow_en = False
                self._publish_follow()
            self._explore_en = True
            self._publish_explore()
            self._detail = 'explore task on (slam kept)'
            self._publish_state()
            return True, 'explore on'
        self._clear_explore()
        self._detail = 'explore off'
        self._publish_state()
        return True, 'explore off'

    def _on_set_explore(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        ok, msg = self._set_explore(bool(req.data))
        res.success = bool(ok)
        res.message = msg
        return res

    def _set_follow(self, active: bool) -> tuple[bool, str]:
        """Toggle follow task without tearing down Nav2.

        Requires nav stack to be (or become) active. Returns (ok, message).
        """
        want = bool(active)
        if want:
            if self._mode == 1:
                return False, 'cannot follow while mapping'
            # Ensure Nav2 capability is on
            if self._mode not in (2, 3):
                if not self._active_map:
                    return False, 'enter navigation with a map first (set_mode 2)'
                # Re-latch map name (leaving nav clears /xw/nav/map_name)
                self._nav_map_pub.publish(String(data=self._active_map))
                self._set_pointcloud_nav(True)
                self._disable_motion_sessions()
                self._set_session(2, True)
                self._mode = 3
            else:
                # Already navigating — keep nav enable true, only latch follow
                if self._active_map:
                    self._nav_map_pub.publish(String(data=self._active_map))
                self._set_session(2, True)
                self._mode = 3
            self._follow_en = True
            self._publish_follow()
            self._clear_recharge()
            self._detail = 'follow task on (nav kept)'
            self._publish_state()
            return True, 'follow on'
        # Turn off follow; keep Nav2 if we were navigating/following
        self._follow_en = False
        self._publish_follow()
        if self._mode == 3:
            self._mode = 2
            self._set_session(2, True)
            self._detail = 'follow off (nav kept)'
        else:
            self._detail = 'follow off'
        self._publish_state()
        return True, 'follow off'

    def _on_set_fall(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._set_fall(bool(req.data))
        if self._mode == 4 and not self._fall_en:
            self._mode = 0
            self._detail = 'fall disabled → idle'
        elif self._fall_en and self._mode == 0:
            self._detail = 'fall enabled (background)'
        res.success = True
        res.message = f'fall={"on" if self._fall_en else "off"}'
        self._publish_state()
        return res

    def _on_set_follow(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        ok, msg = self._set_follow(bool(req.data))
        res.success = bool(ok)
        res.message = msg
        return res

    def _set_recharge(self, active: bool) -> tuple[bool, str]:
        want = bool(active)
        if want:
            if self._mode == 1:
                return False, 'cannot recharge while mapping'
            if self._mode not in (2, 3):
                return False, 'enter navigation with a map first (set_mode 2)'
            if self._follow_en:
                self._follow_en = False
                self._publish_follow()
                if self._mode == 3:
                    self._mode = 2
                    self._set_session(2, True)
            self._recharge_en = True
            self._publish_recharge()
            self._detail = 'recharge task on (nav kept)'
            self._publish_state()
            return True, 'recharge on'
        self._clear_recharge()
        self._detail = 'recharge off'
        self._publish_state()
        return True, 'recharge off'

    def _on_set_recharge(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        ok, msg = self._set_recharge(bool(req.data))
        res.success = bool(ok)
        res.message = msg
        return res

    def _set_pointcloud_nav(self, enabled: bool) -> None:
        """Fire-and-forget nav auto pointcloud (no persist)."""
        if not self._set_pc_nav.service_is_ready():
            self.get_logger().warn('set_pointcloud_nav not ready (depth bridge?)')
            return

        req = SetBool.Request()
        req.data = bool(enabled)

        def _done(fut) -> None:
            try:
                r = fut.result()
                self.get_logger().info(
                    f'pointcloud_nav → {enabled}: {getattr(r, "message", r)}'
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'pointcloud_nav call failed: {exc}')

        fut = self._set_pc_nav.call_async(req)
        fut.add_done_callback(_done)

    def _apply_mode(self, target: int, reason: str, payload_json: str = '') -> None:
        prev = self._mode

        map_name = ''
        if payload_json:
            try:
                payload = json.loads(payload_json)
                if isinstance(payload, dict):
                    map_name = str(payload.get('map_name') or '').strip()
            except json.JSONDecodeError:
                map_name = ''

        if target in (2, 3) and map_name:
            self._active_map = map_name
            msg = String()
            msg.data = map_name
            self._nav_map_pub.publish(msg)
        elif target not in (2, 3):
            # Clear latched map name when leaving nav/follow capability
            self._nav_map_pub.publish(String(data=''))

        # Pointcloud for local costmap while nav capability is up (incl. follow)
        nav_cap_prev = prev in (2, 3)
        nav_cap_next = target in (2, 3)
        if nav_cap_prev and not nav_cap_next:
            self._set_pointcloud_nav(False)
        if nav_cap_next and not nav_cap_prev:
            self._set_pointcloud_nav(True)

        if target == 4:
            # Fall-only mode display; do not tear nav if already up — but legacy
            # set_mode(4) from idle just latches fall.
            self._disable_motion_sessions()
            self._set_follow(False)
            self._clear_recharge()
            self._clear_explore()
            self._mode = 4
            self._set_fall(True)
        elif target == 3:
            # FOLLOWING = nav stack ON + follow latch ON (never stop Nav2 for this)
            if map_name:
                self._active_map = map_name
            if self._active_map:
                self._nav_map_pub.publish(String(data=self._active_map))
            # Disable slam only; keep/enable nav
            self._set_session(1, False)
            self._set_session(2, True)
            self._follow_en = True
            self._publish_follow()
            self._clear_recharge()
            self._clear_explore()
            self._mode = 3
        elif target == 2:
            self._set_session(1, False)
            self._set_session(2, True)
            # Entering pure nav turns follow off
            if self._follow_en:
                self._follow_en = False
                self._publish_follow()
            if prev != 2:
                self._clear_recharge()
            self._clear_explore()
            self._mode = 2
        elif target == 1:
            self._disable_motion_sessions()
            if self._follow_en:
                self._follow_en = False
                self._publish_follow()
            self._clear_recharge()
            # Keep explore latch if already on (re-entry); otherwise leave cleared
            if prev != 1:
                self._clear_explore()
            self._set_session(1, True)
            self._mode = 1
        else:
            # IDLE
            self._disable_motion_sessions()
            if self._follow_en:
                self._follow_en = False
                self._publish_follow()
            self._clear_recharge()
            self._clear_explore()
            if self._loc_recovery_en:
                self._loc_recovery_en = False
                self._publish_loc_recovery()
            # Clear Phase2C latch on IDLE; ownership is still independent of recovery_enable
            # during active recovery (this is an explicit mode exit, not a silent drop).
            # C3 / master: NEVER mid-cut Reloc ownership while LOST/RECOVERING/BOOT.
            if self._phase2c_recovery_active:
                if self._c3_lost_recovery_on() and self._phase2c_loc_state in (
                    'LOST',
                    'RECOVERING',
                    'BOOT_LOCALIZING',
                ):
                    self.get_logger().warn(
                        'IDLE requested during Phase2C Reloc — keeping recovery ownership '
                        f'(state={self._phase2c_loc_state})'
                    )
                else:
                    self._phase2c_exit_lost_cancel('entered IDLE')
            self._mode = 0

        self._detail = reason
        self._publish_state()

    def _on_set_mode(self, req: SetMode.Request, res: SetMode.Response):
        target = int(req.mode)
        if target not in MODE_NAMES:
            res.success = False
            res.message = f'unknown mode {target}'
            res.active_mode = self._mode
            return res
        production = int(self.get_parameter('run_mode').value) == 0
        if production and self._mode != 0 and target != 0 and target != self._mode:
            if target == 4 and self._mode in (1, 2, 3):
                self._set_fall(True)
                res.success = True
                res.message = 'fall on (kept motion mode)'
                res.active_mode = self._mode
                self._publish_state()
                return res
            # Allow nav ↔ follow without idle in production (same nav stack)
            if {self._mode, target} <= {2, 3}:
                pass
            elif self._mode in (1, 2, 3) and target in (1, 2, 3):
                res.success = False
                res.message = f'busy in {MODE_NAMES[self._mode]} (production)'
                res.active_mode = self._mode
                return res

        if target == 3 and not self._active_map and not (req.payload_json or '').strip():
            # Follow needs a map/nav context
            if self._mode not in (2, 3):
                res.success = False
                res.message = 'follow requires navigation map (set_mode 2 with map_name first)'
                res.active_mode = self._mode
                return res

        reason = f'entered {MODE_NAMES[target]}' if target else 'idle'
        cid = req.command_id or f'mode-{target}'
        self._apply_mode(target, reason, req.payload_json or '')

        res.success = True
        res.message = MODE_NAMES[target]
        res.active_mode = self._mode

        p = TaskProgress()
        p.stamp = self.get_clock().now().to_msg()
        p.command_id = cid
        p.capability = MODE_NAMES[target].lower()
        p.phase = 'active' if target else 'idle'
        self._progress_pub.publish(p)
        return res

    def _on_set_run_mode(self, req: SetRunMode.Request, res: SetRunMode.Response):
        """0 production / 1 developer (Gen2 default is developer)."""
        target = int(req.run_mode)
        if target not in (0, 1):
            res.success = False
            res.message = f'invalid run_mode {target} (use 0 production / 1 developer)'
            res.run_mode = int(self.get_parameter('run_mode').value)
            return res
        self.set_parameters([
            rclpy.Parameter('run_mode', rclpy.Parameter.Type.INTEGER, target),
        ])
        label = '量产' if target == 0 else '开发者'
        self._detail = f'run_mode={label}'
        self._emit_event(1, 'run_mode', label)
        self._publish_state()
        res.success = True
        res.message = label
        res.run_mode = target
        self.get_logger().info(f'run_mode -> {target} ({label})')
        return res

    def _emit_event(self, severity: int, etype: str, body: str) -> None:
        e = RobotEvent()
        e.stamp = self.get_clock().now().to_msg()
        e.severity = severity
        e.type = etype
        e.body = body
        e.capability = 'supervisor'
        self._event_pub.publish(e)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
