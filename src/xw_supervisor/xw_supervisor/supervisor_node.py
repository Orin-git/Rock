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

        self._cb = ReentrantCallbackGroup()
        self._mode = 0
        self._estop = False
        self._safety_ok = True
        self._loc_status = 1  # not ready until health publishes
        self._power = PowerState()
        self._active_map = ''
        self._detail = 'boot'
        self._fall_en = False
        self._follow_en = False
        self._recharge_en = False
        self._explore_en = False
        self._explore_map = ''

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
            Bool, '/xw/explore/request_disable', self._on_explore_request_disable, 10
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
        self.get_logger().info(
            'supervisor ready (follow/recharge on nav; explore on mapping; fall orthogonal)'
        )

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
        self._loc_status = int(msg.data)

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
