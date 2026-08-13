#!/usr/bin/env python3
"""Mode FSM: sole business gate for Gen2 sessions.

Motion modes (mapping/nav/follow) are mutually exclusive.
Fall detection is an orthogonal feature latch (/xw/fall/enable) that survives
mode switches (except estop → idle still keeps fall unless cleared elsewhere).

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
from std_msgs.msg import Bool, String
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

# Motion sessions only — fall is orthogonal and not listed here.
MOTION_SESSION = {
    1: '/xw/slam/enable',
    2: '/xw/nav/enable',
    3: '/xw/follow/enable',
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
        self._power = PowerState()
        self._active_map = ''
        self._detail = 'boot'
        self._fall_en = False

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
        self._fall_pub = self.create_publisher(Bool, '/xw/fall/enable', latch)
        self._nav_map_pub = self.create_publisher(String, '/xw/nav/map_name', latch)

        self._set_pc_nav = self.create_client(SetBool, '/xw/camera/set_pointcloud_nav')

        self.create_subscription(Bool, '/xw/chassis/motor_disabled', self._on_motor_disabled, 10)
        self.create_subscription(Bool, 'safety_status', self._on_safety, 10)
        self.create_subscription(PowerState, '/xw/power', self._on_power, 10)

        self.create_service(SetMode, '/xw/supervisor/set_mode', self._on_set_mode, callback_group=self._cb)
        self.create_service(SetRunMode, '/xw/supervisor/set_run_mode', self._on_set_run_mode, callback_group=self._cb)
        self.create_service(GetState, '/xw/supervisor/get_state', self._on_get_state, callback_group=self._cb)
        self.create_service(SetBool, '/xw/supervisor/set_fall', self._on_set_fall, callback_group=self._cb)

        self.create_timer(0.5, self._publish_state)
        for mode in MOTION_SESSION:
            self._session_pubs[mode].publish(Bool(data=False))
        self._publish_fall()
        self.get_logger().info('supervisor ready (fall orthogonal, nav auto pointcloud)')

    def _on_motor_disabled(self, msg: Bool) -> None:
        """MCU Flag_Stop → RobotState.emergency_stop (UI); cancel motion mode if engaged."""
        was = self._estop
        self._estop = bool(msg.data)
        if self._estop and not was:
            self._emit_event(2, 'motor_disabled', 'mcu_flag_stop')
            if self._mode != 0:
                self._apply_mode(0, 'motor_disabled')

    def _on_safety(self, msg: Bool) -> None:
        self._safety_ok = bool(msg.data)

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
        s.localization_ok = True
        s.active_map = self._active_map
        s.profile = str(self.get_parameter('profile').value)
        s.power = self._power
        fall_tag = 'fall=on' if self._fall_en else 'fall=off'
        base = self._detail or ''
        s.detail = f'{base} | {fall_tag}' if base else fall_tag
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

    def _set_fall(self, active: bool) -> None:
        self._fall_en = bool(active)
        self._publish_fall()

    def _on_set_fall(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._set_fall(bool(req.data))
        if self._mode == 4 and not self._fall_en:
            self._mode = 0
            self._detail = 'fall disabled → idle'
        elif self._fall_en and self._mode == 0:
            # Stay IDLE but reflect fall in detail; mode 4 only via set_mode(4).
            self._detail = 'fall enabled (background)'
        res.success = True
        res.message = f'fall={"on" if self._fall_en else "off"}'
        self._publish_state()
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

        if target == 2 and map_name:
            self._active_map = map_name
            msg = String()
            msg.data = map_name
            self._nav_map_pub.publish(msg)
        elif target != 2:
            # Clear latched map name when leaving nav
            self._nav_map_pub.publish(String(data=''))

        if prev == 2 and target != 2:
            self._set_pointcloud_nav(False)
        if target == 2 and prev != 2:
            self._set_pointcloud_nav(True)

        self._disable_motion_sessions()

        if target == 4:
            self._mode = 4
            self._set_fall(True)
        elif target in MOTION_SESSION:
            self._mode = target
            self._set_session(target, True)
            # Keep fall_en as-is (orthogonal).
        else:
            self._mode = 0
            # Keep fall_en as-is.

        self._detail = reason
        self._publish_state()

    def _on_set_mode(self, req: SetMode.Request, res: SetMode.Response):
        target = int(req.mode)
        if target not in MODE_NAMES:
            res.success = False
            res.message = f'unknown mode {target}'
            res.active_mode = self._mode
            return res
        if self._estop and target != 0:
            res.success = False
            res.message = 'motor disabled (MCU Flag_Stop)'
            res.active_mode = self._mode
            return res

        production = int(self.get_parameter('run_mode').value) == 0
        if production and self._mode != 0 and target != 0 and target != self._mode:
            # set_mode(4) while in a motion mode → only latch fall, keep motion.
            if target == 4 and self._mode in MOTION_SESSION:
                self._set_fall(True)
                res.success = True
                res.message = 'fall on (kept motion mode)'
                res.active_mode = self._mode
                self._publish_state()
                return res
            if self._mode in MOTION_SESSION and target in MOTION_SESSION:
                res.success = False
                res.message = f'busy in {MODE_NAMES[self._mode]} (production)'
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
