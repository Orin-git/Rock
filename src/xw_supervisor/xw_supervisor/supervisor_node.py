#!/usr/bin/env python3
"""Mode FSM: sole business gate for Gen2 sessions.

Session lifecycle is commanded via topics (no nested service calls)
to avoid client/server deadlocks inside the same executor.
"""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from xw_interfaces.msg import PowerState, RobotEvent, RobotState, TaskProgress
from xw_interfaces.srv import GetState, SetMode

MODE_NAMES = {
    0: 'IDLE',
    1: 'MAPPING',
    2: 'NAVIGATING',
    3: 'FOLLOWING',
    4: 'FALL_DETECT',
}

# mode -> session enable topic
SESSION_ENABLE = {
    1: '/xw/slam/enable',
    2: '/xw/nav/enable',
    3: '/xw/follow/enable',
    4: '/xw/fall/enable',
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

        # Keep robot_state VOLATILE (high rate) so CLI/Foxglove default QoS always sees updates.
        self._state_pub = self.create_publisher(RobotState, '/xw/robot_state', 10)
        self._event_pub = self.create_publisher(RobotEvent, '/xw/event', 10)
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)

        # Session enables: transient local so late-joining sessions get last command
        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._session_pubs = {
            mode: self.create_publisher(Bool, topic, latch)
            for mode, topic in SESSION_ENABLE.items()
        }

        self.create_subscription(Bool, 'emergency_stop', self._on_estop, 10)
        self.create_subscription(Bool, 'safety_status', self._on_safety, 10)
        self.create_subscription(PowerState, '/xw/power', self._on_power, 10)

        self.create_service(SetMode, '/xw/supervisor/set_mode', self._on_set_mode, callback_group=self._cb)
        self.create_service(GetState, '/xw/supervisor/get_state', self._on_get_state, callback_group=self._cb)

        self.create_timer(0.5, self._publish_state)
        # latched disables
        for mode in SESSION_ENABLE:
            self._session_pubs[mode].publish(Bool(data=False))
        self.get_logger().info('supervisor ready')

    def _on_estop(self, msg: Bool) -> None:
        was = self._estop
        self._estop = bool(msg.data)
        if self._estop and not was:
            self._emit_event(2, 'emergency_stop', 'pressed')
            if self._mode != 0:
                self._apply_mode(0, 'estop')

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
        s.detail = self._detail
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

    def _disable_all_sessions(self) -> None:
        for mode in SESSION_ENABLE:
            self._set_session(mode, False)

    def _apply_mode(self, target: int, reason: str) -> None:
        self._disable_all_sessions()
        self._mode = target
        if target in SESSION_ENABLE:
            self._set_session(target, True)
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
            res.message = 'emergency stop active'
            res.active_mode = self._mode
            return res

        production = int(self.get_parameter('run_mode').value) == 0
        if production and self._mode != 0 and target != 0 and target != self._mode:
            res.success = False
            res.message = f'busy in {MODE_NAMES[self._mode]} (production)'
            res.active_mode = self._mode
            return res

        reason = f'entered {MODE_NAMES[target]}' if target else 'idle'
        cid = req.command_id or f'mode-{target}'
        self._apply_mode(target, reason)

        if target != 0 and '"map_name"' in (req.payload_json or ''):
            self._active_map = req.payload_json

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

    def _emit_event(self, severity: int, etype: str, body: str) -> None:
        e = RobotEvent()
        e.stamp = self.get_clock().now().to_msg()
        e.severity = severity
        e.type = etype
        e.body = body
        e.capability = 'supervisor'
        self._event_pub.publish(e)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
