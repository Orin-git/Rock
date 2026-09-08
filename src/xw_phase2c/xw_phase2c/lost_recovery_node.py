#!/usr/bin/env python3
"""xw_lost_recovery — Phase2C-C3 LOST → STOP → Reloc → READY/UNKNOWN (flag default OFF).

Single recovery owner: Visual+Laser Reloc. Does NOT run spin+reinit.
Does NOT belong in production robot.launch.py defaults.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from xw_interfaces.msg import PowerState, RobotState, TaskProgress
from xw_interfaces.srv import Relocalize
from xw_phase2c.recovery_state import Phase2CLocState, TaskSnapshot


_LATCH = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
_AMCL_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

_RC_READY = 0
_RC_UNKNOWN = 1
_RC_AMCL_TIMEOUT = 3


class LostRecoveryNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_lost_recovery')
        self._cb = ReentrantCallbackGroup()

        # Default FALSE — production unchanged unless explicitly enabled (dev launch).
        self.declare_parameter('phase2c_lost_recovery_enabled', False)
        self.declare_parameter('status2_lost_sec', 6.0)
        self.declare_parameter('status3_debounce_sec', 0.5)
        self.declare_parameter('tf_dead_sec', 2.0)
        self.declare_parameter('ready_stable_sec', 2.5)
        self.declare_parameter('ready_cov_xy', 0.80)
        self.declare_parameter('ready_cov_yaw', 0.35)
        self.declare_parameter('amcl_timeout_sec', 20.0)
        self.declare_parameter('reloc_timeout_sec', 60.0)
        self.declare_parameter('p3_max_attempts', 2)
        self.declare_parameter('p3_cooldown_sec', 30.0)
        self.declare_parameter('unknown_cooldown_sec', 60.0)
        self.declare_parameter('post_ready_guard_sec', 5.0)
        self.declare_parameter('auto_resume_nav', True)
        self.declare_parameter('auto_resume_follow', False)
        self.declare_parameter('auto_resume_recharge', True)

        self._logical = Phase2CLocState.READY
        self._busy = False
        self._lock = threading.Lock()
        self._status2_since: Optional[float] = None
        self._status3_since: Optional[float] = None
        self._unknown_until = 0.0
        self._ready_guard_until = 0.0
        self._snapshot: Optional[TaskSnapshot] = None
        self._last_goal: Optional[Dict[str, float]] = None
        self._patrol_active = False
        self._follow_en = False
        self._recharge_en = False
        self._nav_en = False
        self._mode = 0
        self._map_name = ''
        self._loc_status = 1
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._amcl_mono: Optional[float] = None
        self._power = PowerState()
        self._last_xy: Optional[Tuple[float, float]] = None
        self._jump_flag = False
        self._report: Dict[str, Any] = {}

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._phase2c_rec_pub = self.create_publisher(Bool, '/xw/localization/phase2c_recovery', _LATCH)
        self._goals_blocked_pub = self.create_publisher(Bool, '/xw/nav/goals_blocked', _LATCH)
        self._nav_cancel_pub = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self._follow_pub = self.create_publisher(Bool, '/xw/follow/enable', _LATCH)
        self._recharge_pub = self.create_publisher(Bool, '/xw/recharge/enable', _LATCH)
        # Force health heal OFF while we own recovery (do not arm spin+reinit).
        self._heal_gate_pub = self.create_publisher(Bool, '/xw/localization/recovery_enable', _LATCH)
        self._snapshot_pub = self.create_publisher(
            String, '/xw/localization/phase2c_task_snapshot', _LATCH
        )
        self._logical_pub = self.create_publisher(String, '/xw/localization/phase2c_loc_state', _LATCH)
        self._result_pub = self.create_publisher(String, '/xw/localization/phase2c_lost_result', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)

        self._reloc = self.create_client(Relocalize, '/xw/relocalize', callback_group=self._cb)
        self._set_recharge = self.create_client(
            SetBool, '/xw/supervisor/set_recharge', callback_group=self._cb
        )

        self.create_subscription(Int8, '/xw/localization_status', self._on_status, _LATCH)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, _AMCL_QOS)
        self.create_subscription(Bool, '/xw/follow/enable', self._on_follow, _LATCH)
        self.create_subscription(Bool, '/xw/recharge/enable', self._on_recharge, _LATCH)
        self.create_subscription(Bool, '/xw/nav/enable', self._on_nav, _LATCH)
        self.create_subscription(RobotState, '/xw/robot_state', self._on_state, 10)
        self.create_subscription(String, '/xw/nav/map_name', self._on_map, _LATCH)
        self.create_subscription(PoseStamped, '/xw/goal_pose', self._on_goal, 10)
        self.create_subscription(TaskProgress, '/xw/task/progress', self._on_progress, 10)
        self.create_subscription(PowerState, '/xw/power', self._on_power, 10)

        self.create_timer(0.5, self._tick)
        self._publish_logical()
        self.get_logger().info(
            f'lost_recovery ready enabled='
            f'{bool(self.get_parameter("phase2c_lost_recovery_enabled").value)} '
            '(default false; Reloc-only owner; no spin+reinit)'
        )

    def _mono(self) -> float:
        return time.monotonic()

    def _enabled(self) -> bool:
        return bool(self.get_parameter('phase2c_lost_recovery_enabled').value)

    def _publish_logical(self) -> None:
        self._logical_pub.publish(String(data=self._logical.value))

    def _set_logical(self, st: Phase2CLocState) -> None:
        self._logical = st
        self._publish_logical()

    def _on_status(self, msg: Int8) -> None:
        self._loc_status = int(msg.data)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg
        self._amcl_mono = self._mono()
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self._last_xy is not None:
            jump = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
            if jump > 0.8 and self._enabled() and not self._busy:
                # Extreme jump contributes to LOST via tick path using flag.
                self._jump_flag = True
        else:
            self._jump_flag = False
        self._last_xy = (x, y)

    def _on_follow(self, msg: Bool) -> None:
        self._follow_en = bool(msg.data)

    def _on_recharge(self, msg: Bool) -> None:
        self._recharge_en = bool(msg.data)

    def _on_nav(self, msg: Bool) -> None:
        self._nav_en = bool(msg.data)

    def _on_state(self, msg: RobotState) -> None:
        self._mode = int(msg.mode)
        if msg.active_map:
            self._map_name = str(msg.active_map)

    def _on_map(self, msg: String) -> None:
        if msg.data.strip():
            self._map_name = msg.data.strip()

    def _on_goal(self, msg: PoseStamped) -> None:
        self._last_goal = {
            'x': float(msg.pose.position.x),
            'y': float(msg.pose.position.y),
            'yaw': self._yaw_from_quat(msg.pose.orientation),
            'frame': msg.header.frame_id or 'map',
        }
        self._patrol_active = False

    def _on_progress(self, msg: TaskProgress) -> None:
        if msg.capability != 'nav':
            return
        if msg.phase == 'patrol_start':
            self._patrol_active = True
        elif msg.phase in ('goal_accepted', 'executing'):
            self._patrol_active = False
            try:
                d = json.loads(msg.detail or '{}')
                if 'x' in d and 'y' in d:
                    self._last_goal = {
                        'x': float(d['x']),
                        'y': float(d['y']),
                        'yaw': float(d.get('yaw', 0.0)),
                        'frame': 'map',
                    }
            except (json.JSONDecodeError, TypeError, ValueError):
                pass

    def _on_power(self, msg: PowerState) -> None:
        self._power = msg

    @staticmethod
    def _yaw_from_quat(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _tf_ok(self, parent: str, child: str) -> bool:
        try:
            tf = self._tf.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
            )
            if tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0:
                return True
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            return age < float(self.get_parameter('tf_dead_sec').value)
        except TransformException:
            return False

    def _outside_map_hint(self) -> bool:
        # Health already encodes out-of-map as status 3; no duplicate map sub required.
        return self._loc_status == 3

    def _evaluate_lost_reason(self) -> Optional[str]:
        """Debounced LOST declare. None = not LOST."""
        if not self._enabled():
            return None
        if self._busy or self._logical in (Phase2CLocState.RECOVERING,):
            return None
        if self._mono() < self._unknown_until:
            return None
        if self._mono() < self._ready_guard_until:
            return None
        # Only while nav capability / follow / recharge / navigating
        if not (self._nav_en or self._follow_en or self._recharge_en or self._mode in (2, 3)):
            self._status2_since = None
            self._status3_since = None
            return None

        now = self._mono()
        if not self._tf_ok('map', 'odom') or not self._tf_ok('map', 'base_link'):
            return 'tf_dead'

        if self._loc_status == 3:
            if self._status3_since is None:
                self._status3_since = now
            elif now - self._status3_since >= float(self.get_parameter('status3_debounce_sec').value):
                return 'status_3'
            return None
        self._status3_since = None

        if self._loc_status == 2:
            if self._status2_since is None:
                self._status2_since = now
            elif now - self._status2_since >= float(self.get_parameter('status2_lost_sec').value):
                return 'status_2_sustained'
            return None
        self._status2_since = None

        if getattr(self, '_jump_flag', False):
            self._jump_flag = False
            return 'pose_jump'

        if self._loc_status == 0:
            if self._logical == Phase2CLocState.DEGRADED:
                self._set_logical(Phase2CLocState.READY)
            return None

        if self._loc_status == 1:
            self._set_logical(Phase2CLocState.DEGRADED)
        return None

    def _tick(self) -> None:
        if not self._enabled():
            return
        reason = self._evaluate_lost_reason()
        if reason:
            self._start_recovery(reason)

    def _start_recovery(self, reason: str) -> None:
        with self._lock:
            if self._busy:
                return
            self._busy = True
        threading.Thread(target=self._recovery_worker, args=(reason,), daemon=True).start()

    def _stop_motion_first(self, reason: str) -> TaskSnapshot:
        """STOP before Reloc — order fixed by C3 design."""
        if self._follow_en:
            task_type = 'follow'
        elif self._recharge_en:
            task_type = 'recharge'
        elif self._patrol_active:
            task_type = 'patrol'
        elif self._nav_en or self._mode in (2, 3):
            task_type = 'navigate'
        else:
            task_type = 'none'

        snap = TaskSnapshot(
            task_type=task_type,
            nav_goal=dict(self._last_goal) if self._last_goal else None,
            follow_was_on=bool(self._follow_en),
            recharge_was_on=bool(self._recharge_en),
            patrol_was_on=bool(self._patrol_active),
            map_name=self._map_name,
            reason=reason,
            stamp=time.time(),
        )
        raw = json.loads(snap.to_json())
        raw['owner'] = 'xw_lost_recovery'
        raw['supervisor_owns_snapshot'] = True  # mirrored by Supervisor when C3 flag on
        self._snapshot = snap
        self._snapshot_pub.publish(String(data=json.dumps(raw, separators=(',', ':'))))

        # 1-4 STOP sequence
        self._nav_cancel_pub.publish(Bool(data=True))
        self._nav_cancel_pub.publish(Bool(data=True))
        self._goals_blocked_pub.publish(Bool(data=True))
        if self._follow_en:
            self._follow_en = False
            self._follow_pub.publish(Bool(data=False))
        if self._recharge_en:
            self._recharge_en = False
            self._recharge_pub.publish(Bool(data=False))
        # 5-6 snapshot already; phase2c_recovery + heal OFF
        self._heal_gate_pub.publish(Bool(data=False))  # forbid spin+reinit owner
        self._phase2c_rec_pub.publish(Bool(data=True))
        self.get_logger().warn(f'LOST STOP complete reason={reason} task={task_type}')
        return snap

    def _call_reloc(self) -> Dict[str, Any]:
        if not self._reloc.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'result_code': -1, 'error': 'relocalize_unavailable'}
        req = Relocalize.Request()
        req.map_name = self._map_name or 'vp'
        req.force_visual = True
        req.max_candidates = 10
        req.apply_initial_pose = True
        req.allow_motion = False
        fut = self._reloc.call_async(req)
        timeout = float(self.get_parameter('reloc_timeout_sec').value)
        t0 = self._mono()
        while self._mono() - t0 < timeout and rclpy.ok() and not fut.done():
            time.sleep(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'result_code': -1, 'error': 'relocalize_timeout'}
        res = fut.result()
        return {
            'ok': bool(res.success),
            'result_code': int(res.result_code),
            'laser_score': float(res.laser_score),
            'amcl_convergence_sec': float(res.amcl_convergence_sec),
            'diagnostics_json': res.diagnostics_json,
        }

    def _amcl_ready_stable(self) -> Tuple[bool, Dict[str, Any]]:
        """Post-handoff READY — status0 alone insufficient; need cov+TF+window."""
        need = float(self.get_parameter('ready_stable_sec').value)
        cov_xy_lim = float(self.get_parameter('ready_cov_xy').value)
        cov_yaw_lim = float(self.get_parameter('ready_cov_yaw').value)
        timeout = float(self.get_parameter('amcl_timeout_sec').value)
        t0 = self._mono()
        stable_since: Optional[float] = None
        last: Dict[str, Any] = {}
        while self._mono() - t0 < timeout and rclpy.ok():
            time.sleep(0.1)
            if self._amcl is None:
                stable_since = None
                continue
            c = self._amcl.pose.covariance
            cov_xy = max(float(c[0]), float(c[7]))
            cov_yaw = float(c[35])
            tf_ok = self._tf_ok('map', 'odom') and self._tf_ok('map', 'base_link')
            status_ok = self._loc_status == 0
            cov_ok = cov_xy <= cov_xy_lim and cov_yaw <= cov_yaw_lim
            gates = tf_ok and cov_ok and status_ok
            if gates:
                if stable_since is None:
                    stable_since = self._mono()
                stable_for = self._mono() - stable_since
            else:
                stable_since = None
                stable_for = 0.0
            last = {
                'cov_xy': cov_xy,
                'cov_yaw': cov_yaw,
                'tf_ok': tf_ok,
                'loc_status': self._loc_status,
                'stable_for_sec': stable_for,
                'gates_ok': gates,
            }
            if gates and stable_for >= need:
                last['ready'] = True
                return True, last
        last['ready'] = False
        return False, last

    def _resume_tasks(self, snap: TaskSnapshot) -> Dict[str, Any]:
        policy = {
            'nav': 'skip',
            'follow': 'no_auto',
            'recharge': 'skip',
        }
        # Clear blocks before any resume publish
        self._goals_blocked_pub.publish(Bool(data=False))
        self._phase2c_rec_pub.publish(Bool(data=False))

        if snap.task_type in ('navigate', 'patrol') and bool(
            self.get_parameter('auto_resume_nav').value
        ):
            g = snap.nav_goal
            if g and 'x' in g and 'y' in g:
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = str(g.get('frame') or 'map')
                msg.pose.position.x = float(g['x'])
                msg.pose.position.y = float(g['y'])
                yaw = float(g.get('yaw') or 0.0)
                msg.pose.orientation.z = math.sin(yaw * 0.5)
                msg.pose.orientation.w = math.cos(yaw * 0.5)
                time.sleep(0.2)
                self._goal_pub.publish(msg)
                policy['nav'] = 'replan_published'
            else:
                policy['nav'] = 'no_goal_saved'
        # Follow: never auto
        policy['follow'] = 'no_auto'
        # Recharge: re-check power
        if snap.recharge_was_on and bool(self.get_parameter('auto_resume_recharge').value):
            if self._power.charging or self._power.docked:
                policy['recharge'] = 'already_charging_skip'
            elif self._set_recharge.service_is_ready():
                req = SetBool.Request()
                req.data = True
                self._set_recharge.call_async(req)
                policy['recharge'] = 'set_recharge_requested'
            else:
                policy['recharge'] = 'service_unavailable'
        return policy

    def _recovery_worker(self, reason: str) -> None:
        t0 = self._mono()
        self._set_logical(Phase2CLocState.LOST)
        snap = self._stop_motion_first(reason)
        self._set_logical(Phase2CLocState.RECOVERING)
        attempts = int(self.get_parameter('p3_max_attempts').value)
        cooldown = float(self.get_parameter('p3_cooldown_sec').value)
        stages = []
        final = 'UNKNOWN'
        reloc_out: Dict[str, Any] = {}
        try:
            for attempt in range(1, attempts + 1):
                # Ensure heal stays disarmed every attempt
                self._heal_gate_pub.publish(Bool(data=False))
                out = self._call_reloc()
                reloc_out = out
                code = int(out.get('result_code', -1))
                stages.append({'attempt': attempt, 'reloc': out})
                if code == _RC_READY or (out.get('ok') and code == _RC_READY):
                    ready, amcl_diag = self._amcl_ready_stable()
                    stages[-1]['amcl_ready'] = amcl_diag
                    if ready:
                        final = 'READY'
                        break
                    stages[-1]['amcl_ready_fail'] = True
                if attempt < attempts:
                    self.get_logger().warn(
                        f'Reloc fail/unknown code={code}; cooldown {cooldown:.0f}s'
                    )
                    time.sleep(cooldown)

            resume_policy = {'nav': 'blocked', 'follow': 'no_auto', 'recharge': 'blocked'}
            if final == 'READY':
                self._set_logical(Phase2CLocState.READY)
                resume_policy = self._resume_tasks(snap)
                self._ready_guard_until = self._mono() + float(
                    self.get_parameter('post_ready_guard_sec').value
                )
            else:
                self._set_logical(Phase2CLocState.UNKNOWN)
                # Keep goals blocked; NEED_OPERATOR
                self._goals_blocked_pub.publish(Bool(data=True))
                self._phase2c_rec_pub.publish(Bool(data=False))  # drop RGB load
                self._unknown_until = self._mono() + float(
                    self.get_parameter('unknown_cooldown_sec').value
                )
                resume_policy = {
                    'nav': 'forbidden',
                    'follow': 'forbidden',
                    'recharge': 'forbidden',
                    'detail': 'NEED_OPERATOR',
                }

            self._report = {
                'final': final,
                'reason': reason,
                'snapshot': json.loads(snap.to_json()),
                'stages': stages,
                'resume_policy': resume_policy,
                'total_sec': self._mono() - t0,
                'false_recovery_note': 'debounce+single_owner; no spin+reinit',
            }
            self._result_pub.publish(String(data=json.dumps(self._report, default=str)))
            self.get_logger().info(
                f'LOST recovery done final={final} t={self._report["total_sec"]:.1f}s '
                f'resume={resume_policy}'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'lost recovery exception: {exc}')
            self._set_logical(Phase2CLocState.UNKNOWN)
            self._goals_blocked_pub.publish(Bool(data=True))
            self._phase2c_rec_pub.publish(Bool(data=False))
            self._unknown_until = self._mono() + float(
                self.get_parameter('unknown_cooldown_sec').value
            )
        finally:
            with self._lock:
                self._busy = False
            self._publish_logical()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LostRecoveryNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
