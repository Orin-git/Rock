#!/usr/bin/env python3
"""xw_lost_recovery — Phase2C LOST → STOP → R1/R2/R3 → READY/UNKNOWN.

Cascade (one candidate seed at a time, never parallel):
  R1 CURRENT_POSE → R2 LAST_GOOD → R3 VISUAL_GLOBAL → UNKNOWN

R1/R2 are proposals only. READY requires a fresh S1 laser gate (>=0.38)
then /initialpose → POST_SEED_AMCL_READY. Status/cov alone is never READY.
Does NOT run spin+reinit. Does not retune ORB/NPU/Depth/laser/AMCL.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Empty, SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from xw_interfaces.msg import PowerState, RobotState, TaskProgress
from xw_interfaces.srv import Relocalize
from xw_phase2c.canonical_state import (
    CANONICAL_TOPIC,
    SUBMIT_TOPIC,
    is_newer,
    latched_pose_may_satisfy_ready,
    make_event,
    parse_event,
)
from xw_phase2c.last_good_pose import validate_as_proposal
from xw_phase2c.laser_prior_verify import (
    MIN_LASER_SCORE,
    verify_pose_with_laser,
    verify_prior_in_window,
)
from xw_phase2c.ownership import OWNER_TOPIC, InitialPoseOwner, OwnershipGuard
from xw_phase2c.recovery_state import Phase2CLocState, TaskSnapshot
from xw_global_reloc.laser_verify import DistanceField


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
_MAP_QOS = QoSProfile(
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

        # Production master OR legacy C3 flag (either enables LOST Reloc path).
        self.declare_parameter('phase2c_localization_enabled', False)
        # Default FALSE — production unchanged unless master/legacy flag ON.
        self.declare_parameter('phase2c_lost_recovery_enabled', False)
        self.declare_parameter('status2_lost_sec', 6.0)
        # Must exceed typical ~2s AMCL self-pullback. 0.5s was false-triggering
        # global R1/R2/R3 on brief laser/cov transients. Tick is 1 Hz so the
        # effective wait is ~ceil(this) seconds before LOST.
        self.declare_parameter('status3_debounce_sec', 3.0)
        self.declare_parameter('pose_jump_debounce_sec', 3.0)
        self.declare_parameter('tf_dead_sec', 2.0)
        self.declare_parameter('ready_stable_sec', 2.5)
        self.declare_parameter('ready_cov_xy', 0.80)
        self.declare_parameter('ready_cov_yaw', 0.35)
        self.declare_parameter('amcl_timeout_sec', 20.0)
        self.declare_parameter('maps_dir', '/ros2_ws/maps')
        self.declare_parameter('min_laser_score', MIN_LASER_SCORE)
        self.declare_parameter('min_valid_beams', 20)
        self.declare_parameter('scan_fresh_sec', 1.5)
        # Stationary AMCL often does not republish. Laser gate is the R1
        # protection; only an ancient pose is skipped.
        self.declare_parameter('amcl_pose_fresh_sec', 60.0)
        self.declare_parameter('sensor_arm_sec', 3.0)
        self.declare_parameter('post_seed_settle_sec', 8.0)
        self.declare_parameter('r_seed_cov_xy', 0.25)
        self.declare_parameter('r_seed_cov_yaw', 0.15)
        self.declare_parameter('last_good_max_age_sec', 7 * 24 * 3600.0)
        self.declare_parameter('last_good_min_quality', 0.35)
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
        self._session_id = 0
        self._status2_since: Optional[float] = None
        self._status3_since: Optional[float] = None
        self._jump_since: Optional[float] = None
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
        self._boot_busy = False
        self._boot_state = ''
        self._owner = OwnershipGuard(InitialPoseOwner.LOST)
        self._scan: Optional[LaserScan] = None
        self._scan_mono: Optional[float] = None
        self._map: Optional[OccupancyGrid] = None
        self._field: Optional[DistanceField] = None
        self._seeds_this_session = 0
        self._sequential_fallback_seed_count = 0

        # TF / scan / map only inside an active recovery — never IDLE.
        self._tf: Optional[Buffer] = None
        self._tf_listener = None
        self._scan_sub = None
        self._map_sub = None

        self._phase2c_rec_pub = self.create_publisher(Bool, '/xw/localization/phase2c_recovery', _LATCH)
        self._nav_cancel_pub = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self._follow_pub = self.create_publisher(Bool, '/xw/follow/enable', _LATCH)
        self._recharge_pub = self.create_publisher(Bool, '/xw/recharge/enable', _LATCH)
        # Force health heal OFF while we own recovery (do not arm spin+reinit).
        self._heal_gate_pub = self.create_publisher(Bool, '/xw/localization/recovery_enable', _LATCH)
        self._snapshot_pub = self.create_publisher(
            String, '/xw/localization/phase2c_task_snapshot', _LATCH
        )
        # System truth is supervisor-only. Lost submits events only.
        self._canonical_pub = self.create_publisher(String, SUBMIT_TOPIC, _LATCH)
        self._canonical_gen = 0
        self._external_hold = False
        self._result_pub = self.create_publisher(String, '/xw/localization/phase2c_lost_result', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)
        self._owner_pub = self.create_publisher(String, OWNER_TOPIC, _LATCH)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        self._reloc = self.create_client(Relocalize, '/xw/relocalize', callback_group=self._cb)
        self._nomotion = self.create_client(Empty, '/request_nomotion_update', callback_group=self._cb)
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
        self.create_subscription(String, '/xw/boot/status', self._on_boot_status, _LATCH)
        self.create_subscription(String, OWNER_TOPIC, self._on_owner, _LATCH)
        self.create_subscription(String, CANONICAL_TOPIC, self._on_canonical, _LATCH)

        # 1 Hz status debounce only. No TF scan on this timer.
        self.create_timer(1.0, self._tick)
        self._publish_logical()
        self.get_logger().info(
            f'lost_recovery ready enabled={self._enabled()} '
            f'(master={bool(self.get_parameter("phase2c_localization_enabled").value)}; '
            'IDLE: status events only, no TF/scan; cascade R1→R2→R3; no spin+reinit)'
        )

    def _mono(self) -> float:
        return time.monotonic()

    def _enabled(self) -> bool:
        return bool(self.get_parameter('phase2c_localization_enabled').value) or bool(
            self.get_parameter('phase2c_lost_recovery_enabled').value
        )

    def _on_boot_status(self, msg: String) -> None:
        try:
            d = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            d = {}
        self._boot_state = str(d.get('state') or '')
        self._boot_busy = bool(d.get('busy'))

    def _on_owner(self, msg: String) -> None:
        self._owner.on_remote(msg.data or '')

    def _on_canonical(self, msg: String) -> None:
        event = parse_event(msg.data or '')
        if not event.get('state') or not is_newer(event, self._canonical_gen):
            return
        self._canonical_gen = int(event['generation'])
        src = str(event.get('source') or '')
        if src.startswith('lost'):
            return
        # Operator/boot owns the newer incident. Do not cover it with a stale latch.
        self._external_hold = True
        try:
            self._logical = Phase2CLocState(str(event['state']))
        except ValueError:
            return
        if self._logical in (
            Phase2CLocState.READY,
            Phase2CLocState.NEED_OPERATOR,
            Phase2CLocState.VERIFYING_OPERATOR_POSE,
        ):
            self._status2_since = None
            self._status3_since = None
            self._jump_since = None
            self._jump_flag = False

    def _emit_canonical(self, state: str, goals_blocked: bool, source: str) -> None:
        if self._external_hold and not source.startswith('lost_stop'):
            return
        self._external_hold = False
        self._canonical_gen = int(time.time_ns())
        self._canonical_pub.publish(
            String(
                data=make_event(
                    state,
                    goals_blocked,
                    source,
                    incident_id=int(self._session_id),
                    generation=self._canonical_gen,
                )
            )
        )

    def _publish_logical(self) -> None:
        if self._external_hold:
            return
        blocked = self._logical in (
            Phase2CLocState.LOST,
            Phase2CLocState.RECOVERING,
            Phase2CLocState.NEED_OPERATOR,
            Phase2CLocState.VERIFYING_OPERATOR_POSE,
            Phase2CLocState.BOOT_LOCALIZING,
        )
        self._emit_canonical(self._logical.value, blocked, 'lost')

    def _set_logical(self, st: Phase2CLocState) -> None:
        self._external_hold = False
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

    def _arm_tf(self) -> None:
        if self._tf is None:
            self._tf = Buffer()
            self._tf_listener = TransformListener(self._tf, self, spin_thread=False)

    def _disarm_tf(self) -> None:
        listener = self._tf_listener
        self._tf_listener = None
        self._tf = None
        if listener is not None:
            try:
                listener.unregister()
            except Exception:  # noqa: BLE001
                pass

    def _arm_sensors(self) -> None:
        """Scan + map only while a recovery owns the incident. Never IDLE."""
        if self._scan_sub is None:
            self._scan_sub = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        if self._map_sub is None:
            self._map_sub = self.create_subscription(OccupancyGrid, '/map', self._on_map, _MAP_QOS)

    def _disarm_sensors(self) -> None:
        for attr in ('_scan_sub', '_map_sub'):
            sub = getattr(self, attr)
            if sub is not None:
                try:
                    self.destroy_subscription(sub)
                except Exception:  # noqa: BLE001
                    pass
                setattr(self, attr, None)
        self._scan = None
        self._scan_mono = None
        self._field = None

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_mono = self._mono()

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        self._field = None

    def _laser_floor(self) -> float:
        score = float(self.get_parameter('min_laser_score').value)
        return MIN_LASER_SCORE if score < MIN_LASER_SCORE else score

    def _ensure_field(self) -> bool:
        if self._field is not None:
            return True
        if self._map is None:
            return False
        try:
            self._field = DistanceField(self._map)
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'DistanceField build failed: {exc}')
            self._field = None
            return False

    def _scan_fresh(self) -> bool:
        if self._scan is None or self._scan_mono is None:
            return False
        return (self._mono() - self._scan_mono) <= float(self.get_parameter('scan_fresh_sec').value)

    def _wait_sensors(self) -> None:
        self._arm_tf()
        self._arm_sensors()
        timeout = float(self.get_parameter('sensor_arm_sec').value)
        t0 = self._mono()
        while self._mono() - t0 < timeout and rclpy.ok():
            if self._map is not None and self._scan_fresh():
                return
            time.sleep(0.05)

    def _amcl_tuple(self) -> Optional[Tuple[float, float, float]]:
        if self._amcl is None:
            return None
        p = self._amcl.pose.pose
        x = float(p.position.x)
        y = float(p.position.y)
        yaw = self._yaw_from_quat(p.orientation)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            return None
        return (x, y, yaw)

    def _pose_in_map(self, pose: Tuple[float, float, float]) -> bool:
        if not self._ensure_field() or self._field is None:
            return False
        ix, iy = self._field.world_to_ixy(pose[0], pose[1])
        return 0 <= ix < self._field.width and 0 <= iy < self._field.height

    def _coverage_ok(self, laser: Dict[str, Any]) -> bool:
        beams = int(laser.get('valid_beams') or 0)
        reason = str(laser.get('reason') or '')
        return (
            bool(laser.get('ok'))
            and float(laser.get('laser_score') or 0.0) >= self._laser_floor()
            and beams >= int(self.get_parameter('min_valid_beams').value)
            and reason != 'few_beams'
        )

    def _r1_skip_reason(self, pose: Optional[Tuple[float, float, float]], age: Optional[float]) -> Optional[str]:
        """Skip R1 when the current pose cannot be a reliable proposal.

        status==3 alone is not out-of-map. Positive LOST is induced that way
        while the geometric pose is still a laser proposal.
        """
        if pose is None:
            return 'no_amcl_pose'
        if not all(math.isfinite(v) for v in pose):
            return 'pose_not_finite'
        # AMCL often does not republish while stopped. A latched pose is still
        # a laser proposal; the 0.38 gate is the protection. Do not skip it
        # just because the stamp is old.
        frame = ''
        if self._amcl is not None:
            frame = (self._amcl.header.frame_id or '').strip()
        if frame and frame != 'map':
            return 'map_frame_mismatch'
        if self._map is None:
            return 'map_unavailable'
        if not self._pose_in_map(pose):
            return 'pose_out_of_map'
        if not self._scan_fresh():
            return 'scan_not_fresh'
        return None

    def _stage_row(self, code: str, **kwargs: Any) -> Dict[str, Any]:
        row = {'code': code, 't_mono': self._mono(), **kwargs}
        self.get_logger().info(f'LOST cascade {code}: {json.dumps(kwargs, default=str)[:280]}')
        return row

    def _publish_seed(self, pose: Tuple[float, float, float], source: str) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        q = Quaternion()
        q.z = math.sin(float(pose[2]) * 0.5)
        q.w = math.cos(float(pose[2]) * 0.5)
        msg.pose.pose.orientation = q
        cov_xy = float(self.get_parameter('r_seed_cov_xy').value)
        cov_yaw = float(self.get_parameter('r_seed_cov_yaw').value)
        msg.pose.covariance[0] = cov_xy
        msg.pose.covariance[7] = cov_xy
        msg.pose.covariance[35] = cov_yaw
        self._initialpose_pub.publish(msg)
        self.get_logger().info(
            f'seed /initialpose source={source} ({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f})'
        )

    def _wait_post_seed(
        self, candidate: Tuple[float, float, float], amcl_mono_before: Optional[float]
    ) -> Tuple[bool, Dict[str, Any]]:
        """POST_SEED_AMCL_READY. Latch-clear / status0 alone is not READY."""
        settle = max(float(self.get_parameter('post_seed_settle_sec').value), 5.0)
        timeout = min(float(self.get_parameter('amcl_timeout_sec').value), settle)
        timeout = max(timeout, 5.0)
        need = float(self.get_parameter('ready_stable_sec').value)
        cov_xy_lim = float(self.get_parameter('ready_cov_xy').value)
        cov_yaw_lim = float(self.get_parameter('ready_cov_yaw').value)
        t0 = self._mono()
        stable_since: Optional[float] = None
        last: Dict[str, Any] = {'phase': 'POST_SEED_AMCL_READY'}
        nomotion = 0
        far_latched_rejected = 0
        while self._mono() - t0 < timeout and rclpy.ok():
            if self._nomotion.service_is_ready() and (self._mono() - t0) >= 0.5 * nomotion:
                try:
                    self._nomotion.call_async(Empty.Request())
                    nomotion += 1
                except Exception:  # noqa: BLE001
                    pass
            time.sleep(0.1)
            if self._amcl is None or self._amcl_mono is None:
                continue
            latched_only = amcl_mono_before is not None and self._amcl_mono <= amcl_mono_before
            c = self._amcl.pose.covariance
            cov_xy = max(float(c[0]), float(c[7]))
            cov_yaw = float(c[35])
            pose = self._amcl_tuple()
            if pose is None:
                continue
            dx = math.hypot(pose[0] - candidate[0], pose[1] - candidate[1])
            dyaw = abs(math.atan2(math.sin(pose[2] - candidate[2]), math.cos(pose[2] - candidate[2])))
            near = dx <= 1.0 and dyaw <= 0.60
            if not latched_pose_may_satisfy_ready(latched_only, near):
                far_latched_rejected += 1
                last = {
                    'phase': 'POST_SEED_AMCL_READY',
                    'near_candidate': False,
                    'latched_pose_used': True,
                    'far_latched_rejected': far_latched_rejected,
                    'ready': False,
                }
                continue
            tf_ok = self._tf_ok('map', 'odom') and self._tf_ok('map', 'base_link')
            cov_ok = cov_xy <= cov_xy_lim and cov_yaw <= cov_yaw_lim
            status_ok = self._loc_status == 0
            gates = tf_ok and cov_ok and status_ok and near
            if gates:
                if stable_since is None:
                    stable_since = self._mono()
                stable_for = self._mono() - stable_since
            else:
                stable_since = None
                stable_for = 0.0
            last = {
                'phase': 'POST_SEED_AMCL_READY',
                'cov_xy': cov_xy,
                'cov_yaw': cov_yaw,
                'tf_ok': tf_ok,
                'loc_status': self._loc_status,
                'near_candidate': near,
                'latched_pose_used': latched_only,
                'far_latched_rejected': far_latched_rejected,
                'stable_for_sec': stable_for,
                'gates_ok': gates,
                'amcl_convergence_sec': self._mono() - t0,
            }
            if gates and stable_for >= need:
                last['ready'] = True
                return True, last
        last['ready'] = False
        last['handoff'] = 'HANDOFF_FAILED'
        last['amcl_convergence_sec'] = self._mono() - t0
        return False, last

    def _seed_and_handoff(
        self, path: str, pose: Tuple[float, float, float], laser: Dict[str, Any]
    ) -> Tuple[bool, Dict[str, Any]]:
        if self._seeds_this_session:
            self._sequential_fallback_seed_count += 1
        self._seeds_this_session += 1
        before = self._amcl_mono
        self._publish_seed(pose, path)
        ready, amcl_diag = self._wait_post_seed(pose, before)
        amcl_diag['laser_score'] = laser.get('laser_score')
        amcl_diag['seed_count'] = self._seeds_this_session
        return ready, amcl_diag

    def _try_r1(self, snapped: Optional[Tuple[float, float, float]], age: Optional[float]) -> Dict[str, Any]:
        skip = self._r1_skip_reason(snapped, age)
        if skip:
            return self._stage_row(
                'R1_CURRENT_SKIP',
                stage='R1',
                R1_SKIP_REASON=skip,
                pose=None if snapped is None else {'x': snapped[0], 'y': snapped[1], 'yaw': snapped[2]},
                seeded=False,
            )
        assert snapped is not None
        laser = verify_prior_in_window(
            snapped,
            self._scan,
            self._map,
            min_score=self._laser_floor(),
            field=self._field,
        )
        coverage = self._coverage_ok(laser)
        pose_d = laser.get('pose') or {'x': snapped[0], 'y': snapped[1], 'yaw': snapped[2]}
        proposal = (float(pose_d['x']), float(pose_d['y']), float(pose_d['yaw']))
        if not coverage:
            return self._stage_row(
                'R1_CURRENT_REJECT',
                stage='R1',
                reason=laser.get('reason'),
                laser_score=laser.get('laser_score'),
                valid_beams=laser.get('valid_beams'),
                coverage_ok=False,
                seeded=False,
                pose={'x': proposal[0], 'y': proposal[1], 'yaw': proposal[2]},
            )
        ready, amcl_diag = self._seed_and_handoff('R1', proposal, laser)
        return self._stage_row(
            'R1_CURRENT_ACCEPT',
            stage='R1',
            reason='laser_accept' if ready else 'handoff_failed',
            laser_score=laser.get('laser_score'),
            valid_beams=laser.get('valid_beams'),
            coverage_ok=True,
            seeded=True,
            handoff='READY' if ready else 'HANDOFF_FAILED',
            pose={'x': proposal[0], 'y': proposal[1], 'yaw': proposal[2]},
            amcl=amcl_diag,
            ready=ready,
        )

    def _try_r2(self) -> Dict[str, Any]:
        maps_dir = str(self.get_parameter('maps_dir').value)
        map_name = self._map_name or 'vp'
        v = validate_as_proposal(
            maps_dir,
            map_name,
            max_age_sec=float(self.get_parameter('last_good_max_age_sec').value),
            min_quality=float(self.get_parameter('last_good_min_quality').value),
        )
        if not v.ok or v.pose is None:
            # File/identity/age failures are skips. Do not laser a pose that
            # is not an eligible proposal.
            return self._stage_row(
                'R2_LAST_GOOD_SKIP',
                stage='R2',
                reason=v.reason or 'invalid_proposal',
                seeded=False,
            )
        if not bool(v.pose.laser_verified):
            return self._stage_row(
                'R2_LAST_GOOD_SKIP',
                stage='R2',
                reason='laser_verified_false',
                seeded=False,
            )
        if self._scan is None or self._map is None or not self._scan_fresh() or not self._ensure_field():
            return self._stage_row(
                'R2_LAST_GOOD_SKIP',
                stage='R2',
                reason='scan_or_map_unavailable',
                seeded=False,
            )
        proposal = (float(v.pose.x), float(v.pose.y), float(v.pose.yaw))
        # Write-time laser_verified cannot replace this recovery-time gate.
        laser = verify_pose_with_laser(
            proposal,
            self._scan,
            self._map,
            min_score=self._laser_floor(),
            min_valid_beams=int(self.get_parameter('min_valid_beams').value),
            field=self._field,
        )
        coverage = self._coverage_ok(laser)
        if not coverage:
            return self._stage_row(
                'R2_LAST_GOOD_REJECT',
                stage='R2',
                reason=laser.get('reason') or 'laser_gate',
                laser_score=laser.get('laser_score'),
                valid_beams=laser.get('valid_beams'),
                coverage_ok=False,
                seeded=False,
                pose={'x': proposal[0], 'y': proposal[1], 'yaw': proposal[2]},
                write_laser=float(v.pose.laser_score_at_write),
            )
        ready, amcl_diag = self._seed_and_handoff('R2', proposal, laser)
        return self._stage_row(
            'R2_LAST_GOOD_ACCEPT',
            stage='R2',
            reason='laser_accept' if ready else 'handoff_failed',
            laser_score=laser.get('laser_score'),
            valid_beams=laser.get('valid_beams'),
            coverage_ok=True,
            seeded=True,
            handoff='READY' if ready else 'HANDOFF_FAILED',
            pose={'x': proposal[0], 'y': proposal[1], 'yaw': proposal[2]},
            amcl=amcl_diag,
            ready=ready,
            write_laser=float(v.pose.laser_score_at_write),
        )

    def _try_r3(self) -> Dict[str, Any]:
        # Existing Visual+Laser only. One candidate seed for this stage.
        out = self._call_reloc()
        code = int(out.get('result_code', -1))
        if code == _RC_READY or (out.get('ok') and code == _RC_READY):
            if self._seeds_this_session:
                self._sequential_fallback_seed_count += 1
            self._seeds_this_session += 1
            cand = self._amcl_tuple()
            ready, amcl_diag = self._amcl_ready_stable()
            if cand is not None:
                amcl_diag['near_candidate'] = True
            return self._stage_row(
                'R3_VISUAL_ACCEPT',
                stage='R3',
                laser_score=out.get('laser_score'),
                seeded=True,
                handoff='READY' if ready else 'HANDOFF_FAILED',
                reloc=out,
                amcl=amcl_diag,
                ready=ready,
            )
        return self._stage_row(
            'R3_VISUAL_UNKNOWN',
            stage='R3',
            reason=out.get('error') or 'no_survivor',
            laser_score=out.get('laser_score'),
            result_code=code,
            seeded=False,
            reloc=out,
            ready=False,
        )

    def _tf_ok(self, parent: str, child: str) -> bool:
        if self._tf is None:
            return False
        try:
            if not self._tf.can_transform(parent, child, rclpy.time.Time()):
                return False
            tf = self._tf.lookup_transform(parent, child, rclpy.time.Time())
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
        # Do not fight BOOT cascade for /initialpose ownership.
        if self._boot_busy or self._logical == Phase2CLocState.BOOT_LOCALIZING:
            return None
        # BOOT failure latch is closed by operator pose, not a Reloc storm.
        if self._logical in (
            Phase2CLocState.NEED_OPERATOR,
            Phase2CLocState.VERIFYING_OPERATOR_POSE,
        ):
            return None
        if self._boot_state in (
            'SENSOR_TIMEOUT',
            'UNKNOWN',
            'VERIFYING_OPERATOR_POSE',
            'PRE_LOCALIZATION_READY',
            'TRY_CHARGER',
            'TRY_LAST_GOOD',
            'TRY_VISUAL_LASER',
            'POST_SEED_AMCL_READY',
            'WAIT_AMCL_SETTLE',
        ):
            return None
        if self._owner.remote_blocks():
            return None
        if self._mono() < self._unknown_until:
            return None
        if self._mono() < self._ready_guard_until:
            return None
        # Only while nav capability / follow / recharge / navigating
        if not (self._nav_en or self._follow_en or self._recharge_en or self._mode in (2, 3)):
            self._status2_since = None
            self._status3_since = None
            self._jump_since = None
            self._jump_flag = False
            return None

        now = self._mono()
        # IDLE path: health/status events only. TF is checked inside active recovery.
        # No SUSPECT state machine: sustained anomaly past debounce → LOST.
        # Brief AMCL pullback that clears status before debounce must NOT start R1–R3.

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

        if self._jump_flag:
            if self._jump_since is None:
                self._jump_since = now
            elif now - self._jump_since >= float(self.get_parameter('pose_jump_debounce_sec').value):
                self._jump_flag = False
                self._jump_since = None
                return 'pose_jump'
            return None
        self._jump_since = None

        if self._loc_status == 0:
            self._jump_flag = False
            self._jump_since = None
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
            if self._busy or self._boot_busy:
                return
            self._session_id += 1
            sid = self._session_id
            self._busy = True
        if not self._owner.begin(sid):
            with self._lock:
                self._busy = False
            self.get_logger().warn(f'LOST ownership claim failed reason={reason}')
            return
        self._owner_pub.publish(String(data=self._owner.claim_json(reason)))
        threading.Thread(target=self._recovery_worker, args=(reason, sid), daemon=True).start()

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
        self._emit_canonical(Phase2CLocState.LOST.value, True, 'lost_stop')
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
        self._arm_tf()
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
        self._emit_canonical(Phase2CLocState.READY.value, False, 'lost_ready')
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

    def _recovery_worker(self, reason: str, session_id: int) -> None:
        t0 = self._mono()
        final = 'UNKNOWN'
        selected = ''
        snapped = self._amcl_tuple()
        pose_age = None if self._amcl_mono is None else (self._mono() - self._amcl_mono)
        self._seeds_this_session = 0
        self._sequential_fallback_seed_count = 0
        self._set_logical(Phase2CLocState.LOST)
        snap = self._stop_motion_first(reason)
        self._set_logical(Phase2CLocState.RECOVERING)
        stages: list = []
        timings: Dict[str, Any] = {'lost_detect_mono': t0}
        try:
            self._heal_gate_pub.publish(Bool(data=False))
            self._wait_sensors()
            timings['sensors_ready_mono'] = self._mono()

            # Sequential only. A later stage seeds only after the previous
            # stage did not ACCEPT, or laser-ACCEPTed and handoff failed.
            r1 = self._try_r1(snapped, pose_age)
            stages.append(r1)
            timings['r1_done_mono'] = self._mono()
            if r1.get('ready'):
                final = 'READY'
                selected = 'R1'
            else:
                if r1.get('code') == 'R1_CURRENT_ACCEPT' and r1.get('handoff') == 'HANDOFF_FAILED':
                    self.get_logger().warn('R1 laser ACCEPT but AMCL handoff failed; fallback R2')
                r2 = self._try_r2()
                stages.append(r2)
                timings['r2_done_mono'] = self._mono()
                if r2.get('ready'):
                    final = 'READY'
                    selected = 'R2'
                else:
                    if r2.get('code') == 'R2_LAST_GOOD_ACCEPT' and r2.get('handoff') == 'HANDOFF_FAILED':
                        self.get_logger().warn('R2 laser ACCEPT but AMCL handoff failed; fallback R3')
                    self._heal_gate_pub.publish(Bool(data=False))
                    r3 = self._try_r3()
                    stages.append(r3)
                    timings['r3_done_mono'] = self._mono()
                    if r3.get('ready'):
                        final = 'READY'
                        selected = 'R3'

            resume_policy = {'nav': 'blocked', 'follow': 'no_auto', 'recharge': 'blocked'}
            if final == 'READY':
                self._set_logical(Phase2CLocState.READY)
                resume_policy = self._resume_tasks(snap)
                timings['replan_mono'] = self._mono()
                self._ready_guard_until = self._mono() + float(
                    self.get_parameter('post_ready_guard_sec').value
                )
            else:
                self._set_logical(Phase2CLocState.NEED_OPERATOR)
                self._phase2c_rec_pub.publish(Bool(data=False))
                self._unknown_until = self._mono() + float(
                    self.get_parameter('unknown_cooldown_sec').value
                )
                resume_policy = {
                    'nav': 'forbidden',
                    'follow': 'forbidden',
                    'recharge': 'forbidden',
                    'detail': 'NEED_OPERATOR',
                }

            accept = next((s for s in stages if s.get('ready')), {})
            self._report = {
                'final': final,
                'reason': reason,
                'selected_recovery_path': selected,
                'r1_code': next((s.get('code') for s in stages if s.get('stage') == 'R1'), ''),
                'r2_code': next((s.get('code') for s in stages if s.get('stage') == 'R2'), ''),
                'r3_code': next((s.get('code') for s in stages if s.get('stage') == 'R3'), ''),
                'R1_SKIP_REASON': next(
                    (s.get('R1_SKIP_REASON') for s in stages if s.get('stage') == 'R1'), None
                ),
                'laser_score': accept.get('laser_score'),
                'seed_count': self._seeds_this_session,
                'sequential_fallback_seed_count': self._sequential_fallback_seed_count,
                'snapshot': json.loads(snap.to_json()),
                'stages': stages,
                'resume_policy': resume_policy,
                'timings': timings,
                'total_sec': self._mono() - t0,
                'session_id': session_id,
                'false_recovery_note': 'cascade R1→R2→R3; laser gate before READY; no spin+reinit',
            }
            self._result_pub.publish(String(data=json.dumps(self._report, default=str)))
            self.get_logger().info(
                f'LOST recovery done final={final} path={selected or "NONE"} '
                f't={self._report["total_sec"]:.1f}s resume={resume_policy}'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'lost recovery exception: {exc}')
            self._set_logical(Phase2CLocState.NEED_OPERATOR)
            self._phase2c_rec_pub.publish(Bool(data=False))
            self._unknown_until = self._mono() + float(
                self.get_parameter('unknown_cooldown_sec').value
            )
        finally:
            self._disarm_sensors()
            self._disarm_tf()
            if self._owner.holding():
                self._owner_pub.publish(String(data=self._owner.release_json(f'lost_{final}')))
                self._owner.end()
            with self._lock:
                self._busy = False
            self._publish_logical()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LostRecoveryNode()
    ex = MultiThreadedExecutor(num_threads=2)
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
