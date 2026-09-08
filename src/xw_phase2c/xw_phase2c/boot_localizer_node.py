#!/usr/bin/env python3
"""xw_boot_localizer — Phase2C-C2 BOOT cascade (dev / feature-flag only).

WAIT_SENSORS → TRY_CHARGER → TRY_LAST_GOOD → TRY_VISUAL_LASER → AMCL_VERIFY
→ READY | UNKNOWN

Serial only: this node (or Reloc during P3) is the sole /initialpose writer
while a cascade is active. Does not modify production bringup defaults.
"""

from __future__ import annotations

import json
import math
import threading
import time
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from xw_interfaces.msg import PowerState
from xw_interfaces.srv import Relocalize
from xw_phase2c.charger_prior import (
    evaluate_charger_soft_prior,
    load_charger_waypoint,
    verify_charger_with_laser,
)
from xw_phase2c.last_good_pose import validate_as_proposal
from xw_phase2c.laser_prior_verify import MIN_LASER_SCORE, verify_pose_with_laser
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

# Relocalize result codes (Relocalize.srv)
_RC_READY = 0
_RC_UNKNOWN = 1
_RC_AMCL_TIMEOUT = 3
_RC_NO_DATA = 4
_RC_DRY_RUN = 6


class BootState(str, Enum):
    IDLE = 'IDLE'
    WAIT_SENSORS = 'WAIT_SENSORS'
    TRY_CHARGER = 'TRY_CHARGER'
    TRY_LAST_GOOD = 'TRY_LAST_GOOD'
    TRY_VISUAL_LASER = 'TRY_VISUAL_LASER'
    AMCL_VERIFY = 'AMCL_VERIFY'
    READY = 'READY'
    UNKNOWN = 'UNKNOWN'
    SENSOR_TIMEOUT = 'SENSOR_TIMEOUT'


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BootLocalizerNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_boot_localizer')
        self._cb = ReentrantCallbackGroup()

        self.declare_parameter('maps_dir', '/ros2_ws/maps')
        self.declare_parameter('map_name', 'vp')
        self.declare_parameter('enabled', True)
        self.declare_parameter('auto_start', False)
        # Sensor gate (LiDAR default ~25s after bringup)
        self.declare_parameter('sensor_timeout_sec', 90.0)
        self.declare_parameter('scan_fresh_sec', 1.5)
        self.declare_parameter('tf_stale_sec', 1.5)
        self.declare_parameter('require_amcl_node', True)
        # Laser prior (frozen thr)
        self.declare_parameter('min_laser_score', MIN_LASER_SCORE)
        self.declare_parameter('p1_seed_cov_xy', 0.12)
        self.declare_parameter('p1_seed_cov_yaw', 0.08)
        self.declare_parameter('p2_seed_cov_xy', 0.25)
        self.declare_parameter('p2_seed_cov_yaw', 0.15)
        self.declare_parameter('last_good_max_age_sec', 7 * 24 * 3600.0)
        self.declare_parameter('last_good_min_quality', 0.35)
        # AMCL READY (Phase2B-aligned)
        self.declare_parameter('amcl_timeout_sec', 12.0)
        self.declare_parameter('amcl_ready_cov_xy', 0.80)
        self.declare_parameter('amcl_ready_cov_yaw', 0.35)
        self.declare_parameter('amcl_ready_stable_sec', 2.5)
        self.declare_parameter('amcl_pose_fresh_sec', 2.0)
        self.declare_parameter('amcl_stable_xy_m', 0.12)
        self.declare_parameter('amcl_stable_yaw_rad', 0.12)
        # P3
        self.declare_parameter('p3_cooldown_sec', 30.0)
        self.declare_parameter('p3_max_attempts', 2)
        self.declare_parameter('p3_service_timeout_sec', 60.0)

        self._state = BootState.IDLE
        self._lock = threading.Lock()
        self._busy = False
        self._stages: List[Dict[str, Any]] = []
        self._report: Dict[str, Any] = {}
        self._selected_path = ''

        self._scan: Optional[LaserScan] = None
        self._scan_mono: Optional[float] = None
        self._map: Optional[OccupancyGrid] = None
        self._field: Optional[DistanceField] = None
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._amcl_mono: Optional[float] = None
        self._loc_status = 1
        self._power = PowerState()
        self._battery_charging = False
        self._charger_prior_avail = False

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, _MAP_QOS)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, _AMCL_QOS)
        self.create_subscription(Int8, '/xw/localization_status', self._on_status, _LATCH)
        self.create_subscription(PowerState, '/xw/power', self._on_power, 10)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        self.create_subscription(Bool, '/xw/localization/charger_prior_available', self._on_prior, _LATCH)
        self.create_subscription(String, '/xw/nav/map_name', self._on_map_name, _LATCH)
        self.create_subscription(Bool, '/xw/boot/localize', self._on_trigger, 10)

        self._status_pub = self.create_publisher(String, '/xw/boot/status', _LATCH)
        self._result_pub = self.create_publisher(String, '/xw/boot/result', 10)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._phase2c_rec_pub = self.create_publisher(Bool, '/xw/localization/phase2c_recovery', _LATCH)

        self._nomotion = self.create_client(Empty, '/request_nomotion_update', callback_group=self._cb)
        self._reloc = self.create_client(Relocalize, '/xw/relocalize', callback_group=self._cb)
        self.create_service(Trigger, '/xw/boot/run', self._on_run_srv, callback_group=self._cb)

        if bool(self.get_parameter('auto_start').value):
            self.create_timer(2.0, self._auto_once, callback_group=self._cb)
        self._auto_fired = False

        self.get_logger().info(
            'boot_localizer ready (dev/flag). thr='
            f'{float(self.get_parameter("min_laser_score").value)} '
            'serial P1→P2→P3; no production bringup wiring'
        )
        self._publish_status()

    def _mono(self) -> float:
        return time.monotonic()

    def _map_name(self) -> str:
        return str(self.get_parameter('map_name').value or 'vp').strip() or 'vp'

    def _on_map_name(self, msg: String) -> None:
        name = (msg.data or '').strip()
        if name:
            self.set_parameters([rclpy.Parameter('map_name', rclpy.Parameter.Type.STRING, name)])

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_mono = self._mono()

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        try:
            self._field = DistanceField(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'DistanceField build failed: {exc}')
            self._field = None

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg
        self._amcl_mono = self._mono()

    def _on_status(self, msg: Int8) -> None:
        self._loc_status = int(msg.data)

    def _on_power(self, msg: PowerState) -> None:
        self._power = msg

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_charging = (
            msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        )

    def _on_prior(self, msg: Bool) -> None:
        self._charger_prior_avail = bool(msg.data)

    def _on_trigger(self, msg: Bool) -> None:
        if msg.data:
            self._start_async('topic')

    def _on_run_srv(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok = self._start_async('service')
        res.success = bool(ok)
        res.message = 'started' if ok else 'busy_or_disabled'
        return res

    def _auto_once(self) -> None:
        if self._auto_fired:
            return
        self._auto_fired = True
        self._start_async('auto_start')

    def _start_async(self, source: str) -> bool:
        if not bool(self.get_parameter('enabled').value):
            self.get_logger().warn('boot_localizer disabled')
            return False
        with self._lock:
            if self._busy:
                self.get_logger().warn('cascade already running')
                return False
            self._busy = True
        threading.Thread(target=self._run_cascade, args=(source,), daemon=True).start()
        return True

    def _set_state(self, st: BootState) -> None:
        self._state = st
        self._publish_status()

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'state': self._state.value,
                'map_name': self._map_name(),
                'selected_path': self._selected_path,
                'busy': self._busy,
            },
            separators=(',', ':'),
        )
        self._status_pub.publish(msg)

    def _stage(self, name: str, **kwargs: Any) -> Dict[str, Any]:
        row = {'stage': name, 't_mono': self._mono(), **kwargs}
        self._stages.append(row)
        self.get_logger().info(f'BOOT stage {name}: {json.dumps(kwargs, default=str)[:240]}')
        return row

    # --- sensors ---
    def _tf_ok(self, parent: str, child: str) -> bool:
        stale = float(self.get_parameter('tf_stale_sec').value)
        try:
            tf = self._tf.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
            )
            if tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0:
                return True
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            return age < stale
        except TransformException:
            return False

    def _sensors_ready(self) -> Tuple[bool, str]:
        if self._scan is None or self._scan_mono is None:
            return False, 'no_scan'
        age = self._mono() - self._scan_mono
        if age > float(self.get_parameter('scan_fresh_sec').value):
            return False, f'scan_stale_{age:.1f}s'
        if self._map is None or self._field is None:
            return False, 'no_map'
        if not self._tf_ok('map', 'odom'):
            return False, 'tf_map_odom'
        if not self._tf_ok('odom', 'base_link'):
            return False, 'tf_odom_base'
        if bool(self.get_parameter('require_amcl_node').value):
            # AMCL may not have pose yet before seed — require topic peer exists via any pose OR
            # accept missing pose but require service nomotion (AMCL up).
            if self._amcl is None and not self._nomotion.service_is_ready():
                return False, 'amcl_not_ready'
        return True, 'ok'

    def _wait_sensors(self) -> bool:
        self._set_state(BootState.WAIT_SENSORS)
        # Enter recovery profile for RGB if Reloc later needs it (harmless for P1/P2).
        self._phase2c_rec_pub.publish(Bool(data=True))
        timeout = float(self.get_parameter('sensor_timeout_sec').value)
        t0 = self._mono()
        last_reason = ''
        while self._mono() - t0 < timeout and rclpy.ok():
            ok, reason = self._sensors_ready()
            last_reason = reason
            if ok:
                self._stage(
                    'WAIT_SENSORS',
                    attempt=1,
                    result='ok',
                    reason=reason,
                    runtime_sec=self._mono() - t0,
                )
                return True
            time.sleep(0.2)
        self._stage(
            'WAIT_SENSORS',
            attempt=1,
            result='timeout',
            reason=last_reason,
            runtime_sec=self._mono() - t0,
        )
        self._set_state(BootState.SENSOR_TIMEOUT)
        return False

    # --- seed + AMCL READY ---
    def _publish_seed(
        self, x: float, y: float, yaw: float, cov_xy: float, cov_yaw: float, source: str
    ) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = _yaw_to_quat(float(yaw))
        msg.pose.covariance[0] = float(cov_xy)
        msg.pose.covariance[7] = float(cov_xy)
        msg.pose.covariance[35] = float(cov_yaw)
        self._initialpose_pub.publish(msg)
        self.get_logger().info(
            f'seed /initialpose source={source} ({x:.2f},{y:.2f},{yaw:.2f}) '
            f'cov=({cov_xy},{cov_yaw})'
        )

    def _amcl_tuple(self) -> Optional[Tuple[float, float, float]]:
        if self._amcl is None:
            return None
        p = self._amcl.pose.pose
        return (float(p.position.x), float(p.position.y), _yaw_from_quat(p.orientation))

    def _wait_amcl_ready(
        self, candidate: Tuple[float, float, float], amcl_mono_before: Optional[float]
    ) -> Tuple[bool, Dict[str, Any]]:
        """Phase2B READY rules — latch-3 clear alone is NOT READY."""
        timeout = float(self.get_parameter('amcl_timeout_sec').value)
        cov_xy_lim = float(self.get_parameter('amcl_ready_cov_xy').value)
        cov_yaw_lim = float(self.get_parameter('amcl_ready_cov_yaw').value)
        stable_need = float(self.get_parameter('amcl_ready_stable_sec').value)
        fresh_lim = float(self.get_parameter('amcl_pose_fresh_sec').value)
        jump_xy = float(self.get_parameter('amcl_stable_xy_m').value)
        jump_yaw = float(self.get_parameter('amcl_stable_yaw_rad').value)

        t0 = self._mono()
        deadline = t0 + timeout
        stable_since: Optional[float] = None
        anchor: Optional[Tuple[float, float, float]] = None
        last: Dict[str, Any] = {}
        nomotion = 0

        while self._mono() < deadline and rclpy.ok():
            if nomotion < 4 and self._nomotion.service_is_ready():
                if (self._mono() - t0) >= 0.35 * nomotion:
                    try:
                        self._nomotion.call_async(Empty.Request())
                        nomotion += 1
                    except Exception:  # noqa: BLE001
                        pass
            time.sleep(0.05)
            if self._amcl is None or self._amcl_mono is None:
                continue
            if amcl_mono_before is not None and self._amcl_mono <= amcl_mono_before:
                continue
            age = self._mono() - self._amcl_mono
            pose_fresh = age <= max(fresh_lim, 30.0)
            tf_ok = self._tf_ok('map', 'odom') and self._tf_ok('map', 'base_link')
            c = self._amcl.pose.covariance
            cov_xy = max(float(c[0]), float(c[7]))
            cov_yaw = float(c[35])
            pose = self._amcl_tuple()
            if pose is None:
                continue
            dx = math.hypot(pose[0] - candidate[0], pose[1] - candidate[1])
            dyaw = abs(math.atan2(math.sin(pose[2] - candidate[2]), math.cos(pose[2] - candidate[2])))
            cov_ok = cov_xy <= cov_xy_lim and cov_yaw <= cov_yaw_lim
            near = dx <= 1.0 and dyaw <= 0.60
            status_ok = self._loc_status == 0
            gates = pose_fresh and tf_ok and cov_ok and near and (
                status_ok or (cov_xy < 0.5 and cov_yaw < 0.25)
            )
            if gates:
                if anchor is None:
                    anchor = pose
                    stable_since = self._mono()
                else:
                    dxy = math.hypot(pose[0] - anchor[0], pose[1] - anchor[1])
                    dy = abs(
                        math.atan2(math.sin(pose[2] - anchor[2]), math.cos(pose[2] - anchor[2]))
                    )
                    if dxy > jump_xy or dy > jump_yaw:
                        anchor = pose
                        stable_since = self._mono()
                stable_for = (self._mono() - stable_since) if stable_since else 0.0
            else:
                stable_since = None
                anchor = None
                stable_for = 0.0
            last = {
                'amcl_pose': pose,
                'cov_xy': cov_xy,
                'cov_yaw': cov_yaw,
                'loc_status': self._loc_status,
                'tf_ok': tf_ok,
                'near_candidate': near,
                'stable_for_sec': stable_for,
                'gates_ok': gates,
                'amcl_convergence_sec': self._mono() - t0,
            }
            if gates and stable_for >= stable_need:
                last['ready'] = True
                return True, last
        last['ready'] = False
        last['amcl_convergence_sec'] = self._mono() - t0
        return False, last

    def _try_prior_path(
        self,
        path: str,
        pose: Tuple[float, float, float],
        cov_xy: float,
        cov_yaw: float,
        laser_diag: Dict[str, Any],
    ) -> bool:
        self._set_state(BootState.AMCL_VERIFY)
        before = self._amcl_mono
        self._publish_seed(pose[0], pose[1], pose[2], cov_xy, cov_yaw, path)
        if self._nomotion.service_is_ready():
            try:
                self._nomotion.call_async(Empty.Request())
            except Exception:  # noqa: BLE001
                pass
        ready, amcl_diag = self._wait_amcl_ready(pose, before)
        self._stage(
            path,
            attempt=1,
            result='READY' if ready else 'AMCL_FAIL',
            laser_score=laser_diag.get('laser_score'),
            reason=laser_diag.get('reason'),
            score=laser_diag.get('laser_score'),
            runtime_sec=laser_diag.get('runtime_sec'),
            amcl=amcl_diag,
            candidate_pose={'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
            amcl_pose=amcl_diag.get('amcl_pose'),
        )
        if ready:
            self._selected_path = path
            self._set_state(BootState.READY)
        return ready

    # --- P1 / P2 / P3 ---
    def _eval_prior_available(self) -> bool:
        if self._charger_prior_avail:
            return True
        r = evaluate_charger_soft_prior(
            charging=bool(self._power.charging),
            docked=bool(self._power.docked),
            battery_charging=bool(self._battery_charging),
            maps_dir=str(self.get_parameter('maps_dir').value),
            map_name=self._map_name(),
        )
        return bool(r.charger_prior_available)

    def _try_p1(self) -> bool:
        self._set_state(BootState.TRY_CHARGER)
        t0 = self._mono()
        if not self._eval_prior_available():
            self._stage(
                'P1',
                attempt=1,
                result='skip',
                reason='charger_prior_unavailable',
                runtime_sec=self._mono() - t0,
            )
            return False
        wp = load_charger_waypoint(str(self.get_parameter('maps_dir').value), self._map_name())
        if wp is None or self._scan is None or self._map is None:
            self._stage('P1', attempt=1, result='fail', reason='missing_wp_or_sensors')
            return False
        # Soft prior + laser — never trust charging alone.
        laser = verify_charger_with_laser(
            wp,
            self._scan,
            self._map,
            min_score=float(self.get_parameter('min_laser_score').value),
        )
        if not laser.get('ok'):
            self._stage(
                'P1',
                attempt=1,
                result='laser_reject',
                reason=laser.get('reason') or laser.get('status'),
                score=laser.get('laser_score'),
                runtime_sec=laser.get('runtime_sec') or (self._mono() - t0),
            )
            return False
        verified = laser.get('pose') or {}
        seed = (
            float(verified.get('x', wp[0])),
            float(verified.get('y', wp[1])),
            float(verified.get('yaw', wp[2])),
        )
        return self._try_prior_path(
            'P1',
            seed,
            float(self.get_parameter('p1_seed_cov_xy').value),
            float(self.get_parameter('p1_seed_cov_yaw').value),
            laser,
        )

    def _try_p2(self) -> bool:
        self._set_state(BootState.TRY_LAST_GOOD)
        t0 = self._mono()
        maps_dir = str(self.get_parameter('maps_dir').value)
        v = validate_as_proposal(
            maps_dir,
            self._map_name(),
            max_age_sec=float(self.get_parameter('last_good_max_age_sec').value),
            min_quality=float(self.get_parameter('last_good_min_quality').value),
        )
        if not v.ok or v.pose is None:
            self._stage(
                'P2',
                attempt=1,
                result='skip' if v.reason.startswith('missing') else 'reject',
                reason=v.reason,
                runtime_sec=self._mono() - t0,
            )
            return False
        pose = (v.pose.x, v.pose.y, v.pose.yaw)
        if self._scan is None or self._map is None:
            self._stage('P2', attempt=1, result='fail', reason='missing_sensors')
            return False
        laser = verify_pose_with_laser(
            pose,
            self._scan,
            self._map,
            min_score=float(self.get_parameter('min_laser_score').value),
            field=self._field,
        )
        if not laser.get('ok'):
            self._stage(
                'P2',
                attempt=1,
                result='laser_reject',
                reason=laser.get('reason'),
                score=laser.get('laser_score'),
                runtime_sec=laser.get('runtime_sec'),
            )
            return False
        return self._try_prior_path(
            'P2',
            pose,
            float(self.get_parameter('p2_seed_cov_xy').value),
            float(self.get_parameter('p2_seed_cov_yaw').value),
            laser,
        )

    def _call_relocalize(self) -> Dict[str, Any]:
        if not self._reloc.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'error': 'relocalize_unavailable', 'result_code': -1}
        req = Relocalize.Request()
        req.map_name = self._map_name()
        req.force_visual = True
        req.max_candidates = 10
        req.apply_initial_pose = True
        req.allow_motion = False
        fut = self._reloc.call_async(req)
        timeout = float(self.get_parameter('p3_service_timeout_sec').value)
        t0 = self._mono()
        while self._mono() - t0 < timeout and rclpy.ok() and not fut.done():
            time.sleep(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'error': 'relocalize_timeout', 'result_code': -1}
        res = fut.result()
        pose = None
        if res.pose is not None:
            p = res.pose.pose.pose
            pose = (float(p.position.x), float(p.position.y), _yaw_from_quat(p.orientation))
        return {
            'ok': bool(res.success),
            'result_code': int(res.result_code),
            'laser_score': float(res.laser_score),
            'visual_score': float(res.visual_score),
            'time_to_candidate_sec': float(res.time_to_candidate_sec),
            'amcl_convergence_sec': float(res.amcl_convergence_sec),
            'diagnostics_json': res.diagnostics_json,
            'candidate_pose': pose,
        }

    def _try_p3(self) -> bool:
        self._set_state(BootState.TRY_VISUAL_LASER)
        max_attempts = int(self.get_parameter('p3_max_attempts').value)
        cooldown = float(self.get_parameter('p3_cooldown_sec').value)
        for attempt in range(1, max_attempts + 1):
            t0 = self._mono()
            out = self._call_relocalize()
            code = int(out.get('result_code', -1))
            runtime = self._mono() - t0
            if code == _RC_READY or (out.get('ok') and code == _RC_READY):
                self._stage(
                    'P3',
                    attempt=attempt,
                    result='READY',
                    score=out.get('laser_score'),
                    reason='relocalize_ready',
                    runtime_sec=runtime,
                    reloc=out,
                    candidate_pose=out.get('candidate_pose'),
                )
                self._selected_path = 'P3'
                self._set_state(BootState.READY)
                return True
            reason = {
                _RC_UNKNOWN: 'UNKNOWN',
                _RC_AMCL_TIMEOUT: 'AMCL_TIMEOUT',
                _RC_NO_DATA: 'NO_DATA',
                _RC_DRY_RUN: 'DRY_RUN_UNEXPECTED',
            }.get(code, out.get('error') or f'code_{code}')
            self._stage(
                'P3',
                attempt=attempt,
                result='fail',
                reason=reason,
                score=out.get('laser_score'),
                runtime_sec=runtime,
                reloc=out,
            )
            if attempt < max_attempts:
                self.get_logger().warn(f'P3 fail ({reason}); cooldown {cooldown:.0f}s')
                time.sleep(cooldown)
        return False

    def _finish(self, final: str, t0: float) -> None:
        self._phase2c_rec_pub.publish(Bool(data=False))
        amcl = self._amcl_tuple()
        self._report = {
            'final': final,
            'selected_path': self._selected_path or None,
            'stages': self._stages,
            'total_boot_localization_sec': self._mono() - t0,
            'amcl_pose': amcl,
            'map_name': self._map_name(),
            'note': 'READY requires AMCL stable window; never latch-clear alone',
            'false_accept_policy': 'UNKNOWN allowed; FA must be 0',
        }
        self._result_pub.publish(String(data=json.dumps(self._report, default=str)))
        self.get_logger().info(
            f'BOOT done final={final} path={self._selected_path} '
            f't={self._report["total_boot_localization_sec"]:.1f}s'
        )
        self._publish_status()

    def _run_cascade(self, source: str) -> None:
        t0 = self._mono()
        self._stages = []
        self._selected_path = ''
        self._report = {}
        try:
            self._stage('START', attempt=1, result='ok', reason=source)
            if not self._wait_sensors():
                self._finish('SENSOR_TIMEOUT', t0)
                return
            if self._try_p1():
                self._finish('READY', t0)
                return
            if self._try_p2():
                self._finish('READY', t0)
                return
            if self._try_p3():
                self._finish('READY', t0)
                return
            self._set_state(BootState.UNKNOWN)
            self._selected_path = self._selected_path or 'NONE'
            self._finish('UNKNOWN', t0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cascade exception: {exc}')
            self._set_state(BootState.UNKNOWN)
            self._stage('ERROR', attempt=1, result='exception', reason=str(exc))
            self._finish('UNKNOWN', t0)
        finally:
            with self._lock:
                self._busy = False
            self._publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BootLocalizerNode()
    try:
        from rclpy.executors import MultiThreadedExecutor

        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
