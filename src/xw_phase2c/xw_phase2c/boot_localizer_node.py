#!/usr/bin/env python3
"""xw_boot_localizer — Phase2C BOOT cascade (P1→P2→P3).

PRE_LOCALIZATION_READY (no map→odom) → P1 → P2 → P3
→ /initialpose → POST_SEED_AMCL_READY → READY | NEED_OPERATOR

map→odom / map→base_link are results of a global seed, not preconditions.
IDLE waits for NAV/trigger/operator events — no TF listener, no scan spin.

Production master switch: phase2c_localization_enabled.
When true, NAV session start / map switch auto-runs BOOT and this node (or
Reloc during P3) is the sole automatic /initialpose writer.
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
from xw_phase2c.canonical_state import (
    CANONICAL_TOPIC,
    SUBMIT_TOPIC,
    is_newer,
    is_open_incident,
    latched_pose_may_satisfy_ready,
    make_event,
    parse_event,
)
from xw_phase2c.charger_prior import (
    evaluate_charger_soft_prior,
    load_charger_waypoint,
    verify_charger_with_laser,
)
from xw_phase2c.last_good_pose import validate_as_proposal
from xw_phase2c.laser_prior_verify import MIN_LASER_SCORE, verify_pose_with_laser
from xw_phase2c.ownership import (
    OWNER_TOPIC,
    InitialPoseOwner,
    OwnershipGuard,
    owner_payload,
    parse_owner,
)
from xw_phase2c.recovery_state import Phase2CLocState
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
    # PRE: sensors that must exist *before* a global seed. Never map→odom.
    PRE_LOCALIZATION_READY = 'PRE_LOCALIZATION_READY'
    WAIT_SENSORS = 'PRE_LOCALIZATION_READY'  # alias kept for older readers
    TRY_CHARGER = 'TRY_CHARGER'
    TRY_LAST_GOOD = 'TRY_LAST_GOOD'
    TRY_VISUAL_LASER = 'TRY_VISUAL_LASER'
    POST_SEED_AMCL_READY = 'POST_SEED_AMCL_READY'
    AMCL_VERIFY = 'POST_SEED_AMCL_READY'  # alias
    # Laser ACCEPT is not READY. Wait a bounded settle for TF/status/AMCL.
    WAIT_AMCL_SETTLE = 'WAIT_AMCL_SETTLE'
    VERIFYING_OPERATOR_POSE = 'VERIFYING_OPERATOR_POSE'
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
        # Production master switch (C4B). When true: auto BOOT on NAV session.
        self.declare_parameter('phase2c_localization_enabled', False)
        self.declare_parameter('enabled', True)
        self.declare_parameter('auto_start', False)
        # Sensor gate (LiDAR default ~25s after bringup) — no fixed sleep guess.
        self.declare_parameter('sensor_timeout_sec', 90.0)
        self.declare_parameter('scan_fresh_sec', 1.5)
        self.declare_parameter('tf_stale_sec', 1.5)
        self.declare_parameter('require_amcl_node', True)
        self.declare_parameter('laser_frame', 'lidar_link')
        self.declare_parameter('rgb_wait_sec', 8.0)
        self.declare_parameter('operator_owner_fresh_sec', 5.0)
        # Production: ignore DEV-only latch inject on charger_prior_available.
        self.declare_parameter('accept_external_prior_inject', False)
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
        # After a laser ACCEPT, wait this long for TF/status before handoff fail.
        self.declare_parameter('p2_settle_sec', 8.0)
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
        self._cancel = threading.Event()
        self._session_id = 0
        self._pending_boot: Optional[Tuple[str, int]] = None
        self._stages: List[Dict[str, Any]] = []
        self._report: Dict[str, Any] = {}
        self._selected_path = ''
        self._nav_en = False
        self._phase2c_loc = Phase2CLocState.READY.value
        self._owner = OwnershipGuard(InitialPoseOwner.BOOT)

        self._scan: Optional[LaserScan] = None
        self._scan_mono: Optional[float] = None
        self._map: Optional[OccupancyGrid] = None
        self._map_mono: Optional[float] = None
        self._field: Optional[DistanceField] = None
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._amcl_mono: Optional[float] = None
        self._loc_status = 1
        self._power = PowerState()
        self._battery_charging = False
        self._charger_prior_avail = False
        self._goals_latched = False
        self._op_verify_busy = False
        self._self_ip_suppress_until = 0.0
        self._owner_raw = ''
        self._owner_mono = 0.0
        self._milestones: Dict[str, Any] = {}
        self._canonical_gen = 0
        self._canonical_state = Phase2CLocState.READY.value
        self._seeds_this_session = 0
        self._sequential_fallback_seed_count = 0
        self._laser_tf_ok = False
        self._amcl_if_ok = False
        self._amcl_if_mono = 0.0

        # IDLE: no TransformListener, no /scan, no amcl_pose. Armed only for a session.
        self._tf: Optional[Buffer] = None
        self._tf_listener = None
        self._scan_sub = None
        self._amcl_sub = None

        self.create_subscription(OccupancyGrid, '/map', self._on_map, _MAP_QOS)
        self.create_subscription(Int8, '/xw/localization_status', self._on_status, _LATCH)
        self.create_subscription(PowerState, '/xw/power', self._on_power, 10)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        self.create_subscription(Bool, '/xw/localization/charger_prior_available', self._on_prior, _LATCH)
        self.create_subscription(String, '/xw/nav/map_name', self._on_map_name, _LATCH)
        self.create_subscription(Bool, '/xw/boot/localize', self._on_trigger, 10)
        self.create_subscription(Bool, '/xw/nav/enable', self._on_nav_en, _LATCH)
        self.create_subscription(
            String, '/xw/localization/phase2c_loc_state', self._on_phase2c_loc, _LATCH
        )
        self.create_subscription(String, OWNER_TOPIC, self._on_owner, _LATCH)
        self.create_subscription(String, CANONICAL_TOPIC, self._on_canonical, _LATCH)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initialpose, 10
        )

        self._status_pub = self.create_publisher(String, '/xw/boot/status', _LATCH)
        self._result_pub = self.create_publisher(String, '/xw/boot/result', 10)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._phase2c_rec_pub = self.create_publisher(Bool, '/xw/localization/phase2c_recovery', _LATCH)
        # System truth is supervisor-only. Boot submits events, never latches
        # /xw/nav/goals_blocked or /xw/localization/phase2c_loc_state.
        self._canonical_pub = self.create_publisher(String, SUBMIT_TOPIC, _LATCH)
        self._owner_pub = self.create_publisher(String, OWNER_TOPIC, _LATCH)

        self._nomotion = self.create_client(Empty, '/request_nomotion_update', callback_group=self._cb)
        self._reloc = self.create_client(Relocalize, '/xw/relocalize', callback_group=self._cb)
        self._loc_active = self.create_client(
            Trigger, '/lifecycle_manager_localization/is_active', callback_group=self._cb
        )
        self._rgb_req = self.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
        self.create_service(Trigger, '/xw/boot/run', self._on_run_srv, callback_group=self._cb)

        if bool(self.get_parameter('auto_start').value):
            self.create_timer(2.0, self._auto_once, callback_group=self._cb)
        self._auto_fired = False

        self.get_logger().info(
            'boot_localizer ready master='
            f'{bool(self.get_parameter("phase2c_localization_enabled").value)} '
            f'thr={float(self.get_parameter("min_laser_score").value)} '
            'PRE then P1→P2→P3; IDLE event-driven (no TF/scan); '
            'POST_SEED_AMCL_READY before goals unblock'
        )
        self._publish_status()

    def _mono(self) -> float:
        return time.monotonic()

    def _map_name(self) -> str:
        return str(self.get_parameter('map_name').value or 'vp').strip() or 'vp'

    def _on_map_name(self, msg: String) -> None:
        name = (msg.data or '').strip()
        if not name:
            return
        prev = self._map_name()
        self.set_parameters([rclpy.Parameter('map_name', rclpy.Parameter.Type.STRING, name)])
        # Map switch while NAV active → new localization session (cancel + restart).
        if (
            self._master_on()
            and self._nav_en
            and name != prev
            and bool(self.get_parameter('enabled').value)
        ):
            self._request_session_boot('map_switch')

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_mono = self._mono()
        self._milestones.setdefault('first_scan_mono', self._scan_mono)
        self._milestones.setdefault('first_scan_wall', time.time())

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        self._map_mono = self._mono()
        self._milestones.setdefault('first_map_mono', self._map_mono)
        self._milestones.setdefault('first_map_wall', time.time())
        # Distance field is only needed for P1/P2 laser verify — build lazily.
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

    def _on_owner(self, msg: String) -> None:
        raw = msg.data or ''
        self._owner.on_remote(raw)
        self._owner_raw = raw
        self._owner_mono = self._mono()

    def _on_phase2c_loc(self, msg: String) -> None:
        # Compatibility mirror. Canonical events below are authoritative.
        incoming = (msg.data or 'READY').strip() or 'READY'
        if is_open_incident(self._canonical_state) and incoming == Phase2CLocState.READY.value:
            return
        self._phase2c_loc = incoming

    def _on_canonical(self, msg: String) -> None:
        event = parse_event(msg.data or '')
        if not event.get('state') or not is_newer(event, self._canonical_gen):
            return
        self._canonical_gen = int(event['generation'])
        self._canonical_state = str(event['state'])
        self._phase2c_loc = self._canonical_state
        if is_open_incident(self._canonical_state) and event.get('goals_blocked'):
            self._goals_latched = True
        elif self._canonical_state == Phase2CLocState.READY.value:
            self._goals_latched = False

    def _emit_canonical(self, state: str, goals_blocked: bool, source: str) -> None:
        self._canonical_gen = int(time.time_ns())
        self._canonical_state = str(state)
        self._phase2c_loc = self._canonical_state
        self._goals_latched = bool(goals_blocked)
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

    def _on_nav_en(self, msg: Bool) -> None:
        was = self._nav_en
        self._nav_en = bool(msg.data)
        if self._nav_en and not was and self._master_on():
            self._request_session_boot('nav_enable')
        elif was and not self._nav_en:
            # Leaving NAV: cancel in-flight BOOT. Do not clear an open incident.
            self._cancel.set()
            if not self._busy and not self._op_verify_busy:
                if not is_open_incident(self._canonical_state):
                    self._emit_canonical(Phase2CLocState.READY.value, False, 'boot_nav_off')
                if self._owner.holding():
                    self._owner_pub.publish(String(data=self._owner.release_json('nav_off')))
                    self._owner.end()
                self._disarm_session_io()

    def _on_trigger(self, msg: Bool) -> None:
        if msg.data:
            self._start_async('topic')

    def _on_run_srv(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        ok = self._start_async('service')
        res.success = bool(ok)
        res.message = 'started' if ok else 'busy_or_disabled'
        return res

    def _master_on(self) -> bool:
        return bool(self.get_parameter('phase2c_localization_enabled').value)

    def _request_session_boot(self, source: str) -> None:
        """Cancel any in-flight cascade and start a new session generation."""
        start_now = False
        with self._lock:
            self._session_id += 1
            sid = self._session_id
            if self._busy:
                self._pending_boot = (source, sid)
            else:
                self._pending_boot = None
                start_now = True
        self._cancel.set()
        if start_now:
            self._start_async(source, session_id=sid)

    def _auto_once(self) -> None:
        if self._auto_fired:
            return
        self._auto_fired = True
        self._start_async('auto_start')

    def _lost_busy(self) -> bool:
        return self._phase2c_loc in (
            Phase2CLocState.LOST.value,
            Phase2CLocState.RECOVERING.value,
        )

    def _start_async(self, source: str, session_id: Optional[int] = None) -> bool:
        if not bool(self.get_parameter('enabled').value):
            self.get_logger().warn('boot_localizer disabled')
            return False
        if self._lost_busy() or self._owner.remote_blocks():
            self.get_logger().warn(
                f'skip BOOT source={source}: LOST/owner busy loc={self._phase2c_loc}'
            )
            return False
        with self._lock:
            if self._busy:
                if session_id is not None:
                    self._pending_boot = (source, int(session_id))
                self.get_logger().warn('cascade running — queued newer session')
                return False
            if session_id is None:
                self._session_id += 1
                session_id = self._session_id
            elif session_id < self._session_id:
                return False
            self._busy = True
            self._pending_boot = None
            self._cancel.clear()
        if not self._owner.begin(int(session_id)):
            with self._lock:
                self._busy = False
            self.get_logger().warn('BOOT ownership claim failed')
            return False
        self._owner_pub.publish(String(data=self._owner.claim_json(source)))
        threading.Thread(
            target=self._run_cascade, args=(source, int(session_id)), daemon=True
        ).start()
        return True

    def _set_state(self, st: BootState) -> None:
        self._state = st
        if st in (
            BootState.PRE_LOCALIZATION_READY,
            BootState.TRY_CHARGER,
            BootState.TRY_LAST_GOOD,
            BootState.TRY_VISUAL_LASER,
            BootState.POST_SEED_AMCL_READY,
            BootState.WAIT_AMCL_SETTLE,
        ):
            self._emit_canonical(Phase2CLocState.BOOT_LOCALIZING.value, True, 'boot')
        elif st == BootState.VERIFYING_OPERATOR_POSE:
            self._emit_canonical(Phase2CLocState.VERIFYING_OPERATOR_POSE.value, True, 'boot')
        elif st == BootState.READY:
            self._emit_canonical(Phase2CLocState.READY.value, False, 'boot')
        elif st in (BootState.UNKNOWN, BootState.SENSOR_TIMEOUT):
            self._emit_canonical(Phase2CLocState.NEED_OPERATOR.value, True, 'boot')
        self._publish_status()

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'state': self._state.value,
                'map_name': self._map_name(),
                'selected_path': self._selected_path,
                'busy': self._busy,
                'session_id': self._session_id,
                'master': self._master_on(),
                'goals_latched': self._goals_latched,
                'milestones': self._milestones,
            },
            separators=(',', ':'),
        )
        self._status_pub.publish(msg)

    def _stage(self, name: str, **kwargs: Any) -> Dict[str, Any]:
        row = {'stage': name, 't_mono': self._mono(), **kwargs}
        self._stages.append(row)
        self.get_logger().info(f'BOOT stage {name}: {json.dumps(kwargs, default=str)[:240]}')
        return row

    def _cancelled(self, session_id: int) -> bool:
        if self._cancel.is_set():
            return True
        with self._lock:
            return session_id != self._session_id

    # --- sensors (armed only while a cascade / operator verify is active) ---
    def _arm_session_io(self) -> None:
        if self._scan_sub is None:
            self._scan_sub = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        if self._amcl_sub is None:
            self._amcl_sub = self.create_subscription(
                PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, _AMCL_QOS
            )
        if self._tf is None:
            self._tf = Buffer()
            self._tf_listener = TransformListener(self._tf, self, spin_thread=False)

    def _disarm_session_io(self) -> None:
        for attr in ('_scan_sub', '_amcl_sub'):
            sub = getattr(self, attr)
            if sub is not None:
                try:
                    self.destroy_subscription(sub)
                except Exception:  # noqa: BLE001
                    pass
                setattr(self, attr, None)
        listener = self._tf_listener
        self._tf_listener = None
        self._tf = None
        if listener is not None:
            try:
                listener.unregister()
            except Exception:  # noqa: BLE001
                pass

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

    def _tf_ok(self, parent: str, child: str) -> bool:
        if self._tf is None:
            return False
        stale = float(self.get_parameter('tf_stale_sec').value)
        try:
            if not self._tf.can_transform(parent, child, rclpy.time.Time()):
                return False
            tf = self._tf.lookup_transform(parent, child, rclpy.time.Time())
            if tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0:
                return True
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            return age < stale
        except TransformException:
            return False

    def _topic_has_publisher(self, topic: str) -> bool:
        try:
            return bool(self.get_publishers_info_by_topic(topic))
        except Exception:  # noqa: BLE001
            return False

    def _amcl_interfaces_ready(self) -> bool:
        """AMCL lifecycle / service ready. Does not require an /initialpose yet."""
        now = self._mono()
        if self._amcl_if_ok and (now - self._amcl_if_mono) < 2.0:
            return True
        if self._nomotion.service_is_ready():
            self._amcl_if_ok = True
            self._amcl_if_mono = now
            self._milestones.setdefault('amcl_lifecycle_active_mono', now)
            self._milestones.setdefault('amcl_lifecycle_active_wall', time.time())
            return True
        if self._loc_active.service_is_ready():
            try:
                fut = self._loc_active.call_async(Trigger.Request())
            except Exception:  # noqa: BLE001
                return False
            t0 = self._mono()
            while self._mono() - t0 < 0.4 and rclpy.ok() and not fut.done():
                time.sleep(0.02)
            if fut.done() and fut.result() is not None and bool(fut.result().success):
                self._amcl_if_ok = True
                self._amcl_if_mono = now
                self._milestones.setdefault('amcl_lifecycle_active_mono', now)
                self._milestones.setdefault('amcl_lifecycle_active_wall', time.time())
                return True
        return False

    def _pre_localization_ready(self) -> Tuple[bool, str]:
        """PRE gate. Must NOT require map→odom or map→base_link."""
        if not self._map_name():
            return False, 'no_active_map'
        if self._map is None:
            return False, 'no_map'
        if not self._topic_has_publisher('/map'):
            return False, 'map_publisher_missing'
        if self._scan is None or self._scan_mono is None:
            return False, 'no_scan'
        age = self._mono() - self._scan_mono
        if age > float(self.get_parameter('scan_fresh_sec').value):
            return False, f'scan_stale_{age:.1f}s'
        if not self._tf_ok('odom', 'base_link'):
            return False, 'tf_odom_base'
        laser = ''
        if self._scan is not None and (self._scan.header.frame_id or '').strip():
            laser = self._scan.header.frame_id.strip()
        if not laser:
            laser = str(self.get_parameter('laser_frame').value or 'lidar_link')
        if self._laser_tf_ok or self._tf_ok('base_link', laser):
            self._laser_tf_ok = True
        else:
            return False, f'tf_base_laser_{laser}'
        if bool(self.get_parameter('require_amcl_node').value) and not self._amcl_interfaces_ready():
            return False, 'amcl_not_ready'
        return True, 'ok'

    def _rgb_sensor_ready(self) -> bool:
        for topic in (
            '/camera/front_up/color/image_raw',
            '/camera/front_up/color/image_raw/compressed',
        ):
            if self._topic_has_publisher(topic):
                return True
        return False

    def _wait_rgb_for_p3(self) -> bool:
        self._rgb_req.publish(Bool(data=True))
        timeout = float(self.get_parameter('rgb_wait_sec').value)
        t0 = self._mono()
        while self._mono() - t0 < timeout and rclpy.ok():
            if self._rgb_sensor_ready():
                return True
            time.sleep(0.25)
        return self._rgb_sensor_ready()

    def _wait_pre_localization(self, session_id: int) -> bool:
        self._set_state(BootState.PRE_LOCALIZATION_READY)
        self._goals_latched = True
        self._emit_canonical(Phase2CLocState.BOOT_LOCALIZING.value, True, 'boot_pre')
        self._phase2c_rec_pub.publish(Bool(data=True))
        self._arm_session_io()
        timeout = float(self.get_parameter('sensor_timeout_sec').value)
        t0 = self._mono()
        last_reason = ''
        while self._mono() - t0 < timeout and rclpy.ok():
            if self._cancelled(session_id):
                self._stage(
                    'PRE_LOCALIZATION_READY', attempt=1, result='cancelled', reason='session'
                )
                return False
            ok, reason = self._pre_localization_ready()
            last_reason = reason
            if ok:
                map_odom_before = self._tf_ok('map', 'odom')
                self._milestones['pre_ok_mono'] = self._mono()
                self._milestones['pre_ok_wall'] = time.time()
                self._milestones['map_odom_before_seed'] = bool(map_odom_before)
                self._stage(
                    'PRE_LOCALIZATION_READY',
                    attempt=1,
                    result='ok',
                    reason=reason,
                    runtime_sec=self._mono() - t0,
                    map_odom_required=False,
                    map_odom_before_seed=bool(map_odom_before),
                )
                return True
            time.sleep(0.25)
        self._stage(
            'PRE_LOCALIZATION_READY',
            attempt=1,
            result='timeout',
            reason=last_reason,
            runtime_sec=self._mono() - t0,
            map_odom_required=False,
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
        self._self_ip_suppress_until = self._mono() + 1.0
        self._initialpose_pub.publish(msg)
        now = self._mono()
        self._milestones['initialpose_publish_mono'] = now
        self._milestones['initialpose_publish_wall'] = time.time()
        self._milestones['initialpose_source'] = source
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
        tf_ok_after: Optional[float] = None
        status_0_after: Optional[float] = None
        far_latched_rejected = 0

        while self._mono() < deadline and rclpy.ok():
            if self._nomotion.service_is_ready() and (self._mono() - t0) >= 0.5 * nomotion:
                try:
                    self._nomotion.call_async(Empty.Request())
                    nomotion += 1
                except Exception:  # noqa: BLE001
                    pass
            time.sleep(0.1)
            if self._tf_ok('map', 'odom') and 'first_map_odom_mono' not in self._milestones:
                self._milestones['first_map_odom_mono'] = self._mono()
                self._milestones['first_map_odom_wall'] = time.time()
            if self._amcl is None or self._amcl_mono is None:
                continue
            # Stationary AMCL does not republish after an identical /initialpose.
            # A latched pose may count only if it is already the seed neighborhood.
            # A far latched pose must not become READY without a post-seed update.
            latched_only = (
                amcl_mono_before is not None and self._amcl_mono <= amcl_mono_before
            )
            age = self._mono() - self._amcl_mono
            pose_fresh = age <= max(fresh_lim, 30.0)
            tf_map_odom = self._tf_ok('map', 'odom')
            tf_map_base = self._tf_ok('map', 'base_link')
            tf_ok = tf_map_odom and tf_map_base
            if tf_map_odom and 'first_map_odom_mono' not in self._milestones:
                self._milestones['first_map_odom_mono'] = self._mono()
                self._milestones['first_map_odom_wall'] = time.time()
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
            if not latched_pose_may_satisfy_ready(latched_only, near):
                far_latched_rejected += 1
                last = {
                    'amcl_pose': pose,
                    'near_candidate': False,
                    'latched_pose_used': True,
                    'far_latched_rejected': far_latched_rejected,
                    'gates_ok': False,
                    'ready': False,
                    'amcl_convergence_sec': self._mono() - t0,
                }
                continue
            if tf_ok and tf_ok_after is None:
                tf_ok_after = self._mono() - t0
            if self._loc_status == 0 and status_0_after is None:
                status_0_after = self._mono() - t0
            status_ok = self._loc_status == 0
            # status==0 or latch-3 clear alone is not READY.
            gates = pose_fresh and tf_ok and cov_ok and near and status_ok
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
                'latched_pose_used': latched_only,
                'far_latched_rejected': far_latched_rejected,
                'tf_ok_after_sec': tf_ok_after,
                'status_0_after_sec': status_0_after,
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
        before = self._amcl_mono
        if self._seeds_this_session:
            self._sequential_fallback_seed_count += 1
        self._seeds_this_session += 1
        self._publish_seed(pose[0], pose[1], pose[2], cov_xy, cov_yaw, path)
        self._stage(
            f'{path}_ACCEPTED',
            attempt=1,
            result='accepted',
            laser_score=laser_diag.get('laser_score'),
            reason='laser_accept',
            score=laser_diag.get('laser_score'),
            candidate_pose={'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
        )
        self._set_state(BootState.WAIT_AMCL_SETTLE)
        settle = float(self.get_parameter('p2_settle_sec').value)
        timeout_was = float(self.get_parameter('amcl_timeout_sec').value)
        # Bounded settle. Do not abandon a laser-accepted candidate on the first
        # TF/status sample. READY gates themselves are unchanged.
        self.set_parameters([
            rclpy.Parameter('amcl_timeout_sec', rclpy.Parameter.Type.DOUBLE, max(settle, 5.0)),
        ])
        try:
            ready, amcl_diag = self._wait_amcl_ready(pose, before)
        finally:
            self.set_parameters([
                rclpy.Parameter(
                    'amcl_timeout_sec', rclpy.Parameter.Type.DOUBLE, timeout_was
                ),
            ])
        handoff = 'READY' if ready else 'P2_HANDOFF_FAILED' if path == 'P2' else 'HANDOFF_FAILED'
        self._stage(
            path,
            attempt=1,
            result=handoff,
            laser_score=laser_diag.get('laser_score'),
            reason='ok' if ready else 'handoff_timeout',
            score=laser_diag.get('laser_score'),
            runtime_sec=laser_diag.get('runtime_sec'),
            amcl=amcl_diag,
            candidate_pose={'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
            amcl_pose=amcl_diag.get('amcl_pose'),
            candidate_rejected=False,
            handoff_failed=not ready,
        )
        if ready:
            self._selected_path = path
            self._set_state(BootState.READY)
        return ready

    # --- P1 / P2 / P3 ---
    def _eval_prior_available(self) -> bool:
        # Production: decide from real /xw/power + /battery_state only.
        # DEV-only latch inject on charger_prior_available is ignored unless
        # accept_external_prior_inject=true (bench isolation).
        if bool(self.get_parameter('accept_external_prior_inject').value) and self._charger_prior_avail:
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
        self._milestones.setdefault('p1_start_mono', t0)
        self._milestones.setdefault('p1_start_wall', time.time())
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
        if wp is None or self._scan is None or self._map is None or not self._ensure_field():
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
        self._milestones.setdefault('p2_start_mono', t0)
        self._milestones.setdefault('p2_start_wall', time.time())
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
        if self._scan is None or self._map is None or not self._ensure_field():
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
        self._milestones.setdefault('p3_start_mono', self._mono())
        self._milestones.setdefault('p3_start_wall', time.time())
        if not self._wait_rgb_for_p3():
            self._stage('P3', attempt=1, result='fail', reason='rgb_not_ready')
            return False
        max_attempts = int(self.get_parameter('p3_max_attempts').value)
        cooldown = float(self.get_parameter('p3_cooldown_sec').value)
        for attempt in range(1, max_attempts + 1):
            t0 = self._mono()
            out = self._call_relocalize()
            code = int(out.get('result_code', -1))
            runtime = self._mono() - t0
            if code == _RC_READY or (out.get('ok') and code == _RC_READY):
                # Reloc already published the single /initialpose. Do not seed again.
                cand = out.get('candidate_pose') or self._amcl_tuple()
                if cand is None:
                    self._stage(
                        'P3',
                        attempt=attempt,
                        result='fail',
                        reason='reloc_ready_no_pose',
                        runtime_sec=runtime,
                        reloc=out,
                    )
                    continue
                self._milestones.setdefault('initialpose_publish_mono', self._mono())
                self._milestones.setdefault('initialpose_publish_wall', time.time())
                self._milestones['initialpose_source'] = 'reloc_p3'
                if self._seeds_this_session:
                    self._sequential_fallback_seed_count += 1
                self._seeds_this_session += 1
                self._set_state(BootState.WAIT_AMCL_SETTLE)
                ready, amcl_diag = self._wait_amcl_ready(tuple(cand), None)
                self._stage(
                    'P3',
                    attempt=attempt,
                    result='READY' if ready else 'AMCL_FAIL',
                    score=out.get('laser_score'),
                    reason='relocalize_ready' if ready else 'post_seed_not_stable',
                    runtime_sec=runtime,
                    reloc=out,
                    candidate_pose=cand,
                    amcl=amcl_diag,
                )
                if ready:
                    self._selected_path = 'P3'
                    self._set_state(BootState.READY)
                    return True
                if attempt < max_attempts:
                    self.get_logger().warn(f'P3 post-seed not stable; cooldown {cooldown:.0f}s')
                    time.sleep(cooldown)
                continue
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

    def _operator_owner_fresh(self) -> bool:
        info = parse_owner(self._owner_raw)
        if info.get('owner') != InitialPoseOwner.OPERATOR.value:
            return False
        fresh = float(self.get_parameter('operator_owner_fresh_sec').value)
        return (self._mono() - self._owner_mono) <= fresh

    def _awaiting_operator(self) -> bool:
        if self._busy or self._op_verify_busy:
            return False
        # Canonical incident wins over this node's last BOOT READY latch.
        if self._canonical_state == Phase2CLocState.NEED_OPERATOR.value:
            return True
        if self._phase2c_loc == Phase2CLocState.NEED_OPERATOR.value:
            return True
        if self._state in (BootState.SENSOR_TIMEOUT, BootState.UNKNOWN):
            return True
        return False

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        now = self._mono()
        self._milestones.setdefault('initialpose_publish_mono', now)
        self._milestones.setdefault('initialpose_publish_wall', time.time())
        if now < self._self_ip_suppress_until or self._busy or self._op_verify_busy:
            return
        if not self._awaiting_operator():
            return
        p = msg.pose.pose
        pose = (float(p.position.x), float(p.position.y), _yaw_from_quat(p.orientation))
        threading.Thread(target=self._operator_pose_entry, args=(pose,), daemon=True).start()

    def _operator_pose_entry(self, pose: Tuple[float, float, float]) -> None:
        t0 = self._mono()
        while self._mono() - t0 < 1.0 and rclpy.ok():
            if self._operator_owner_fresh():
                self._run_operator_verify(pose)
                return
            time.sleep(0.05)
        self.get_logger().warn(
            'operator /initialpose ignored: web path must declare initialpose_owner=operator'
        )

    def _run_operator_verify(self, pose: Tuple[float, float, float]) -> None:
        if self._busy or self._op_verify_busy:
            return
        if not self._awaiting_operator() and self._state not in (
            BootState.SENSOR_TIMEOUT,
            BootState.UNKNOWN,
        ):
            # Re-check after the owner wait; still require latched NEED_OPERATOR.
            if not (self._goals_latched and self._phase2c_loc == Phase2CLocState.NEED_OPERATOR.value):
                return
        self._op_verify_busy = True
        t0 = self._mono()
        try:
            self.get_logger().info(
                f'operator initialpose VERIFYING ({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f})'
            )
            self._arm_session_io()
            self._set_state(BootState.VERIFYING_OPERATOR_POSE)
            # Do not unblock goals until POST-SEED window holds.
            self._goals_latched = True
            self._emit_canonical(
                Phase2CLocState.VERIFYING_OPERATOR_POSE.value, True, 'boot_operator'
            )
            before = self._amcl_mono
            if self._nomotion.service_is_ready():
                try:
                    self._nomotion.call_async(Empty.Request())
                except Exception:  # noqa: BLE001
                    pass
            ready, amcl_diag = self._wait_amcl_ready(pose, before)
            self._stage(
                'OPERATOR',
                attempt=1,
                result='READY' if ready else 'AMCL_FAIL',
                reason='operator_post_seed' if ready else 'operator_pose_not_stable',
                amcl=amcl_diag,
                candidate_pose={'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
            )
            if ready:
                self._selected_path = 'OPERATOR'
                self._goals_latched = False
                self._phase2c_rec_pub.publish(Bool(data=False))
                self._milestones['operator_ready_mono'] = self._mono()
                self._milestones['operator_ready_wall'] = time.time()
                self._set_state(BootState.READY)
                self._owner_pub.publish(
                    String(data=owner_payload(InitialPoseOwner.NONE, self._session_id, note='operator_ready'))
                )
                self.get_logger().info('operator pose verified → READY goals_blocked=false')
            else:
                self._goals_latched = True
                self._phase2c_loc = Phase2CLocState.NEED_OPERATOR.value
                self._set_state(BootState.UNKNOWN)
                self.get_logger().warn(
                    'operator pose rejected (no stable AMCL/TF/cov/status window); stay NEED_OPERATOR'
                )
            self._result_pub.publish(
                String(
                    data=json.dumps(
                        {
                            'final': 'READY' if ready else 'NEED_OPERATOR',
                            'selected_path': 'OPERATOR',
                            'source': 'operator',
                            'stages': self._stages[-2:],
                            'amcl': amcl_diag,
                            'candidate_pose': {'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
                            'far_latched_rejected': amcl_diag.get('far_latched_rejected', 0),
                        },
                        default=str,
                    )
                )
            )
        finally:
            self._op_verify_busy = False
            self._rgb_req.publish(Bool(data=False))
            self._disarm_session_io()
            self._publish_status()

    def _finish(self, final: str, t0: float) -> None:
        self._phase2c_rec_pub.publish(Bool(data=False))
        self._rgb_req.publish(Bool(data=False))
        if final == 'READY':
            self._goals_latched = False
            self._milestones['ready_mono'] = self._mono()
            self._milestones['ready_wall'] = time.time()
            self._set_state(BootState.READY)
        elif final == 'CANCELLED':
            # Superseded by newer session — new cascade owns goals_blocked.
            self._set_state(BootState.IDLE)
        else:
            # UNKNOWN / SENSOR_TIMEOUT: keep goals blocked (NEED_OPERATOR).
            self._goals_latched = True
            if final == 'SENSOR_TIMEOUT':
                self._set_state(BootState.SENSOR_TIMEOUT)
            else:
                self._set_state(BootState.UNKNOWN)
        if self._owner.holding():
            self._owner_pub.publish(String(data=self._owner.release_json(f'boot_{final}')))
            self._owner.end()
        amcl = self._amcl_tuple()
        self._report = {
            'final': final,
            'selected_path': self._selected_path or None,
            'stages': self._stages,
            'total_boot_localization_sec': self._mono() - t0,
            'amcl_pose': amcl,
            'map_name': self._map_name(),
            'session_id': self._session_id,
            'note': 'READY requires post-seed AMCL+cov+TF+status0+stable_window; never latch-clear alone',
            'false_accept_policy': 'FA is a wrong candidate reaching AMCL READY; sequential P2 handoff-fail then P3 is not FH',
            'sequential_fallback_seed_count': self._sequential_fallback_seed_count,
            'milestones': self._milestones,
            'cold_break': {
                'map_odom_required_in_pre': False,
                'initialpose_publish_mono': self._milestones.get('initialpose_publish_mono'),
                'first_map_odom_mono': self._milestones.get('first_map_odom_mono'),
            },
        }
        self._result_pub.publish(String(data=json.dumps(self._report, default=str)))
        self.get_logger().info(
            f'BOOT done final={final} path={self._selected_path} '
            f't={self._report["total_boot_localization_sec"]:.1f}s'
        )
        self._publish_status()

    def _run_cascade(self, source: str, session_id: int) -> None:
        t0 = self._mono()
        self._stages = []
        self._selected_path = ''
        self._report = {}
        self._seeds_this_session = 0
        self._sequential_fallback_seed_count = 0
        try:
            self._stage('START', attempt=1, result='ok', reason=source, session_id=session_id)
            if self._cancelled(session_id):
                self._finish('CANCELLED', t0)
                return
            if not self._wait_pre_localization(session_id):
                if self._cancelled(session_id):
                    self._finish('CANCELLED', t0)
                else:
                    self._finish('SENSOR_TIMEOUT', t0)
                return
            if self._cancelled(session_id):
                self._finish('CANCELLED', t0)
                return
            if self._try_p1():
                self._finish('READY', t0)
                return
            if self._cancelled(session_id):
                self._finish('CANCELLED', t0)
                return
            if self._try_p2():
                self._finish('READY', t0)
                return
            if self._cancelled(session_id):
                self._finish('CANCELLED', t0)
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
            pending: Optional[Tuple[str, int]] = None
            with self._lock:
                self._busy = False
                if self._pending_boot is not None:
                    pending = self._pending_boot
                    self._pending_boot = None
            if self._owner.holding():
                self._owner_pub.publish(String(data=self._owner.release_json('boot_finally')))
                self._owner.end()
            if pending is None:
                self._disarm_session_io()
            self._publish_status()
            if pending is not None:
                self._start_async(pending[0], session_id=pending[1])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BootLocalizerNode()
    try:
        from rclpy.executors import MultiThreadedExecutor

        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(node)
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
