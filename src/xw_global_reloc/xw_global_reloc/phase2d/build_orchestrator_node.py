#!/usr/bin/env python3
"""Phase2D Visual DB Build orchestrator — B2 patrol + C2 AUTO_BUILD promote.

Modes:
  micro|partial|full  → Candidate-only (B2; never touches Active)
  AUTO_BUILD          → patrol → validate → FA gate → next version → safe promote/reload
  TARGETED_BUILD      → reserved (targets payload); same pipeline when targets provided

Phase2C LOST → PAUSED (Build never owns localization).
"""

from __future__ import annotations

import json
import math
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger

from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files
from xw_global_reloc.phase2d.candidate_writer import CandidateWriter
from xw_global_reloc.phase2d.config_loader import load_phase2d_config, production_visual_root
from xw_global_reloc.phase2d.coverage_model import build_coverage_model
from xw_global_reloc.phase2d.coverage_report import build_coverage_dict, write_coverage_report
from xw_global_reloc.phase2d.build_completion import (
    evaluate_coverage_completion,
    format_completion_status_text,
    remaining_gap_cells,
    update_nav_fail_history,
    load_nav_fail_history,
)
from xw_global_reloc.phase2d.patrol_planner import PatrolGoal, plan_patrol_goals
from xw_global_reloc.phase2d.version_store import (
    next_visual_version,
    resolve_active_root,
    short_version_label,
)


_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

PRODUCTION_BUILD_MODES = frozenset({'AUTO_BUILD', 'TARGETED_BUILD', 'RESUME_BUILD'})
CANDIDATE_ONLY_MODES = frozenset({'micro', 'partial', 'full'})

STATES = (
    'IDLE',
    'PRECHECK',
    'PLANNING',
    'PATROLLING',
    'COLLECTING',
    'VALIDATING',
    'PROMOTING',
    'RELOADING',
    'COMPLETE',
    'PAUSED',
    'FAILED',
    'FAILED_VALIDATION',
    'FAILED_PROMOTE',
    'FAILED_RELOAD',
    'ABORTED',
    'NEED_OPERATOR',
)

# Stop must not rewrite these mid-transaction / terminal states.
_STOP_PROTECTED = frozenset(
    {
        'COMPLETE',
        'FAILED',
        'FAILED_VALIDATION',
        'FAILED_PROMOTE',
        'FAILED_RELOAD',
        'ABORTED',
        'PROMOTING',
        'RELOADING',
    }
)


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _parse_state(raw: str) -> str:
    try:
        data = json.loads(raw or '{}')
        if isinstance(data, dict) and data.get('state'):
            return str(data['state'])
    except json.JSONDecodeError:
        pass
    return str(raw or '').strip()


@dataclass
class BuildSession:
    build_session_id: str
    mode: str
    state: str = 'IDLE'
    start_time: float = 0.0
    end_time: float = 0.0
    planned_goals: int = 0
    reached_goals: int = 0
    nav_failed: int = 0
    accepted_candidates: int = 0
    duplicate_skips: int = 0
    covered_skips: int = 0
    image_rejects: int = 0
    laser_rejects: int = 0
    pose_rejects: int = 0
    quota_skips: int = 0
    other_skips: int = 0
    coverage_before: Dict[str, Any] = field(default_factory=dict)
    coverage_after: Dict[str, Any] = field(default_factory=dict)
    current_goal: Optional[Dict[str, Any]] = None
    goals: List[Dict[str, Any]] = field(default_factory=list)
    message: str = ''
    dry_run: bool = False
    simulate_nav: bool = False
    patrol_mode: str = 'micro'
    build_kind: str = 'CANDIDATE_ONLY'  # AUTO_BUILD | TARGETED_BUILD | CANDIDATE_ONLY
    targets: Dict[str, Any] = field(default_factory=dict)
    old_version: str = ''
    new_version: str = ''
    validation: Dict[str, Any] = field(default_factory=dict)
    promote: Dict[str, Any] = field(default_factory=dict)
    map_name: str = ''
    completion: Dict[str, Any] = field(default_factory=dict)
    stop_reason: str = ''
    map_complete: bool = False
    resume: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class VisualDbBuildOrchestrator(Node):
    def __init__(self, cfg: Optional[Dict[str, Any]] = None) -> None:
        super().__init__('xw_visual_db_build')
        self.declare_parameter('config_path', '')
        self.declare_parameter('maps_dir', '')
        self.declare_parameter('map_name', '')
        self.declare_parameter('simulate_nav', False)
        self.declare_parameter('follow_localization_mode', 'continuous')

        cfg_path = str(self.get_parameter('config_path').value or '').strip()
        self._cfg = cfg or load_phase2d_config(Path(cfg_path) if cfg_path else None)
        md = str(self.get_parameter('maps_dir').value or '').strip()
        mn = str(self.get_parameter('map_name').value or '').strip()
        if md:
            self._cfg['maps_dir'] = md
        if mn:
            self._cfg['map_name'] = mn

        self._cb = ReentrantCallbackGroup()
        self._state_cb = MutuallyExclusiveCallbackGroup()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._session: Optional[BuildSession] = None
        self._nav_goal_handle = None
        self._critical = False  # PROMOTING/RELOADING: ignore user abort rewrite

        self._loc_status: Optional[int] = None
        self._phase2c_state = ''
        self._phase2c_loc = ''
        self._goals_blocked = False
        self._follow = False
        self._recharge = False
        self._explore = False
        self._map_name = str(self._cfg.get('map_name') or 'vp')
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._odom: Optional[Odometry] = None

        self.create_subscription(Int8, '/xw/localization_status', self._on_loc, _LATCH, callback_group=self._state_cb)
        self.create_subscription(
            String, '/xw/localization/phase2c_state', self._on_p2c, _LATCH, callback_group=self._state_cb
        )
        self.create_subscription(
            String, '/xw/localization/phase2c_loc_state', self._on_p2c_loc, _LATCH, callback_group=self._state_cb
        )
        self.create_subscription(Bool, '/xw/nav/goals_blocked', self._on_blocked, _LATCH, callback_group=self._state_cb)
        self.create_subscription(Bool, '/xw/follow/enable', self._on_follow, _LATCH, callback_group=self._state_cb)
        self.create_subscription(Bool, '/xw/recharge/enable', self._on_recharge, _LATCH, callback_group=self._state_cb)
        self.create_subscription(Bool, '/xw/explore/enable', self._on_explore, _LATCH, callback_group=self._state_cb)
        self.create_subscription(String, '/xw/nav/map_name', self._on_map_name, _LATCH, callback_group=self._state_cb)
        self._amcl_sub = None
        self._odom_sub = None

        self.create_subscription(String, '/xw/visual_db/build', self._on_build_cmd, 10, callback_group=self._cb)
        self._status_pub = self.create_publisher(String, '/xw/visual_db/build_status', _LATCH)
        self._nav_cancel_pub = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb)

        self.create_service(Trigger, '/xw/visual_db/build_status_svc', self._on_status_svc, callback_group=self._cb)
        self._capture_pub = self.create_publisher(String, '/xw/visual_db/capture_candidate', 10)
        self._capture_result: Optional[Dict[str, Any]] = None
        self.create_subscription(
            String, '/xw/visual_db/capture_result', self._on_capture_result, 10, callback_group=self._cb
        )
        self._publish_status()
        self.get_logger().info('Phase2D-C2 build orchestrator ready (AUTO_BUILD + Candidate)')

    def _on_capture_result(self, msg: String) -> None:
        try:
            self._capture_result = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self._capture_result = {'status': 'ERROR', 'reason': 'bad_capture_result'}

    def _on_loc(self, msg: Int8) -> None:
        self._loc_status = int(msg.data)

    def _on_p2c(self, msg: String) -> None:
        self._phase2c_state = _parse_state(msg.data)

    def _on_p2c_loc(self, msg: String) -> None:
        self._phase2c_loc = str(msg.data or '').strip()

    def _on_blocked(self, msg: Bool) -> None:
        self._goals_blocked = bool(msg.data)
        if self._goals_blocked and self._session and self._session.state in ('PATROLLING', 'COLLECTING'):
            self._set_state('PAUSED', 'goals_blocked / localization recovery')

    def _on_follow(self, msg: Bool) -> None:
        self._follow = bool(msg.data)

    def _on_recharge(self, msg: Bool) -> None:
        self._recharge = bool(msg.data)

    def _on_explore(self, msg: Bool) -> None:
        self._explore = bool(msg.data)

    def _on_map_name(self, msg: String) -> None:
        name = str(msg.data or '').strip()
        if name:
            self._map_name = name
            self._cfg['map_name'] = name

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _set_state(self, state: str, message: str = '') -> None:
        with self._lock:
            if self._session is None:
                return
            self._session.state = state
            if message:
                self._session.message = message
        self._publish_status()
        self.get_logger().info(f'build state={state} {message}')

    def _publish_status(self) -> None:
        payload = self.status_dict()
        self._status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def status_dict(self) -> Dict[str, Any]:
        with self._lock:
            if self._session is None:
                return {'state': 'IDLE', 'build_session_id': None}
            d = self._session.to_dict()
            before = self._session.coverage_before or {}
            after = self._session.coverage_after or {}
            def _cells(d: Dict[str, Any]) -> int:
                for k in ('cells', 'occupied_cells', 'occupied_spatial_cells', 'active_occupied_cells'):
                    if d.get(k) is not None:
                        return int(d[k])
                return 0

            def _yaw(d: Dict[str, Any]) -> int:
                for k in ('yaw_occupancy', 'active_yaw_occupancy', 'candidate_new_yaw_bins'):
                    if d.get(k) is not None and k != 'candidate_new_yaw_bins':
                        return int(d[k])
                return int(d.get('candidate_new_yaw_bins') or 0)

            d['progress'] = {
                'planned': self._session.planned_goals,
                'reached': self._session.reached_goals,
                'accepted': self._session.accepted_candidates,
                'nav': f'{self._session.reached_goals} / {self._session.planned_goals}',
                'new_cells': max(0, _cells(after) - _cells(before))
                if after
                else int(before.get('candidate_new_cells') or 0),
                'new_yaw': max(0, _yaw(after) - _yaw(before))
                if after and _yaw(before)
                else int(before.get('candidate_new_yaw_bins') or after.get('candidate_new_yaw_bins') or 0),
            }
            d['ui'] = {
                'map_name': self._session.map_name or self._map_name,
                'old_version_short': short_version_label(self._session.old_version),
                'new_version_short': short_version_label(self._session.new_version),
            }
            comp = self._session.completion or {}
            d['completion_summary'] = {
                'eligible': (comp.get('eligible') or {}).get('eligible_visual_cells'),
                'covered': comp.get('covered_eligible'),
                'spatial_coverage_ratio': comp.get('spatial_coverage_ratio'),
                'yaw_sufficient': comp.get('yaw_sufficient'),
                'yaw_insufficient': comp.get('yaw_insufficient'),
                'nav_failed_retryable': comp.get('nav_failed_retryable'),
                'unreachable': comp.get('unreachable'),
                'gate_pass': comp.get('gate_pass'),
                'map_complete_claim_allowed': comp.get('map_complete_claim_allowed'),
                'map_complete': self._session.map_complete,
                'stop_reason': self._session.stop_reason,
                'status_text': comp.get('status_text') or '',
            }
            return d

    def _on_status_svc(self, request, response):  # noqa: ANN001, ARG002
        response.success = True
        response.message = json.dumps(self.status_dict(), separators=(',', ':'))
        return response

    def _on_build_cmd(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            payload = {}
        action = str(payload.get('action') or 'start').lower()
        if action in ('stop', 'abort', 'cancel'):
            self.stop_build(abort=True)
            return
        if action == 'status':
            self._publish_status()
            return
        mode = str(payload.get('mode') or self._cfg.get('patrol', {}).get('mode_default') or 'micro')
        patrol_mode = str(payload.get('patrol_mode') or payload.get('scope') or '')
        dry_run = bool(payload.get('dry_run', False))
        simulate = bool(payload.get('simulate_nav', self.get_parameter('simulate_nav').value))
        targets = payload.get('targets') if isinstance(payload.get('targets'), dict) else {}
        self.start_build(
            mode=mode,
            dry_run=dry_run,
            simulate_nav=simulate,
            patrol_mode=patrol_mode or None,
            targets=targets,
        )

    def start_build(
        self,
        *,
        mode: str = 'micro',
        dry_run: bool = False,
        simulate_nav: bool = False,
        patrol_mode: Optional[str] = None,
        targets: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        mode_u = str(mode or 'micro').strip()
        mode_up = mode_u.upper()
        resume = False
        if mode_up in ('RESUME_BUILD', 'RESUME', 'CONTINUE'):
            build_kind = 'RESUME_BUILD'
            pmode = str(patrol_mode or 'full')
            resume = True
        elif mode_up in ('FULL_AUTO_BUILD', 'FULL_BUILD'):
            build_kind = 'AUTO_BUILD'
            pmode = 'full'
        elif mode_up in PRODUCTION_BUILD_MODES:
            build_kind = mode_up
            pmode = str(patrol_mode or self._cfg.get('auto_build', {}).get('patrol_mode_default') or 'micro')
            if build_kind == 'RESUME_BUILD':
                resume = True
                pmode = str(patrol_mode or 'full')
        elif mode_u in CANDIDATE_ONLY_MODES:
            build_kind = 'CANDIDATE_ONLY'
            pmode = mode_u
        else:
            # Unknown → treat as AUTO_BUILD with given patrol_mode if present
            build_kind = 'AUTO_BUILD' if mode_up == 'AUTO_BUILD' else 'CANDIDATE_ONLY'
            pmode = str(patrol_mode or mode_u or 'micro')

        pmode_l = str(pmode).lower()
        if pmode_l in ('full_auto_build', 'full_build'):
            pmode_l = 'full'
        if pmode_l not in ('micro', 'partial', 'full'):
            pmode_l = 'micro'

        with self._lock:
            if self._worker and self._worker.is_alive():
                return {'ok': False, 'message': 'build_busy', **self.status_dict()}
            sid = f'build_{time.strftime("%Y%m%d_%H%M%S")}_{uuid.uuid4().hex[:6]}'
            maps_dir = Path(str(self._cfg.get('maps_dir') or '/ros2_ws/maps'))
            _, active_ver, _ = resolve_active_root(maps_dir, self._map_name)
            try:
                nxt = next_visual_version(active_ver) if build_kind in PRODUCTION_BUILD_MODES else ''
            except ValueError:
                nxt = ''
            self._session = BuildSession(
                build_session_id=sid,
                mode=mode_u if build_kind == 'CANDIDATE_ONLY' else build_kind,
                state='PRECHECK',
                start_time=time.time(),
                dry_run=dry_run,
                simulate_nav=simulate_nav,
                patrol_mode=pmode_l,
                build_kind=build_kind,
                targets=dict(targets or {}),
                old_version=str(active_ver or ''),
                new_version=nxt,
                map_name=self._map_name,
                resume=resume or build_kind == 'RESUME_BUILD',
            )
            self._stop.clear()
            self._critical = False
        self._arm_motion_subs()
        self._publish_status()
        self._worker = threading.Thread(target=self._run_build, daemon=True)
        self._worker.start()
        return {'ok': True, **self.status_dict()}

    def _arm_motion_subs(self) -> None:
        if self._amcl_sub is None:
            self._amcl_sub = self.create_subscription(
                PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10, callback_group=self._state_cb
            )
        if self._odom_sub is None:
            self._odom_sub = self.create_subscription(
                Odometry, '/odom', self._on_odom, 10, callback_group=self._state_cb
            )

    def _disarm_motion_subs(self) -> None:
        for attr in ('_amcl_sub', '_odom_sub'):
            sub = getattr(self, attr)
            if sub is not None:
                try:
                    self.destroy_subscription(sub)
                except Exception:  # noqa: BLE001
                    pass
                setattr(self, attr, None)
        self._amcl = None
        self._odom = None

    def stop_build(self, *, abort: bool = True) -> Dict[str, Any]:
        self._stop.set()
        try:
            self._nav_cancel_pub.publish(Bool(data=True))
        except Exception:  # noqa: BLE001
            pass
        gh = self._nav_goal_handle
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        with self._lock:
            cur = self._session.state if self._session else None
            critical = self._critical
        # During PROMOTING/RELOADING: request stop for post-step abort only; never leave half Active.
        if critical or cur in ('PROMOTING', 'RELOADING'):
            self.get_logger().warn('stop ignored during critical promote/reload (transactional)')
            return self.status_dict()
        if abort and cur not in _STOP_PROTECTED:
            self._set_state('ABORTED', 'user_stop')
            with self._lock:
                if self._session:
                    self._session.end_time = time.time()
                    self._persist_session()
            self._disarm_motion_subs()
        return self.status_dict()

    def _persist_session(self) -> None:
        if not self._session:
            return
        root = production_visual_root(self._cfg) / 'state'
        root.mkdir(parents=True, exist_ok=True)
        path = root / f'{self._session.build_session_id}.json'
        path.write_text(json.dumps(self._session.to_dict(), indent=2) + '\n', encoding='utf-8')

    def _precheck(self) -> Tuple[bool, str]:
        simulate = bool(self._session and self._session.simulate_nav)
        state = self._phase2c_state or self._phase2c_loc
        if not simulate:
            if not state or state != 'READY':
                return False, f'phase2c_not_ready:{state or "missing"}'
            if self._loc_status is None or int(self._loc_status) != 0:
                return False, f'localization_status={self._loc_status}'
        else:
            if state and state not in ('READY', ''):
                if state in ('LOST', 'RECOVERING', 'NEED_OPERATOR', 'BOOT_LOCALIZING'):
                    return False, f'loc_state:{state}'
            if self._loc_status is not None and int(self._loc_status) != 0:
                return False, f'localization_status={self._loc_status}'
        if self._goals_blocked and not simulate:
            return False, 'goals_blocked'
        if self._follow:
            return False, 'follow_active'
        if self._recharge:
            return False, 'recharge_active'
        if self._explore and not simulate:
            return False, 'explore_active_map_not_frozen'
        if state in ('LOST', 'RECOVERING', 'NEED_OPERATOR', 'BOOT_LOCALIZING'):
            return False, f'loc_state:{state}'

        maps_dir = Path(str(self._cfg.get('maps_dir') or '/ros2_ws/maps'))
        map_name = self._map_name
        if not map_name:
            return False, 'map_name_invalid'
        try:
            y, p = resolve_map_files(maps_dir, map_name)
            if not y.is_file() or not p.is_file():
                return False, 'map_not_saved'
            cur = map_pair_hash(y, p)
        except FileNotFoundError:
            return False, 'map_files_missing'

        root, version, source = resolve_active_root(maps_dir, map_name)
        if source == 'missing':
            return False, 'active_version_missing'
        if not version or version == 'missing':
            return False, 'current_active_version_invalid'

        man = root / 'manifest.yaml'
        if not man.is_file():
            return False, 'active_manifest_missing'
        import yaml

        data = yaml.safe_load(man.read_text(encoding='utf-8')) or {}
        db_hash = str(data.get('map_hash') or '')
        if db_hash and cur and db_hash != cur:
            return False, 'map_hash_mismatch'

        writer = CandidateWriter(self._cfg)
        try:
            writer.ensure_dirs()
            writer.assert_not_production(writer.root)
        except Exception as exc:  # noqa: BLE001
            return False, f'candidate_writer:{exc}'

        if self._session and self._session.build_kind == 'TARGETED_BUILD':
            targets = self._session.targets or {}
            if not any(targets.get(k) for k in ('rooms', 'waypoints', 'cells', 'goals')):
                return False, 'targeted_build_requires_targets'

        if self._session and not self._session.simulate_nav:
            if bool(self._cfg.get('patrol', {}).get('use_nav2_action', True)):
                if not self._nav_client.wait_for_server(timeout_sec=3.0):
                    return False, 'nav2_action_unavailable'
        return True, f'ok active={version}'

    def _run_build(self) -> None:
        assert self._session is not None
        try:
            ok, reason = self._precheck()
            if not ok:
                self._set_state('FAILED', reason)
                self._session.end_time = time.time()
                self._persist_session()
                return
            self._set_state('PRECHECK', reason)

            model = build_coverage_model(self._cfg, load_descriptors=False)
            before = build_coverage_dict(model)['summary']
            self._session.coverage_before = before

            maps_dir = Path(str(self._cfg.get('maps_dir') or '/ros2_ws/maps'))
            map_yaml = maps_dir / f'{self._map_name}.yaml'
            seed = None
            if self._amcl is not None:
                seed = (float(self._amcl.pose.pose.position.x), float(self._amcl.pose.pose.position.y))

            patrol_cfg = dict(self._cfg.get('patrol') or {})
            auto_cfg = dict(self._cfg.get('auto_build') or {})
            pmode = str(self._session.patrol_mode or 'micro')
            is_full = pmode == 'full' or self._session.resume
            if is_full:
                max_rounds = int(auto_cfg.get('full_max_planning_rounds') or 12)
                max_session = float(auto_cfg.get('full_max_session_sec') or 7200)
                # Per-round goal budget still uses patrol.max_total_goals
                patrol_cfg = dict(patrol_cfg)
                patrol_cfg['max_total_goals'] = int(
                    auto_cfg.get('full_max_goals_per_round')
                    or patrol_cfg.get('max_total_goals', 40)
                )
                session_goal_cap = int(auto_cfg.get('full_max_total_goals') or 120)
            else:
                max_rounds = int(auto_cfg.get('max_planning_rounds') or patrol_cfg.get('max_planning_rounds', 2))
                max_session = float(auto_cfg.get('max_session_sec') or patrol_cfg.get('max_session_sec', 1800))
                session_goal_cap = int(patrol_cfg.get('max_total_goals', 40)) * max_rounds
            max_cand = int(self._cfg.get('candidate_limits', {}).get('max_per_build_session', 500))
            if self._session.build_kind in PRODUCTION_BUILD_MODES:
                # Micro one-click default: small goal budget
                if self._session.patrol_mode == 'micro':
                    patrol_cfg = dict(patrol_cfg)
                    patrol_cfg['micro_max_goals'] = int(
                        auto_cfg.get('micro_max_goals') or patrol_cfg.get('micro_max_goals', 6)
                    )

            vroot = production_visual_root(self._cfg)
            stop_reason = ''
            total_planned = 0
            for round_i in range(max_rounds):
                if self._stop.is_set():
                    stop_reason = 'user_stop'
                    break
                if time.time() - self._session.start_time > max_session:
                    stop_reason = 'session_timeout'
                    break
                if self._session.accepted_candidates >= max_cand:
                    stop_reason = 'candidate_session_quota'
                    break
                if total_planned >= session_goal_cap:
                    stop_reason = 'max_total_goals'
                    break

                self._set_state('PLANNING', f'round={round_i+1}')
                model = build_coverage_model(self._cfg, load_descriptors=False)
                hist = load_nav_fail_history(vroot)
                session_fail_ids = {
                    str(g.get('spatial_cell'))
                    for g in self._session.goals
                    if g.get('nav') == 'NAV_FAILED'
                }
                completion = evaluate_coverage_completion(
                    model,
                    map_yaml,
                    self._cfg,
                    seed_xy=seed,
                    nav_fail_history=hist,
                    session_nav_failed_cells=session_fail_ids,
                    patrol_mode=pmode,
                    build_kind=self._session.build_kind,
                )
                comp_dict = completion.as_dict()
                comp_dict['status_text'] = format_completion_status_text(completion, mode=pmode)
                self._session.completion = comp_dict
                self._session.map_complete = bool(completion.map_complete_claim_allowed)
                self._publish_status()

                # FULL / RESUME: stop patrol early when Coverage Gate met
                if is_full and completion.gate_pass:
                    stop_reason = (
                        'coverage_gate_pass'
                        if completion.map_complete_claim_allowed
                        else 'coverage_gate_pass_micro_blocked'
                    )
                    break

                only_cells = None
                exclude_cells = set(completion.unreachable)
                if self._session.resume or is_full:
                    gaps = remaining_gap_cells(completion)
                    only_cells = gaps if gaps else set()
                    if not gaps:
                        stop_reason = 'no_actionable_gaps'
                        break

                # Temporary cfg override for this round's max goals
                round_cfg = dict(self._cfg)
                round_cfg['patrol'] = patrol_cfg
                goals = plan_patrol_goals(
                    model,
                    map_yaml=map_yaml,
                    cfg=round_cfg,
                    mode=pmode,
                    seed_xy=seed,
                    should_stop=self._stop.is_set,
                    only_cells=only_cells,
                    exclude_cells=exclude_cells,
                )
                # TARGETED_BUILD: filter to requested cells/waypoints when provided
                goals = self._apply_targets(goals)
                # Session goal cap
                remain_cap = max(0, session_goal_cap - total_planned)
                if remain_cap <= 0:
                    stop_reason = 'max_total_goals'
                    break
                if len(goals) > remain_cap:
                    goals = goals[:remain_cap]
                seen = {(g['spatial_cell'], g['yaw_bin']) for g in self._session.goals}
                fresh = [g for g in goals if (g.spatial_cell, g.yaw_bin) not in seen]
                if not fresh:
                    if round_i == 0 and self._session.build_kind == 'TARGETED_BUILD':
                        self._set_state('NEED_OPERATOR', 'no_targeted_goals')
                        self._session.end_time = time.time()
                        self._persist_session()
                        return
                    if round_i == 0 and self._session.build_kind == 'CANDIDATE_ONLY':
                        self._session.stop_reason = 'no_goals'
                        self._set_state('COMPLETE', 'no_goals')
                        self._session.end_time = time.time()
                        self._persist_session()
                        return
                    stop_reason = stop_reason or 'planner_no_new_goals'
                    break
                self._session.planned_goals += len(fresh)
                total_planned += len(fresh)

                for g in fresh:
                    if self._stop.is_set():
                        stop_reason = 'user_stop'
                        break
                    if time.time() - self._session.start_time > max_session:
                        stop_reason = 'session_timeout'
                        break
                    while not self._stop.is_set() and self._should_pause():
                        self._set_state('PAUSED', 'waiting_localization_ready')
                        time.sleep(0.5)
                    if self._stop.is_set():
                        stop_reason = 'user_stop'
                        break
                    if (self._phase2c_state or self._phase2c_loc) == 'NEED_OPERATOR':
                        self._set_state('NEED_OPERATOR', 'phase2c_NEED_OPERATOR')
                        self._session.end_time = time.time()
                        self._persist_session()
                        return

                    self._session.current_goal = g.as_dict()
                    self._session.goals.append(g.as_dict())
                    self._set_state('PATROLLING', f'nav {g.spatial_cell} yaw_bin={g.yaw_bin}')
                    nav_ok, nav_code = self._navigate_to(g)
                    self._session.goals[-1]['nav_result_code'] = nav_code
                    self._session.goals[-1]['nav_retryable'] = True
                    if not nav_ok:
                        self._session.nav_failed += 1
                        self._session.goals[-1]['nav'] = 'NAV_FAILED'
                        self._session.goals[-1]['future_resume_candidate'] = True
                        self._session.goals[-1]['permanent_unreachable'] = False
                        self._publish_status()
                        continue
                    self._session.reached_goals += 1
                    self._session.goals[-1]['nav'] = 'REACHED'
                    self._session.goals[-1]['future_resume_candidate'] = False

                    settle = float(patrol_cfg.get('settle_sec', 1.0))
                    self._wait_settle(settle)
                    if self._stop.is_set():
                        stop_reason = 'user_stop'
                        break

                    self._set_state('COLLECTING', f'capture {g.spatial_cell}')
                    retries = int(patrol_cfg.get('capture_retry', 1))
                    result_code = self._capture_at_goal(g)
                    if result_code.startswith('REJECTED') and retries > 0 and not self._stop.is_set():
                        time.sleep(0.3)
                        result_code = self._capture_at_goal(g)
                    self._session.goals[-1]['capture'] = result_code
                    self._tally(result_code)
                    self._publish_status()

                if self._stop.is_set():
                    stop_reason = stop_reason or 'user_stop'
                    break

                # After each FULL round: recompute coverage; continue if gaps remain
                if is_full and not stop_reason:
                    continue
                if not is_full:
                    # micro/partial: fixed rounds only
                    pass

            if not stop_reason:
                if pmode == 'micro':
                    stop_reason = 'micro_budget_exhausted'
                elif total_planned >= session_goal_cap:
                    stop_reason = 'max_total_goals'
                else:
                    stop_reason = 'planning_rounds_exhausted'

            # Persist nav-fail history (retryable vs unreachable)
            try:
                hist = update_nav_fail_history(vroot, self._session.goals)
                # Mark permanent unreachable on goals when threshold hit
                thr = int((self._cfg.get('build_completion') or {}).get('nav_fail_sessions_to_unreachable', 3))
                for g in self._session.goals:
                    if g.get('nav') != 'NAV_FAILED':
                        continue
                    cell = str(g.get('spatial_cell') or '')
                    n = int(hist.get(cell, 0))
                    if n >= thr:
                        g['permanent_unreachable'] = True
                        g['nav_retryable'] = False
                        g['future_resume_candidate'] = False
                    else:
                        g['permanent_unreachable'] = False
                        g['nav_retryable'] = True
                        g['future_resume_candidate'] = True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'nav_fail_history update failed: {exc}')

            # Coverage after patrol + completion gate snapshot
            try:
                model = build_coverage_model(self._cfg, load_descriptors=False)
                self._session.coverage_after = build_coverage_dict(model)['summary']
                write_coverage_report(
                    self._cfg,
                    production_visual_root(self._cfg) / 'candidate' / 'reports',
                    model=model,
                )
                hist = load_nav_fail_history(vroot)
                session_fail_ids = {
                    str(g.get('spatial_cell'))
                    for g in self._session.goals
                    if g.get('nav') == 'NAV_FAILED'
                }
                completion = evaluate_coverage_completion(
                    model,
                    map_yaml,
                    self._cfg,
                    seed_xy=seed,
                    nav_fail_history=hist,
                    session_nav_failed_cells=session_fail_ids,
                    patrol_mode=pmode,
                    build_kind=self._session.build_kind,
                )
                comp_dict = completion.as_dict()
                comp_dict['status_text'] = format_completion_status_text(completion, mode=pmode)
                self._session.completion = comp_dict
                self._session.map_complete = bool(completion.map_complete_claim_allowed)
                if completion.map_complete_claim_allowed:
                    stop_reason = 'coverage_gate_pass'
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'coverage report failed: {exc}')

            self._session.stop_reason = stop_reason

            if self._stop.is_set() and self._session.state not in _STOP_PROTECTED | {'NEED_OPERATOR'}:
                self._set_state('ABORTED', 'stopped')
                self._session.end_time = time.time()
                self._persist_session()
                return

            # AUTO_BUILD / TARGETED_BUILD / RESUME_BUILD → validate + promote
            if self._session.build_kind in PRODUCTION_BUILD_MODES and self._session.state not in (
                'FAILED',
                'ABORTED',
                'NEED_OPERATOR',
            ):
                if int(self._session.accepted_candidates or 0) <= 0:
                    # Nothing new to promote — still a successful session end with coverage report
                    label = (
                        'map_complete'
                        if self._session.map_complete
                        else f'session_done:{self._session.stop_reason or "no_new_candidates"}'
                    )
                    self._set_state('COMPLETE', label)
                else:
                    self._run_validate_promote()
            elif self._session.state not in _STOP_PROTECTED | {'NEED_OPERATOR'}:
                # Candidate-only COMPLETE = session finished; map_complete only if gate allows
                label = 'map_complete' if self._session.map_complete else f'session_done:{stop_reason}'
                self._set_state('COMPLETE', label)

            self._session.end_time = time.time()
            self._persist_session()
            self._publish_status()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'build failed: {exc}')
            self._set_state('FAILED', str(exc))
            if self._session:
                self._session.end_time = time.time()
                self._persist_session()
        finally:
            self._critical = False
            if self._session:
                self._session.current_goal = None
            self._disarm_motion_subs()
            self._publish_status()

    def _apply_targets(self, goals: List[PatrolGoal]) -> List[PatrolGoal]:
        if not self._session or self._session.build_kind != 'TARGETED_BUILD':
            return goals
        targets = self._session.targets or {}
        cells = set(str(c) for c in (targets.get('cells') or []))
        if cells:
            goals = [g for g in goals if g.spatial_cell in cells]
        # rooms / waypoints reserved — planner already free-space based
        return goals

    def _run_validate_promote(self) -> None:
        assert self._session is not None
        if self._stop.is_set():
            self._set_state('ABORTED', 'stopped_before_validate')
            return

        self._set_state('VALIDATING', 'c1_validator')
        maps_dir = Path(str(self._cfg.get('maps_dir') or '/ros2_ws/maps'))
        if str(maps_dir).endswith('/maps'):
            work = (
                Path(str(maps_dir).rsplit('/maps', 1)[0])
                / 'bench'
                / f'phase2d_c2_{time.strftime("%Y%m%d")}'
                / self._session.build_session_id
            )
        else:
            work = maps_dir / 'bench' / f'phase2d_c2_{time.strftime("%Y%m%d")}' / self._session.build_session_id
        work.mkdir(parents=True, exist_ok=True)

        from xw_global_reloc.phase2d.c1_validate_promote import run_c1

        # Validate first (stoppable). Promote/reload only if PASS and not stopped.
        report = run_c1(
            maps_dir=maps_dir,
            map_name=self._map_name,
            work_dir=work,
            run_fa=True,
            promote=False,
            reload=False,
            rollback_test=False,
            build_session_id=self._session.build_session_id,
            phase='Phase2D-C2',
            stop_check=self._stop.is_set,
        )
        counts = report.get('counts') or {}
        rejected = int(counts.get('total') or 0) - int(counts.get('verified') or 0)
        self._session.validation = {
            'pass': bool(report.get('validation_pass')),
            'verified': counts.get('verified'),
            'rejected': rejected,
            'false_accept': report.get('false_accept_count'),
            'Hit@1': report.get('Hit@1'),
            'Hit@3': report.get('Hit@3'),
            'Hit@5': report.get('Hit@5'),
            'confusion': (report.get('E3_confusion') or {}).get('visual_confusion_cases'),
            'gate': report.get('promote_gate'),
            'error': report.get('error'),
            'aborted': report.get('aborted'),
            'coverage_before': report.get('coverage_before'),
            'coverage_projected': report.get('coverage_projected'),
        }
        self._session.new_version = str(report.get('target_version') or self._session.new_version)
        self._session.old_version = str(report.get('active_version') or self._session.old_version)
        self._publish_status()

        if report.get('aborted') or self._stop.is_set():
            self._set_state('ABORTED', 'stopped_during_validate')
            return
        if report.get('error'):
            self._set_state('FAILED_VALIDATION', str(report.get('error')))
            return
        if not report.get('validation_pass'):
            fa = report.get('false_accept_count')
            self._set_state('FAILED_VALIDATION', f'gate_failed FA={fa}')
            return

        # Critical section: build version → pointer → reload (no half-finished Active)
        self._critical = True
        try:
            self._set_state('PROMOTING', self._session.new_version)
            from xw_global_reloc.phase2d.c1_validate_promote import (
                build_promoted_version,
                coverage_snapshot_from_db,
                load_candidates,
                filter_candidates_for_promote,
            )
            from xw_global_reloc.phase2d.version_store import (
                inventory_keyframe_hashes,
                validate_version_for_activate,
                version_dir,
                visual_root as _vroot,
            )
            from xw_global_reloc.phase2d.set_active_cli import set_active_version
            from xw_global_reloc.phase2d.config_loader import load_phase2d_config
            from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files

            cfg = load_phase2d_config()
            cfg['maps_dir'] = str(maps_dir)
            cfg['map_name'] = self._map_name
            vroot = _vroot(maps_dir, self._map_name)
            active_root, active_ver, _ = resolve_active_root(maps_dir, self._map_name)
            y, p = resolve_map_files(maps_dir, self._map_name)
            mhash = map_pair_hash(y, p)

            # Reload session candidates and take VERIFIED from first-pass disk marks
            all_recs = load_candidates(vroot / 'candidate')
            recs = filter_candidates_for_promote(all_recs, build_session_id=self._session.build_session_id)
            verified = []
            for r in recs:
                st = str((r.meta.get('validation') or {}).get('status') or '')
                if st == 'VERIFIED':
                    r.status = 'VERIFIED'
                    verified.append(r)

            if not verified:
                self._set_state('FAILED_PROMOTE', 'no_verified_candidates')
                return

            prev_inv = inventory_keyframe_hashes(active_root)
            new_ver = self._session.new_version or next_visual_version(active_ver)
            if version_dir(vroot, new_ver).exists():
                self._set_state('FAILED_PROMOTE', f'target_exists:{new_ver}')
                return

            built = build_promoted_version(
                vroot=vroot,
                active_root=active_root,
                verified=verified,
                map_hash=mhash,
                validation_report=dict(report),
                cfg=cfg,
                previous_version=active_ver,
                target_version=new_ver,
                phase='Phase2D-C2',
            )
            ok, reason = validate_version_for_activate(
                vroot, new_ver, maps_dir=maps_dir, map_name=self._map_name, require_map_hash_match=True
            )
            prev_after = inventory_keyframe_hashes(active_root)
            prev_ok = (
                prev_after['index_sha256'] == prev_inv['index_sha256']
                and prev_after['keyframe_count'] == prev_inv['keyframe_count']
            )
            self._session.promote = {
                'built': str(built),
                'validate': reason,
                'ok': ok,
                'previous_unchanged': prev_ok,
                'promoted_count': len(verified),
                'promoted_ids': [{'cand': r.id, 'kf': r.promoted_id} for r in verified],
                'previous_version': active_ver,
                'new_version': new_ver,
            }
            if not ok or not prev_ok:
                self._set_state('FAILED_PROMOTE', reason if not ok else 'previous_mutated')
                return

            self._set_state('RELOADING', new_ver)
            reload_res = set_active_version(
                maps_dir=maps_dir,
                map_name=self._map_name,
                version=new_ver,
                reload=True,
                timeout=60.0,
            )
            self._session.promote['set_active'] = reload_res
            if reload_res.get('status') != 'OK':
                self._set_state('FAILED_RELOAD', str(reload_res.get('status')))
                return
            self._session.promote['commit'] = True
            self._session.new_version = new_ver
            self._session.old_version = active_ver

            try:
                cov = coverage_snapshot_from_db(version_dir(vroot, new_ver), cfg)
                self._session.coverage_after = {
                    'frames': cov.get('frames'),
                    'cells': cov.get('cells'),
                    'occupied_cells': cov.get('cells'),
                    'yaw_occupancy': cov.get('yaw_occupancy'),
                }
                self._session.validation['coverage_after'] = cov
            except Exception:  # noqa: BLE001
                pass

            self._set_state(
                'COMPLETE',
                (
                    f'build_done {short_version_label(active_ver)}→{short_version_label(new_ver)}'
                    f' stop={self._session.stop_reason or "ok"}'
                    f' map_complete={self._session.map_complete}'
                ),
            )
        finally:
            self._critical = False

    def _should_pause(self) -> bool:
        st = self._phase2c_state or self._phase2c_loc
        if st in ('LOST', 'RECOVERING', 'BOOT_LOCALIZING'):
            return True
        if self._goals_blocked:
            return True
        return False

    def _tally(self, code: str) -> None:
        assert self._session
        if code == 'ACCEPTED':
            self._session.accepted_candidates += 1
        elif code == 'SKIP_DUPLICATE':
            self._session.duplicate_skips += 1
        elif code == 'SKIP_COVERED':
            self._session.covered_skips += 1
        elif code == 'REJECTED_IMAGE':
            self._session.image_rejects += 1
        elif code == 'REJECTED_LASER':
            self._session.laser_rejects += 1
        elif code == 'REJECTED_POSE':
            self._session.pose_rejects += 1
        elif 'QUOTA' in code:
            self._session.quota_skips += 1
        else:
            self._session.other_skips += 1

    def _wait_settle(self, settle_sec: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < settle_sec and not self._stop.is_set():
            if self._odom is not None:
                tw = self._odom.twist.twist
                spd = math.hypot(float(tw.linear.x), float(tw.linear.y))
                yr = abs(float(tw.angular.z))
                if spd < 0.05 and yr < 0.08:
                    st = self._phase2c_state or self._phase2c_loc or 'READY'
                    if st == 'READY' and (self._loc_status in (None, 0)):
                        return
            time.sleep(0.1)

    def _navigate_to(self, goal: PatrolGoal) -> Tuple[bool, str]:
        assert self._session
        if self._session.simulate_nav or bool(self._cfg.get('patrol', {}).get('simulate_nav', False)):
            time.sleep(0.05)
            return True, 'SIMULATED'
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose unavailable')
            return False, 'NAV2_UNAVAILABLE'
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal.x)
        pose.pose.position.y = float(goal.y)
        pose.pose.orientation = _yaw_to_quat(float(goal.yaw))
        ng = NavigateToPose.Goal()
        ng.pose = pose
        send_fut = self._nav_client.send_goal_async(ng)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not send_fut.done():
            if self._stop.is_set():
                return False, 'ABORTED'
            time.sleep(0.05)
        if not send_fut.done():
            return False, 'SEND_TIMEOUT'
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            return False, 'GOAL_REJECTED'
        self._nav_goal_handle = gh
        result_fut = gh.get_result_async()
        timeout = float(self._cfg.get('patrol', {}).get('nav_timeout_sec', 120.0))
        t0 = time.monotonic()
        while not result_fut.done():
            if self._stop.is_set() or self._should_pause():
                try:
                    gh.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                self._nav_goal_handle = None
                return False, 'CANCELLED'
            if time.monotonic() - t0 > timeout:
                try:
                    gh.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                self._nav_goal_handle = None
                return False, 'TIMEOUT'
            time.sleep(0.1)
        self._nav_goal_handle = None
        try:
            wrap = result_fut.result()
            status = int(getattr(wrap, 'status', 0)) if wrap is not None else 0
            # GoalStatus.STATUS_SUCCEEDED == 4
            if status == 4:
                return True, f'SUCCEEDED:{status}'
            return False, f'NAV2_STATUS:{status}'
        except Exception as exc:  # noqa: BLE001
            return False, f'ERROR:{exc}'

    def _capture_at_goal(self, goal: PatrolGoal) -> str:
        assert self._session
        self._capture_result = None
        req = {
            'dry_run': bool(self._session.dry_run),
            'source': 'auto_patrol',
            'build_session_id': self._session.build_session_id,
            'goal': goal.as_dict(),
        }
        self._capture_pub.publish(String(data=json.dumps(req)))
        t0 = time.monotonic()
        timeout = 20.0 if not self._session.simulate_nav else 1.5
        while time.monotonic() - t0 < timeout and not self._stop.is_set():
            if self._capture_result is not None:
                break
            time.sleep(0.05)
        if self._capture_result is None:
            if self._session.simulate_nav:
                return 'SIMULATED_NO_CAPTURE_NODE'
            return 'ERROR'
        status = str(self._capture_result.get('status') or 'ERROR')
        path = self._capture_result.get('path')
        if self._capture_result.get('written') and path:
            try:
                import yaml
                from pathlib import Path as P

                meta_p = P(path) / 'meta.yaml'
                if meta_p.is_file():
                    meta = yaml.safe_load(meta_p.read_text(encoding='utf-8')) or {}
                    meta['build_session_id'] = self._session.build_session_id
                    meta['source'] = 'auto_patrol'
                    meta_p.write_text(yaml.safe_dump(meta, sort_keys=False), encoding='utf-8')
            except Exception:  # noqa: BLE001
                pass
        return status


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualDbBuildOrchestrator()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_build(abort=True)
        except Exception:  # noqa: BLE001
            pass
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
