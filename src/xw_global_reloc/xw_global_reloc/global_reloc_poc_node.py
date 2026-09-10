#!/usr/bin/env python3
"""Phase2B PoC Relocalizer — AMCL handoff gated by allow_amcl_handoff (dev only)."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from xw_global_reloc.acceptance import decide, evaluate_candidate
from xw_global_reloc.geometry import GeometryResult, verify_pnp_rgbd
from xw_global_reloc.laser_refine import refine_candidate_with_laser
from xw_global_reloc.laser_verify import DistanceField, score_scan_at_pose
from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files
from xw_global_reloc.orb_utils import extract_orb, make_orb, unpack_keypoints
from xw_global_reloc.pairing import ImageRingBuffer, pair_nearest
from xw_global_reloc.pose_cluster_accept import (
    ClusterMember,
    absolute_gate_member,
    decide_pose_clusters,
    decision_to_dict,
)
from xw_global_reloc.retrieval import retrieve_topk
from xw_global_reloc.transforms import Pose2D, compose_candidate_base_pose, se3_from_xyz_rpy
from xw_global_reloc.phase2d.version_store import (
    load_visual_db_from_root,
    resolve_active_root,
)

try:
    from xw_interfaces.srv import Relocalize
except ImportError:  # pragma: no cover
    Relocalize = None  # type: ignore


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_AMCL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# result codes
READY = 0
UNKNOWN = 1
REJECTED = 2
AMCL_TIMEOUT = 3
NO_DATA = 4
MAP_HASH_MISMATCH = 5
DRY_RUN_OK = 6


def _yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class GlobalRelocPoc(Node):
    def __init__(self) -> None:
        super().__init__('xw_global_reloc_poc')
        if Relocalize is None:
            raise RuntimeError('xw_interfaces.srv.Relocalize not built')

        self.declare_parameter('maps_dir', '/ros2_ws/maps')
        self.declare_parameter('map_name', 'vp')
        self.declare_parameter('db_root', '')
        self.declare_parameter('depth_scale_m', 0.001)
        self.declare_parameter('max_candidates', 10)
        self.declare_parameter('min_matches', 20)
        self.declare_parameter('min_pnp_inliers', 12)
        self.declare_parameter('min_inlier_ratio', 0.35)
        self.declare_parameter('max_reprojection_error', 4.0)
        self.declare_parameter('min_depth_consistency', 0.55)
        self.declare_parameter('rgb_depth_max_dt_sec', 0.08)
        self.declare_parameter('laser_beam_stride', 6)
        self.declare_parameter('laser_match_dist_m', 0.25)
        self.declare_parameter('min_laser_score', 0.38)
        self.declare_parameter('min_valid_beams', 20)
        self.declare_parameter('min_top_margin', 0.05)
        self.declare_parameter('pipeline_mode', 'visual_laser')  # visual_laser | rgbd_laser
        self.declare_parameter('laser_coarse_xy_m', 1.0)
        self.declare_parameter('laser_coarse_yaw_deg', 30.0)
        self.declare_parameter('laser_coarse_xy_step', 0.10)
        self.declare_parameter('laser_coarse_yaw_step_deg', 3.0)
        self.declare_parameter('laser_fine_xy_m', 0.20)
        self.declare_parameter('laser_fine_yaw_deg', 5.0)
        self.declare_parameter('laser_fine_xy_step', 0.05)
        self.declare_parameter('laser_fine_yaw_step_deg', 1.0)
        self.declare_parameter('laser_min_margin', 0.03)
        self.declare_parameter('laser_reject_local_grid_margin', False)
        self.declare_parameter('laser_max_refine_trans_m', 1.2)
        self.declare_parameter('laser_max_refine_yaw_deg', 35.0)
        self.declare_parameter('cluster_xy_m', 0.25)
        self.declare_parameter('cluster_yaw_deg', 6.0)
        self.declare_parameter('cluster_min_score_margin', 0.03)
        # Phase2B AMCL handoff — default OFF (production-safe). Dev launch sets true.
        self.declare_parameter('allow_amcl_handoff', False)
        self.declare_parameter('amcl_timeout_sec', 12.0)
        self.declare_parameter('amcl_ready_cov_xy', 0.80)
        self.declare_parameter('amcl_ready_cov_yaw', 0.35)
        self.declare_parameter('amcl_ready_stable_sec', 2.5)
        self.declare_parameter('amcl_pose_fresh_sec', 2.0)
        self.declare_parameter('amcl_tf_fresh_sec', 1.5)
        self.declare_parameter('amcl_stable_xy_m', 0.12)
        self.declare_parameter('amcl_stable_yaw_rad', 0.12)
        # Covariance seeded on /initialpose from Visual+Laser PoC accuracy.
        self.declare_parameter('handoff_cov_xy', 0.12)
        self.declare_parameter('handoff_cov_yaw', 0.08)
        # Sensor arming gate (Phase2B1) — no fixed short sleep before retrieval.
        self.declare_parameter('sensor_arm_timeout_sec', 8.0)
        self.declare_parameter('sensor_fresh_sec', 1.0)
        self.declare_parameter('debug_dump_dir', '/ros2_ws/bench/phase2a_poc_v1_2026-09-07/reloc_dumps')

        self._cb = ReentrantCallbackGroup()
        self._bridge = CvBridge()
        # Lazy ORB — constructing ORB at init is fine; keep idle free of sensor work.
        self._orb = None
        self._rgb_buf = ImageRingBuffer(40)
        self._depth_buf = ImageRingBuffer(40)
        self._info: Optional[CameraInfo] = None
        self._scan: Optional[LaserScan] = None
        self._scan_mono: Optional[float] = None
        self._map: Optional[OccupancyGrid] = None
        self._field: Optional[DistanceField] = None
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._amcl_mono: Optional[float] = None
        self._loc_status = 1
        self._keyframes: List[Dict[str, Any]] = []
        self._kf_by_id: Dict[str, Dict[str, Any]] = {}
        # Gate camera decoding — idle reloc must not burn CPU on every RGB/depth frame.
        self._rgb_want = False
        self._arm_mono: Optional[float] = None
        self._first_rgb_mono: Optional[float] = None
        self._first_scan_mono: Optional[float] = None
        # TF only during AMCL wait (TransformListener on /tf is expensive idle).
        self._tf: Optional[Buffer] = None
        self._tf_listener = None
        self._active = False
        self._reloc_in_progress = False
        self._db_version = ''
        self._db_source = ''
        self._db_path: Optional[Path] = None
        self._db_hash = ''
        self._cur_hash = ''

        self._rgb_req = self.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._cmd_stop = self.create_publisher(Twist, '/xw/cmd/motion', 10)
        self._nav_cancel = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self._nomotion = self.create_client(Empty, '/request_nomotion_update', callback_group=self._cb)

        # Heavy / high-rate subs are created only while a reloc call is active.
        self._rgb_sub = None
        self._depth_sub = None
        self._info_sub = None
        self._scan_sub = None
        self._map_sub = None
        self._amcl_sub = None
        self._loc_sub = None
        # localization_status is tiny + latched — keep for diagnostics without load.
        self._loc_sub = self.create_subscription(Int8, '/xw/localization_status', self._on_loc, _LATCH)

        self.create_service(Relocalize, '/xw/relocalize', self._on_relocalize, callback_group=self._cb)
        self.create_service(
            Trigger, '/xw/visual_db/reload', self._on_reload_db, callback_group=self._cb
        )
        self._load_db()
        # Ensure idle release on startup.
        self._request_rgb(False)
        allow = bool(self.get_parameter('allow_amcl_handoff').value)
        self.get_logger().info(
            f'Visual DB Active Version: {self._db_version or "unknown"} '
            f'source={self._db_source} Loaded: {len(self._keyframes)} keyframes '
            f'DB Root: {self._db_path}'
        )
        if self._db_source == 'fallback_legacy':
            self.get_logger().warn('VISUAL_DB_FALLBACK_LEGACY — current_active_version missing')
        self.get_logger().info(
            f'reloc PoC ready db_kfs={len(self._keyframes)} '
            f'allow_amcl_handoff={allow} (IDLE: no RGB/Depth/Scan/map/amcl/TF)'
        )

    def _db_root(self) -> Path:
        maps_dir = Path(str(self.get_parameter('maps_dir').value))
        map_name = str(self.get_parameter('map_name').value)
        db = str(self.get_parameter('db_root').value).strip()
        root, version, source = resolve_active_root(maps_dir, map_name, db_root_override=db)
        self._db_version = version
        self._db_source = source
        self._db_path = root
        return root

    def _load_db(self) -> None:
        """Initial load into empty node state (startup only)."""
        root = self._db_root()
        loaded = load_visual_db_from_root(root)
        if loaded.error:
            self.get_logger().warn(f'Visual DB load failed: {loaded.error} root={root}')
            self._keyframes = []
            self._kf_by_id = {}
            self._db_hash = ''
            return
        maps_dir = Path(str(self.get_parameter('maps_dir').value))
        map_name = str(self.get_parameter('map_name').value)
        try:
            yaml_p, pgm_p = resolve_map_files(maps_dir, map_name)
            self._cur_hash = map_pair_hash(yaml_p, pgm_p)
        except FileNotFoundError:
            self._cur_hash = ''
        self._db_hash = loaded.db_hash
        self._db_version = loaded.version or self._db_version
        self._keyframes = loaded.keyframes
        self._kf_by_id = loaded.kf_by_id
        # Never retain candidate ids
        self._keyframes = [k for k in self._keyframes if not str(k.get('id', '')).startswith('cand_')]
        self._kf_by_id = {k['id']: k for k in self._keyframes}

    def _on_reload_db(self, request, response):  # noqa: ANN001, ARG002
        """Safe Active DB reload: load into temp → validate → atomic swap."""
        t0 = time.monotonic()
        old_version = self._db_version
        old_count = len(self._keyframes)
        payload: Dict[str, Any] = {
            'old_version': old_version,
            'requested_version': None,
            'active_version_after': old_version,
            'loaded_keyframe_count': old_count,
            'elapsed_ms': 0,
        }
        if self._reloc_in_progress or self._active:
            payload['status'] = 'BUSY_REJECTED'
            payload['elapsed_ms'] = int((time.monotonic() - t0) * 1000)
            response.success = False
            response.message = json.dumps(payload, separators=(',', ':'))
            return response

        maps_dir = Path(str(self.get_parameter('maps_dir').value))
        map_name = str(self.get_parameter('map_name').value)
        db = str(self.get_parameter('db_root').value).strip()
        root, version, source = resolve_active_root(maps_dir, map_name, db_root_override=db)
        payload['requested_version'] = version
        if source == 'missing':
            payload['status'] = 'VERSION_NOT_FOUND'
            payload['elapsed_ms'] = int((time.monotonic() - t0) * 1000)
            response.success = False
            response.message = json.dumps(payload, separators=(',', ':'))
            return response

        loaded = load_visual_db_from_root(root)
        if loaded.error:
            payload['status'] = loaded.error
            payload['elapsed_ms'] = int((time.monotonic() - t0) * 1000)
            response.success = False
            response.message = json.dumps(payload, separators=(',', ':'))
            return response

        try:
            yaml_p, pgm_p = resolve_map_files(maps_dir, map_name)
            cur_hash = map_pair_hash(yaml_p, pgm_p)
        except FileNotFoundError:
            cur_hash = ''
        if loaded.db_hash and cur_hash and loaded.db_hash != cur_hash:
            payload['status'] = 'MAP_HASH_MISMATCH'
            payload['db_hash'] = loaded.db_hash
            payload['cur_hash'] = cur_hash
            payload['elapsed_ms'] = int((time.monotonic() - t0) * 1000)
            response.success = False
            response.message = json.dumps(payload, separators=(',', ':'))
            return response

        # Atomic in-memory swap only after full successful load
        self._keyframes = [k for k in loaded.keyframes if not str(k.get('id', '')).startswith('cand_')]
        self._kf_by_id = {k['id']: k for k in self._keyframes}
        self._db_hash = loaded.db_hash
        self._cur_hash = cur_hash
        self._db_version = loaded.version or version
        self._db_source = source
        self._db_path = root

        payload['status'] = 'RELOAD_OK'
        payload['active_version_after'] = self._db_version
        payload['loaded_keyframe_count'] = len(self._keyframes)
        payload['db_root'] = str(root)
        payload['elapsed_ms'] = int((time.monotonic() - t0) * 1000)
        self.get_logger().info(
            f'Visual DB reload OK version={self._db_version} kfs={len(self._keyframes)} '
            f'root={root} ({payload["elapsed_ms"]} ms)'
        )
        response.success = True
        response.message = json.dumps(payload, separators=(',', ':'))
        return response

    def _on_rgb(self, msg: Image) -> None:
        if self._rgb_want:
            self._rgb_buf.push(msg)
            if self._first_rgb_mono is None:
                self._first_rgb_mono = time.monotonic()

    def _on_depth(self, msg: Image) -> None:
        if self._rgb_want:
            self._depth_buf.push(msg)

    def _on_info(self, msg: CameraInfo) -> None:
        self._info = msg

    def _on_scan(self, msg: LaserScan) -> None:
        if self._rgb_want:
            self._scan = msg
            self._scan_mono = time.monotonic()
            if self._first_scan_mono is None:
                self._first_scan_mono = time.monotonic()

    def _ensure_orb(self):
        if self._orb is None:
            self._orb = make_orb(1000)
        return self._orb

    def _destroy_sub(self, attr: str) -> None:
        sub = getattr(self, attr, None)
        if sub is None:
            return
        try:
            self.destroy_subscription(sub)
        except Exception:  # noqa: BLE001
            pass
        setattr(self, attr, None)

    def _arm_sensors(self, on: bool) -> None:
        """Subscribe/unsubscribe heavy topics. Idle must not deserialize Image/Scan/map/amcl."""
        self._rgb_want = bool(on)
        self._active = bool(on)
        mode = str(self.get_parameter('pipeline_mode').value)
        if on:
            self._arm_mono = time.monotonic()
            self._first_rgb_mono = None
            self._first_scan_mono = None
            self._scan = None
            self._scan_mono = None
            self._rgb_buf = ImageRingBuffer(40)
            self._depth_buf = ImageRingBuffer(40)
            if self._map_sub is None:
                self._map_sub = self.create_subscription(
                    OccupancyGrid, '/map', self._on_map, _MAP_QOS
                )
            if self._amcl_sub is None:
                self._amcl_sub = self.create_subscription(
                    PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, _AMCL_QOS
                )
            if self._rgb_sub is None:
                self._rgb_sub = self.create_subscription(
                    Image, '/camera/front_up/color/image_raw', self._on_rgb, _SENSOR_QOS
                )
            # Depth only for rgbd_laser mode — visual_laser must not pay Depth bandwidth.
            if mode == 'rgbd_laser' and self._depth_sub is None:
                self._depth_sub = self.create_subscription(
                    Image, '/camera/front_up/depth/image_raw', self._on_depth, _SENSOR_QOS
                )
            if self._info_sub is None:
                self._info_sub = self.create_subscription(
                    CameraInfo, '/camera/front_up/color/camera_info', self._on_info, _SENSOR_QOS
                )
            if self._scan_sub is None:
                self._scan_sub = self.create_subscription(
                    LaserScan, '/scan', self._on_scan, _SENSOR_QOS
                )
            self._rgb_req.publish(Bool(data=True))
        else:
            self._rgb_req.publish(Bool(data=False))
            # Drop TF first (owns /tf subscription).
            self._arm_tf(False)
            for attr in (
                '_rgb_sub',
                '_depth_sub',
                '_info_sub',
                '_scan_sub',
                '_map_sub',
                '_amcl_sub',
            ):
                self._destroy_sub(attr)
            self._rgb_buf = ImageRingBuffer(40)
            self._depth_buf = ImageRingBuffer(40)
            self._scan = None
            self._scan_mono = None
            self._arm_mono = None
            self._first_rgb_mono = None
            self._first_scan_mono = None
            # Keep last map field in memory (no ongoing sub). Clear amcl cache.
            self._amcl = None
            self._amcl_mono = None

    def _request_rgb(self, on: bool) -> None:
        self._arm_sensors(bool(on))

    def _on_map(self, msg: OccupancyGrid) -> None:
        # Build distance field once per arming; ignore repeats while armed.
        if self._field is not None and self._map is not None:
            return
        self._map = msg
        self._field = DistanceField(msg)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._active and self._tf is None:
            return
        self._amcl = msg
        self._amcl_mono = time.monotonic()

    def _on_loc(self, msg: Int8) -> None:
        self._loc_status = int(msg.data)

    def _mono_now(self) -> float:
        return time.monotonic()

    @staticmethod
    def _yaw_from_quat(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _stop_robot(self) -> None:
        """Stop motion before seeding AMCL. Does not call reinitialize_global_localization."""
        self._nav_cancel.publish(Bool(data=True))
        twist = Twist()
        for _ in range(3):
            self._cmd_stop.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)

    def _arm_tf(self, on: bool) -> None:
        if on:
            if self._tf is None:
                self._tf = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
                self._tf_listener = TransformListener(self._tf, self, spin_thread=False)
            # Need amcl during READY wait even if sensors already disarmed.
            if self._amcl_sub is None:
                self._amcl_sub = self.create_subscription(
                    PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, _AMCL_QOS
                )
        else:
            listener = self._tf_listener
            self._tf_listener = None
            self._tf = None
            if listener is not None:
                # TransformListener does not always auto-destroy node subscriptions.
                for attr in (
                    'tf_sub',
                    'tf_static_sub',
                    'subscription',
                    '_subscription',
                    'tf_sub_',
                    'tf_static_sub_',
                ):
                    sub = getattr(listener, attr, None)
                    if sub is not None:
                        try:
                            self.destroy_subscription(sub)
                        except Exception:  # noqa: BLE001
                            pass
                del listener
            # If not in full sensor-arm session, drop amcl too.
            if not self._rgb_want:
                self._destroy_sub('_amcl_sub')
                self._amcl = None
                self._amcl_mono = None

    def _release_idle(self) -> None:
        """Force IDLE resource profile: no heavy subs / TF / RGB request."""
        try:
            self._rgb_want = False
            self._active = False
            self._arm_tf(False)
            self._rgb_req.publish(Bool(data=False))
            for attr in (
                '_rgb_sub',
                '_depth_sub',
                '_info_sub',
                '_scan_sub',
                '_map_sub',
                '_amcl_sub',
            ):
                self._destroy_sub(attr)
            self._rgb_buf = ImageRingBuffer(40)
            self._depth_buf = ImageRingBuffer(40)
            self._scan = None
            self._scan_mono = None
            self._arm_mono = None
            self._first_rgb_mono = None
            self._first_scan_mono = None
            self._amcl = None
            self._amcl_mono = None
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'release_idle failed: {exc}')

    def _map_odom_fresh(self) -> Tuple[bool, Optional[float]]:
        """map→odom must exist; stamp age is advisory (AMCL may be quiet while static)."""
        if self._tf is None:
            return False, None
        stale = float(self.get_parameter('amcl_tf_fresh_sec').value)
        try:
            tf = self._tf.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            if tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0:
                return True, 0.0
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            _ = stale
            return True, float(age)
        except TransformException:
            return False, None

    def _amcl_pose_tuple(self) -> Optional[Tuple[float, float, float]]:
        if self._amcl is None:
            return None
        p = self._amcl.pose.pose
        return (float(p.position.x), float(p.position.y), self._yaw_from_quat(p.orientation))

    def _wait_amcl_ready(
        self, candidate: Pose2D, amcl_mono_before: Optional[float]
    ) -> Tuple[bool, Dict[str, Any]]:
        """READY = post-seed AMCL update + fresh map→odom + cov OK + stable window.

        Never calls reinitialize_global_localization.
        """
        timeout = float(self.get_parameter('amcl_timeout_sec').value)
        cov_xy_lim = float(self.get_parameter('amcl_ready_cov_xy').value)
        cov_yaw_lim = float(self.get_parameter('amcl_ready_cov_yaw').value)
        stable_need = float(self.get_parameter('amcl_ready_stable_sec').value)
        fresh_lim = float(self.get_parameter('amcl_pose_fresh_sec').value)
        jump_xy = float(self.get_parameter('amcl_stable_xy_m').value)
        jump_yaw = float(self.get_parameter('amcl_stable_yaw_rad').value)

        t0 = self._mono_now()
        deadline = t0 + timeout
        stable_since: Optional[float] = None
        anchor: Optional[Tuple[float, float, float]] = None
        last_diag: Dict[str, Any] = {}
        nomotion_pulses = 0
        saw_post_seed = False

        while self._mono_now() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            # Periodic nomotion while stopped to force AMCL update without global reinit.
            if nomotion_pulses < 4 and self._nomotion.service_is_ready():
                if (self._mono_now() - t0) >= 0.35 * nomotion_pulses:
                    try:
                        self._nomotion.call_async(Empty.Request())
                        nomotion_pulses += 1
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().warn(f'nomotion pulse failed: {exc}')

            if self._amcl is None or self._amcl_mono is None:
                stable_since = None
                continue
            if amcl_mono_before is not None and self._amcl_mono <= amcl_mono_before:
                continue
            saw_post_seed = True

            age = self._mono_now() - self._amcl_mono
            # One post-seed update is enough; AMCL may stay quiet while static.
            pose_fresh = age <= max(fresh_lim, 30.0)
            tf_ok, tf_age = self._map_odom_fresh()
            c = self._amcl.pose.covariance
            cov_xy = max(float(c[0]), float(c[7]))
            cov_yaw = float(c[35])
            pose = self._amcl_pose_tuple()
            assert pose is not None
            dx = math.hypot(pose[0] - candidate.x, pose[1] - candidate.y)
            dyaw = abs(math.atan2(math.sin(pose[2] - candidate.yaw), math.cos(pose[2] - candidate.yaw)))

            cov_ok = cov_xy <= cov_xy_lim and cov_yaw <= cov_yaw_lim
            # Candidate agreement: reject wild particle jumps after seed.
            near_candidate = dx <= 1.0 and dyaw <= 0.60
            status_ok = self._loc_status == 0
            gates_ok = (
                pose_fresh
                and tf_ok
                and cov_ok
                and near_candidate
                and (status_ok or (cov_xy < 0.5 and cov_yaw < 0.25))
            )

            if gates_ok:
                if anchor is None:
                    anchor = pose
                    stable_since = self._mono_now()
                else:
                    dxy = math.hypot(pose[0] - anchor[0], pose[1] - anchor[1])
                    dy = abs(math.atan2(math.sin(pose[2] - anchor[2]), math.cos(pose[2] - anchor[2])))
                    if dxy > jump_xy or dy > jump_yaw:
                        anchor = pose
                        stable_since = self._mono_now()
                stable_for = (self._mono_now() - stable_since) if stable_since else 0.0
            else:
                stable_since = None
                anchor = None
                stable_for = 0.0

            last_diag = {
                'amcl_pose': pose,
                'amcl_age_sec': age,
                'map_odom_ok': tf_ok,
                'map_odom_age_sec': tf_age,
                'cov_xy': cov_xy,
                'cov_yaw': cov_yaw,
                'loc_status': self._loc_status,
                'candidate_delta_xy': dx,
                'candidate_delta_yaw': dyaw,
                'near_candidate': near_candidate,
                'stable_for_sec': stable_for,
                'gates_ok': gates_ok,
                'saw_post_seed_amcl': saw_post_seed,
            }
            if gates_ok and stable_for >= stable_need:
                last_diag['localization_ready'] = True
                last_diag['amcl_convergence_sec'] = self._mono_now() - t0
                return True, last_diag

        if not last_diag:
            last_diag = {'saw_post_seed_amcl': saw_post_seed}
        last_diag['localization_ready'] = False
        last_diag['amcl_convergence_sec'] = self._mono_now() - t0
        return False, last_diag

    def _wait_sensors_ready(self, timeout: Optional[float] = None) -> Tuple[Optional[Image], Dict[str, Any]]:
        """Explicit Sensor Ready Gate: new RGB + new Scan + CameraInfo + map after arm."""
        if timeout is None:
            timeout = float(self.get_parameter('sensor_arm_timeout_sec').value)
        fresh = float(self.get_parameter('sensor_fresh_sec').value)
        t0 = self._arm_mono or time.monotonic()
        deadline = t0 + float(timeout)
        metrics: Dict[str, Any] = {
            'sensor_arm_timeout_sec': float(timeout),
            'sensor_fresh_sec': fresh,
        }
        rgb_msg = None
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if self._first_rgb_mono is not None and 'rgb_request_to_first_rgb_ms' not in metrics:
                metrics['rgb_request_to_first_rgb_ms'] = (self._first_rgb_mono - t0) * 1000.0
            if self._first_scan_mono is not None and 'scan_wait_ms' not in metrics:
                metrics['scan_wait_ms'] = (self._first_scan_mono - t0) * 1000.0
            if len(self._rgb_buf) > 0:
                rgb_msg = self._rgb_buf._buf[-1].msg
            # Ready = post-arm RGB frame + post-arm scan still fresh + camera_info + map.
            rgb_ok = rgb_msg is not None and self._first_rgb_mono is not None
            scan_ok = (
                self._scan is not None
                and self._first_scan_mono is not None
                and self._scan_mono is not None
                and (now - self._scan_mono) <= max(fresh, 2.0)
            )
            info_ok = self._info is not None
            map_ok = self._field is not None
            if rgb_ok and scan_ok and info_ok and map_ok:
                metrics['sensor_ready_ms'] = (now - t0) * 1000.0
                metrics['sensor_ready'] = True
                metrics['rgb'] = True
                metrics['scan'] = True
                metrics['camera_info'] = True
                metrics['map'] = True
                return rgb_msg, metrics
        metrics['sensor_ready'] = False
        metrics['sensor_ready_ms'] = (time.monotonic() - t0) * 1000.0
        metrics['rgb'] = len(self._rgb_buf) > 0
        metrics['scan'] = self._scan is not None
        metrics['camera_info'] = self._info is not None
        metrics['map'] = self._field is not None
        if self._first_rgb_mono is not None and 'rgb_request_to_first_rgb_ms' not in metrics:
            metrics['rgb_request_to_first_rgb_ms'] = (self._first_rgb_mono - t0) * 1000.0
        if self._first_scan_mono is not None and 'scan_wait_ms' not in metrics:
            metrics['scan_wait_ms'] = (self._first_scan_mono - t0) * 1000.0
        return None, metrics

    def _wait_rgb(self, timeout: float = 5.0):
        # Legacy helper; prefer _wait_sensors_ready.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._info is None:
                continue
            if len(self._rgb_buf) > 0:
                return self._rgb_buf._buf[-1].msg
        return None

    def _wait_pair(self, timeout: float = 5.0):
        deadline = time.monotonic() + timeout
        max_dt = float(self.get_parameter('rgb_depth_max_dt_sec').value)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._info is None:
                continue
            pair = pair_nearest(self._rgb_buf, self._depth_buf, prefer='rgb')
            if pair is not None and pair.pair_dt_sec <= max_dt:
                return pair
        return None

    def _dump_failure(self, bgr, depth, topk, scores: dict) -> None:
        out = Path(str(self.get_parameter('debug_dump_dir').value))
        out.mkdir(parents=True, exist_ok=True)
        stamp = int(time.time())
        d = out / f'fail_{stamp}'
        d.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(d / 'query_rgb.jpg'), bgr)
        cv2.imwrite(str(d / 'query_depth.png'), depth)
        (d / 'topk.json').write_text(json.dumps(topk, indent=2), encoding='utf-8')
        (d / 'scores.json').write_text(json.dumps(scores, indent=2), encoding='utf-8')

    def _on_relocalize(self, req, res):
        timings: Dict[str, float] = {}
        t_all = time.monotonic()
        self._reloc_in_progress = True
        try:
            return self._on_relocalize_impl(req, res, timings, t_all)
        finally:
            self._reloc_in_progress = False
            self._release_idle()

    def _on_relocalize_impl(self, req, res, timings: Dict[str, float], t_all: float):
        apply_pose = bool(req.apply_initial_pose)
        allow_handoff = bool(self.get_parameter('allow_amcl_handoff').value)
        # Phase2B: apply only when dev/explicit allow_amcl_handoff=true.
        # Production robot.launch.py does not start this node; yaml default remains false.
        if apply_pose and not allow_handoff:
            self.get_logger().warn(
                'apply_initial_pose=true ignored (allow_amcl_handoff=false; production-safe)'
            )
            apply_pose = False
        force_visual = bool(req.force_visual) if req.force_visual is not None else True
        # PoC V1: force visual path (ignore charger / last_good)
        _ = force_visual
        max_k = int(req.max_candidates) if req.max_candidates else int(self.get_parameter('max_candidates').value)

        res.success = False
        res.result_code = UNKNOWN
        res.candidate_count = 0
        res.visual_score = 0.0
        res.geometry_score = 0.0
        res.laser_score = 0.0
        res.composite_score = 0.0
        res.confidence = 0.0
        res.time_to_candidate_sec = 0.0
        res.amcl_convergence_sec = 0.0

        if self._db_hash and self._cur_hash and self._db_hash != self._cur_hash:
            res.result_code = MAP_HASH_MISMATCH
            res.diagnostics_json = json.dumps({'db': self._db_hash, 'cur': self._cur_hash})
            res.stage_timings_json = '{}'
            return res
        if not self._keyframes:
            res.result_code = NO_DATA
            res.diagnostics_json = '{"error":"empty_keyframe_db"}'
            res.stage_timings_json = '{}'
            return res

        try:
            self._request_rgb(True)
            mode = str(self.get_parameter('pipeline_mode').value)
            t0 = time.monotonic()
            rgb_msg, sensor_metrics = self._wait_sensors_ready()
            timings['sensor_ready'] = time.monotonic() - t0
            timings.update({k: v for k, v in sensor_metrics.items() if isinstance(v, (int, float, bool))})
            # Depth optional for visual_laser; still try nearest pair for diagnostics.
            pair = pair_nearest(self._rgb_buf, self._depth_buf, prefer='rgb')
            if rgb_msg is None or self._scan is None or self._field is None:
                res.result_code = NO_DATA
                res.diagnostics_json = json.dumps(
                    {
                        'error': 'sensor_not_ready',
                        'rgb': rgb_msg is not None,
                        'scan': self._scan is not None,
                        'map': self._field is not None,
                        'camera_info': self._info is not None,
                        'mode': mode,
                        'sensor_gate': sensor_metrics,
                    }
                )
                res.stage_timings_json = json.dumps(timings)
                return res

            bgr = self._bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_u16 = None
            if pair is not None:
                try:
                    depth_u16 = self._bridge.imgmsg_to_cv2(pair.depth, 'passthrough')
                    timings['pair_dt_ms'] = pair.pair_dt_sec * 1000.0
                except Exception:  # noqa: BLE001
                    depth_u16 = None

            t0 = time.monotonic()
            qorb = extract_orb(bgr, self._ensure_orb())
            timings['orb'] = time.monotonic() - t0

            retrieval_pool = [k for k in self._keyframes if k.get('retrieval_ready', True)]
            t0 = time.monotonic()
            retr = retrieve_topk(bgr, retrieval_pool, top_k=max_k)
            timings['retrieval'] = time.monotonic() - t0

            import math as _math

            topk_dbg = []
            members: list = []
            t_laser = 0.0
            for cand in retr.candidates:
                kf = self._kf_by_id.get(cand.keyframe_id)
                if kf is None:
                    continue
                mp = kf['meta']['map_pose']
                seed = Pose2D(float(mp['x']), float(mp['y']), float(mp['yaw']))
                tl = time.monotonic()
                refined = refine_candidate_with_laser(
                    self._field,
                    self._scan,
                    seed,
                    coarse_xy_m=float(self.get_parameter('laser_coarse_xy_m').value),
                    coarse_yaw_rad=_math.radians(float(self.get_parameter('laser_coarse_yaw_deg').value)),
                    coarse_xy_step=float(self.get_parameter('laser_coarse_xy_step').value),
                    coarse_yaw_step=_math.radians(float(self.get_parameter('laser_coarse_yaw_step_deg').value)),
                    fine_xy_m=float(self.get_parameter('laser_fine_xy_m').value),
                    fine_yaw_rad=_math.radians(float(self.get_parameter('laser_fine_yaw_deg').value)),
                    fine_xy_step=float(self.get_parameter('laser_fine_xy_step').value),
                    fine_yaw_step=_math.radians(float(self.get_parameter('laser_fine_yaw_step_deg').value)),
                    beam_stride=int(self.get_parameter('laser_beam_stride').value),
                    match_dist_m=float(self.get_parameter('laser_match_dist_m').value),
                    min_valid_beams=int(self.get_parameter('min_valid_beams').value),
                    min_laser_score=float(self.get_parameter('min_laser_score').value),
                    min_margin=float(self.get_parameter('laser_min_margin').value),
                    max_refine_trans_m=float(self.get_parameter('laser_max_refine_trans_m').value),
                    max_refine_yaw_rad=_math.radians(float(self.get_parameter('laser_max_refine_yaw_deg').value)),
                    reject_local_grid_margin=bool(
                        self.get_parameter('laser_reject_local_grid_margin').value
                    ),
                )
                t_laser += time.monotonic() - tl
                seed_t = seed.as_tuple()
                refined_t = refined.refined.as_tuple()
                abs_ok, abs_reason, dx, dy, dyaw = absolute_gate_member(
                    refined=refined_t,
                    seed=seed_t,
                    laser_score=refined.top1_score,
                    min_laser_score=float(self.get_parameter('min_laser_score').value),
                    max_refine_trans_m=float(self.get_parameter('laser_max_refine_trans_m').value),
                    max_refine_yaw_rad=_math.radians(
                        float(self.get_parameter('laser_max_refine_yaw_deg').value)
                    ),
                    free_space=self._field.is_free(refined.refined.x, refined.refined.y),
                    valid_beams=refined.score.valid_beams,
                    min_valid_beams=int(self.get_parameter('min_valid_beams').value),
                    legacy_reason=refined.reason,
                )
                if not refined.accepted and refined.reason != 'ambiguous_margin':
                    abs_ok = False
                    abs_reason = refined.reason
                region = str((kf.get('meta') or {}).get('region_id') or '')
                entry = {
                    'id': cand.keyframe_id,
                    'retrieval_rank': cand.rank,
                    'retrieval_score': cand.score,
                    'ratio_matches': cand.ratio_matches,
                    'visual_region': region,
                    'seed': seed_t,
                    'refined': refined_t,
                    'laser_ok': abs_ok,
                    'laser_reason': abs_reason,
                    'laser_score': refined.top1_score,
                    'laser_local_grid_margin': refined.margin,
                    'valid_beams': refined.score.valid_beams,
                    'matched_ratio': refined.score.matched_ratio,
                    'mean_dist': refined.score.mean_dist,
                    'p90_dist': refined.score.p90_dist,
                    'dx': dx,
                    'dy': dy,
                    'dyaw': dyaw,
                    'refine_runtime': refined.runtime_sec,
                }
                topk_dbg.append(entry)
                members.append(
                    ClusterMember(
                        keyframe_id=cand.keyframe_id,
                        refined=refined_t,
                        seed=seed_t,
                        laser_score=float(refined.top1_score),
                        visual_rank=int(cand.rank),
                        visual_score=float(cand.score),
                        visual_region=region,
                        dx=dx,
                        dy=dy,
                        dyaw=dyaw,
                        valid_beams=refined.score.valid_beams,
                        matched_ratio=refined.score.matched_ratio,
                        mean_dist=refined.score.mean_dist,
                        p90_dist=refined.score.p90_dist,
                        absolute_ok=abs_ok,
                        absolute_reason=abs_reason,
                    )
                )

            timings['laser_total'] = t_laser
            timings['total'] = time.monotonic() - t_all
            res.candidate_count = len(retr.candidates)
            res.time_to_candidate_sec = float(timings['total'])
            res.stage_timings_json = json.dumps(timings)

            dec = decide_pose_clusters(
                members,
                cluster_xy_m=float(self.get_parameter('cluster_xy_m').value),
                cluster_yaw_rad=_math.radians(float(self.get_parameter('cluster_yaw_deg').value)),
                cluster_min_score_margin=float(self.get_parameter('cluster_min_score_margin').value),
            )
            cluster_dbg = decision_to_dict(dec)
            decision_status = dec.status
            decision_reason = dec.reason
            best_cand = None
            best_ref_pose = None
            margin = float(dec.cluster_margin)
            if dec.best_cluster is not None:
                best_ref_pose = Pose2D(*dec.best_cluster.center)
                best_id = dec.best_cluster.members[0].keyframe_id
                best_cand = next((c for c in retr.candidates if c.keyframe_id == best_id), None)

            if decision_status != 'ACCEPT' or best_ref_pose is None or best_cand is None:
                dump_depth = depth_u16 if depth_u16 is not None else np.zeros((480, 640), np.uint16)
                self._dump_failure(
                    bgr,
                    dump_depth,
                    topk_dbg,
                    {
                        'decision': decision_status,
                        'reason': decision_reason,
                        'mode': mode,
                        'cluster_accept': cluster_dbg,
                    },
                )
                self._request_rgb(False)
                res.result_code = UNKNOWN
                res.diagnostics_json = json.dumps(
                    {
                        'decision': decision_status,
                        'reason': decision_reason,
                        'pipeline_mode': mode,
                        'accept_policy': 'pose_cluster_v1',
                        'query_features': retr.query_features,
                        'topk': topk_dbg,
                        'cluster_accept': cluster_dbg,
                        'note': 'Visual proposes; Laser absolute gates + SE(2) clusters decide.',
                    }
                )
                return res

            res.visual_score = float(best_cand.score)
            res.geometry_score = 0.0  # not used in visual_laser V1
            res.laser_score = float(dec.best_cluster.best_laser_score)
            res.composite_score = float(0.4 * best_cand.score + 0.6 * res.laser_score)
            res.confidence = float(res.composite_score)
            cov_xy = float(self.get_parameter('handoff_cov_xy').value)
            cov_yaw = float(self.get_parameter('handoff_cov_yaw').value)
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose.position.x = best_ref_pose.x
            pose_msg.pose.pose.position.y = best_ref_pose.y
            pose_msg.pose.pose.orientation = _yaw_to_quat(best_ref_pose.yaw)
            pose_msg.pose.covariance[0] = cov_xy
            pose_msg.pose.covariance[7] = cov_xy
            pose_msg.pose.covariance[35] = cov_yaw
            res.pose = pose_msg
            diag_base = {
                'decision': 'ACCEPT',
                'reason': decision_reason,
                'pipeline_mode': mode,
                'accept_policy': 'pose_cluster_v1',
                'keyframe_id': best_cand.keyframe_id,
                'visual_rank': best_cand.rank,
                'cluster_margin': margin,
                'cluster_support': dec.best_cluster.support_count,
                'refined': best_ref_pose.as_tuple(),
                'laser_score': float(res.laser_score),
                'apply_initial_pose': apply_pose,
                'allow_amcl_handoff': allow_handoff,
                'handoff_cov_xy': cov_xy,
                'handoff_cov_yaw': cov_yaw,
                'sensor_gate': sensor_metrics,
                'topk': topk_dbg,
                'cluster_accept': cluster_dbg,
            }
            res.diagnostics_json = json.dumps(diag_base)

            if not apply_pose:
                self._request_rgb(False)
                res.success = True
                res.result_code = DRY_RUN_OK
                return res

            # ACCEPT-only handoff. UNKNOWN/REJECTED never reach here.
            # Do NOT call reinitialize_global_localization (would wipe Visual+Laser prior).
            if bool(req.allow_motion):
                self.get_logger().warn('allow_motion ignored in Phase2B handoff (no auto spin)')
            self._stop_robot()
            time.sleep(0.15)
            amcl_mono_before = self._amcl_mono
            # Free camera/scan DDS while waiting on AMCL.
            self._request_rgb(False)
            self._arm_tf(True)
            self._initialpose_pub.publish(pose_msg)
            if self._nomotion.service_is_ready():
                try:
                    self._nomotion.call_async(Empty.Request())
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'nomotion failed: {exc}')
            else:
                self.get_logger().warn('request_nomotion_update not ready')

            ready, amcl_diag = self._wait_amcl_ready(best_ref_pose, amcl_mono_before)
            self._arm_tf(False)
            res.amcl_convergence_sec = float(amcl_diag.get('amcl_convergence_sec') or 0.0)
            amcl_pose = amcl_diag.get('amcl_pose')
            cand_t = best_ref_pose.as_tuple()
            correction = None
            if amcl_pose is not None:
                correction = {
                    'dxy': math.hypot(amcl_pose[0] - cand_t[0], amcl_pose[1] - cand_t[1]),
                    'dyaw': abs(
                        math.atan2(
                            math.sin(amcl_pose[2] - cand_t[2]),
                            math.cos(amcl_pose[2] - cand_t[2]),
                        )
                    ),
                }
            diag_base['amcl_handoff'] = amcl_diag
            diag_base['candidate_to_amcl_correction'] = correction
            diag_base['localization_ready'] = bool(ready)
            res.diagnostics_json = json.dumps(diag_base)
            self._request_rgb(False)
            if ready:
                res.success = True
                res.result_code = READY
                self.get_logger().info(
                    f'LOCALIZATION_READY amcl={amcl_pose} conv={res.amcl_convergence_sec:.2f}s'
                )
            else:
                res.success = False
                res.result_code = AMCL_TIMEOUT
                self.get_logger().warn(
                    f'AMCL_TIMEOUT after handoff diag={json.dumps(amcl_diag, default=str)[:400]}'
                )
            return res

        finally:
            self._release_idle()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalRelocPoc()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._release_idle()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
