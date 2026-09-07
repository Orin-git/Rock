#!/usr/bin/env python3
"""Phase2A PoC Relocalizer — dry-run default (apply_initial_pose=false)."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Empty

from xw_global_reloc.acceptance import decide, evaluate_candidate
from xw_global_reloc.geometry import GeometryResult, verify_pnp_rgbd
from xw_global_reloc.laser_refine import refine_candidate_with_laser
from xw_global_reloc.laser_verify import DistanceField, score_scan_at_pose
from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files
from xw_global_reloc.orb_utils import extract_orb, make_orb, unpack_keypoints
from xw_global_reloc.pairing import ImageRingBuffer, pair_nearest
from xw_global_reloc.retrieval import retrieve_topk
from xw_global_reloc.transforms import Pose2D, compose_candidate_base_pose, se3_from_xyz_rpy

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
        self.declare_parameter('min_laser_score', 0.45)
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
        self.declare_parameter('laser_max_refine_trans_m', 1.2)
        self.declare_parameter('laser_max_refine_yaw_deg', 35.0)
        self.declare_parameter('amcl_timeout_sec', 8.0)
        self.declare_parameter('debug_dump_dir', '/ros2_ws/bench/phase2a_poc_v1_2026-09-07/reloc_dumps')

        self._cb = ReentrantCallbackGroup()
        self._bridge = CvBridge()
        self._orb = make_orb(1000)
        self._rgb_buf = ImageRingBuffer(40)
        self._depth_buf = ImageRingBuffer(40)
        self._info: Optional[CameraInfo] = None
        self._scan: Optional[LaserScan] = None
        self._map: Optional[OccupancyGrid] = None
        self._field: Optional[DistanceField] = None
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._loc_status = 1
        self._keyframes: List[Dict[str, Any]] = []
        self._kf_by_id: Dict[str, Dict[str, Any]] = {}

        self._rgb_req = self.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._nomotion = self.create_client(Empty, '/request_nomotion_update', callback_group=self._cb)

        self.create_subscription(Image, '/camera/front_up/color/image_raw', self._on_rgb, _SENSOR_QOS)
        self.create_subscription(Image, '/camera/front_up/depth/image_raw', self._on_depth, _SENSOR_QOS)
        self.create_subscription(CameraInfo, '/camera/front_up/color/camera_info', self._on_info, _SENSOR_QOS)
        self.create_subscription(LaserScan, '/scan', self._on_scan, _SENSOR_QOS)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, _MAP_QOS)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10)
        self.create_subscription(Int8, '/xw/localization_status', self._on_loc, _LATCH)

        self.create_service(Relocalize, '/xw/relocalize', self._on_relocalize, callback_group=self._cb)
        self._load_db()
        self.get_logger().info(
            f'reloc PoC ready db_kfs={len(self._keyframes)} '
            f'(default apply_initial_pose=false / force_visual)'
        )

    def _db_root(self) -> Path:
        maps_dir = Path(str(self.get_parameter('maps_dir').value))
        map_name = str(self.get_parameter('map_name').value)
        db = str(self.get_parameter('db_root').value).strip()
        return Path(db) if db else (maps_dir / map_name / 'visual')

    def _load_db(self) -> None:
        root = self._db_root()
        man = root / 'manifest.yaml'
        if not man.is_file():
            self.get_logger().warn(f'no manifest at {man}')
            return
        manifest = yaml.safe_load(man.read_text(encoding='utf-8')) or {}
        maps_dir = Path(str(self.get_parameter('maps_dir').value))
        map_name = str(self.get_parameter('map_name').value)
        try:
            yaml_p, pgm_p = resolve_map_files(maps_dir, map_name)
            cur = map_pair_hash(yaml_p, pgm_p)
        except FileNotFoundError:
            cur = ''
        self._db_hash = str(manifest.get('map_hash') or '')
        self._cur_hash = cur
        idx_path = root / 'descriptors' / 'index.json'
        if not idx_path.is_file():
            return
        idx = json.loads(idx_path.read_text(encoding='utf-8'))
        for item in idx:
            kid = item['id']
            kdir = root / 'keyframes' / kid
            desc_p = kdir / 'descriptors.npy'
            kp_p = kdir / 'keypoints.npy'
            meta_p = kdir / 'meta.yaml'
            if not (desc_p.is_file() and kp_p.is_file() and meta_p.is_file()):
                continue
            desc = np.load(str(desc_p))
            kps = unpack_keypoints(np.load(str(kp_p)))
            meta = yaml.safe_load(meta_p.read_text(encoding='utf-8')) or {}
            depth = cv2.imread(str(kdir / 'depth.png'), cv2.IMREAD_UNCHANGED)
            entry = {
                'id': kid,
                'descriptors': desc,
                'keypoints': kps,
                'meta': meta,
                'depth': depth,
                'retrieval_ready': bool(meta.get('retrieval_ready', True)),
                'geometry_ready': bool(meta.get('geometry_ready', depth is not None)),
                'dir': kdir,
            }
            self._keyframes.append(entry)
            self._kf_by_id[kid] = entry

    def _on_rgb(self, msg: Image) -> None:
        self._rgb_buf.push(msg)

    def _on_depth(self, msg: Image) -> None:
        self._depth_buf.push(msg)

    def _on_info(self, msg: CameraInfo) -> None:
        self._info = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        self._field = DistanceField(msg)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg

    def _on_loc(self, msg: Int8) -> None:
        self._loc_status = int(msg.data)

    def _request_rgb(self, on: bool) -> None:
        self._rgb_req.publish(Bool(data=bool(on)))

    def _wait_rgb(self, timeout: float = 5.0):
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
        apply_pose = bool(req.apply_initial_pose)
        # PoC V1.1 data qualification: AMCL apply remains forbidden.
        if apply_pose:
            self.get_logger().warn('apply_initial_pose=true ignored (V1.1 gate)')
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

        self._request_rgb(True)
        mode = str(self.get_parameter('pipeline_mode').value)
        t0 = time.monotonic()
        rgb_msg = self._wait_rgb(6.0)
        timings['wait_rgb'] = time.monotonic() - t0
        # Depth optional for visual_laser; still try nearest pair for diagnostics.
        pair = pair_nearest(self._rgb_buf, self._depth_buf, prefer='rgb')
        if rgb_msg is None or self._scan is None or self._field is None:
            self._request_rgb(False)
            res.result_code = NO_DATA
            res.diagnostics_json = json.dumps(
                {
                    'rgb': rgb_msg is not None,
                    'scan': self._scan is not None,
                    'map': self._field is not None,
                    'mode': mode,
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
        qorb = extract_orb(bgr, self._orb)
        timings['orb'] = time.monotonic() - t0

        retrieval_pool = [k for k in self._keyframes if k.get('retrieval_ready', True)]
        t0 = time.monotonic()
        retr = retrieve_topk(bgr, retrieval_pool, top_k=max_k)
        timings['retrieval'] = time.monotonic() - t0

        import math as _math

        topk_dbg = []
        laser_rank: list = []
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
            )
            t_laser += time.monotonic() - tl
            entry = {
                'id': cand.keyframe_id,
                'retrieval_rank': cand.rank,
                'retrieval_score': cand.score,
                'ratio_matches': cand.ratio_matches,
                'seed': seed.as_tuple(),
                'refined': refined.refined.as_tuple(),
                'laser_ok': refined.accepted,
                'laser_reason': refined.reason,
                'laser_score': refined.top1_score,
                'laser_margin': refined.margin,
                'valid_beams': refined.score.valid_beams,
                'matched_ratio': refined.score.matched_ratio,
                'mean_dist': refined.score.mean_dist,
                'p90_dist': refined.score.p90_dist,
                'dx': refined.dx,
                'dy': refined.dy,
                'dyaw': refined.dyaw,
                'refine_runtime': refined.runtime_sec,
            }
            topk_dbg.append(entry)
            if refined.accepted:
                laser_rank.append((cand, refined))

        timings['laser_total'] = t_laser
        timings['total'] = time.monotonic() - t_all
        laser_rank.sort(key=lambda x: x[1].top1_score, reverse=True)
        res.candidate_count = len(retr.candidates)
        res.time_to_candidate_sec = float(timings['total'])
        res.stage_timings_json = json.dumps(timings)

        decision_status = 'UNKNOWN'
        decision_reason = 'no_laser_survivor'
        best_cand = None
        best_ref = None
        margin = 0.0
        if laser_rank:
            best_cand, best_ref = laser_rank[0]
            top2 = laser_rank[1][1].top1_score if len(laser_rank) > 1 else 0.0
            margin = best_ref.top1_score - top2
            # Hard gates: visual Top-K already satisfied; laser score/margin/free/refine distance
            if margin < float(self.get_parameter('laser_min_margin').value):
                decision_status = 'UNKNOWN'
                decision_reason = 'laser_top_margin'
            else:
                decision_status = 'ACCEPT'
                decision_reason = 'visual_topk_and_laser_ok'

        if decision_status != 'ACCEPT' or best_ref is None or best_cand is None:
            dump_depth = depth_u16 if depth_u16 is not None else np.zeros((480, 640), np.uint16)
            self._dump_failure(
                bgr,
                dump_depth,
                topk_dbg,
                {'decision': decision_status, 'reason': decision_reason, 'mode': mode},
            )
            self._request_rgb(False)
            res.result_code = UNKNOWN
            res.diagnostics_json = json.dumps(
                {
                    'decision': decision_status,
                    'reason': decision_reason,
                    'pipeline_mode': mode,
                    'query_features': retr.query_features,
                    'topk': topk_dbg,
                    'note': 'Visual proposes; Laser decides. RGB-D geometry optional/future.',
                }
            )
            return res

        res.visual_score = float(best_cand.score)
        res.geometry_score = 0.0  # not used in visual_laser V1
        res.laser_score = float(best_ref.top1_score)
        res.composite_score = float(0.4 * best_cand.score + 0.6 * best_ref.top1_score)
        res.confidence = float(res.composite_score)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = best_ref.refined.x
        pose_msg.pose.pose.position.y = best_ref.refined.y
        pose_msg.pose.pose.orientation = _yaw_to_quat(best_ref.refined.yaw)
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = 0.15
        res.pose = pose_msg
        res.diagnostics_json = json.dumps(
            {
                'decision': 'ACCEPT',
                'reason': decision_reason,
                'pipeline_mode': mode,
                'keyframe_id': best_cand.keyframe_id,
                'visual_rank': best_cand.rank,
                'laser_margin': margin,
                'seed': best_ref.seed.as_tuple(),
                'refined': best_ref.refined.as_tuple(),
                'dx': best_ref.dx,
                'dy': best_ref.dy,
                'dyaw': best_ref.dyaw,
                'apply_initial_pose': apply_pose,
                'topk': topk_dbg,
            }
        )

        if not apply_pose:
            self._request_rgb(False)
            res.success = True
            res.result_code = DRY_RUN_OK
            return res

        # apply path remains code-present but V1.1 forces apply_pose=false above
        if bool(req.allow_motion):
            self.get_logger().warn('allow_motion ignored in PoC V1 (no auto spin)')
        self._initialpose_pub.publish(pose_msg)
        if self._nomotion.service_is_ready():
            try:
                self._nomotion.call_async(Empty.Request())
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'nomotion failed: {exc}')
        t_amcl = time.monotonic()
        deadline = t_amcl + float(self.get_parameter('amcl_timeout_sec').value)
        ready = False
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._amcl is None:
                continue
            c = self._amcl.pose.covariance
            xy = max(float(c[0]), float(c[7]))
            yaw = float(c[35])
            if self._loc_status == 0 and xy < 0.8 and yaw < 0.35:
                ready = True
                break
        res.amcl_convergence_sec = float(time.monotonic() - t_amcl)
        self._request_rgb(False)
        if ready:
            res.success = True
            res.result_code = READY
        else:
            res.success = False
            res.result_code = AMCL_TIMEOUT
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalRelocPoc()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._request_rgb(False)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
