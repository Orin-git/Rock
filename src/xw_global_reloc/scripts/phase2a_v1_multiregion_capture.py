#!/usr/bin/env python3
"""Phase2A V1 multi-region Visual DB capture.

Navigates existing waypoints, captures retrieval_ready keyframes + query
snapshots (RGB + scan + GT pose). Depth may be geometry_ready=false.
Does not change retrieval/laser algorithms. apply_initial_pose unused.
"""

from __future__ import annotations

import json
import math
import shutil
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files
from xw_global_reloc.orb_utils import extract_orb, make_orb, pack_keypoints
from xw_global_reloc.pairing import ImageRingBuffer, pair_nearest
from xw_global_reloc.quality import evaluate_quality
from xw_global_reloc.transforms import Pose2D, se3_from_xyz_rpy, se3_to_se2_yaw
from xw_interfaces.msg import TaskResult
from xw_interfaces.srv import GetState, MotionCommand


MAPS_DIR = Path('/ros2_ws/maps')
MAP_NAME = 'vp'
VISUAL_ROOT = MAPS_DIR / MAP_NAME / 'visual'
BENCH = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07')
QUERY_ROOT = BENCH / 'queries_v1'
STATE_PATH = BENCH / 'capture_state.json'

# Travel-efficient order from charger. Classes cover the required set.
REGIONS = [
    {
        'id': 'charger_room',
        'cls': 'room',
        'x': 1.8663955491712294,
        'y': -0.05958837147746455,
        'yaw': -3.1286646850836126,
    },
    {
        'id': 'doorway_wp9',
        'cls': 'doorway',
        'x': 0.014664885102192216,
        'y': -0.5797687003570173,
        'yaw': 3.252962113309337,
    },
    {
        'id': 'corridor_wp2',
        'cls': 'corridor',
        'x': -4.0779483147098965,
        'y': -0.9390276581991088,
        'yaw': 3.3018138790728147,
    },
    {
        'id': 'similar_corridor_wp3',
        'cls': 'visual_similar',
        'x': -9.20885679699119,
        'y': 1.1913716785939492,
        'yaw': 3.385938748868999,
    },
    {
        'id': 'room_wp5',
        'cls': 'room',
        'x': -7.871928715871139,
        'y': 3.2495538319074075,
        'yaw': 0.8115781021773631,
    },
    {
        'id': 'open_wp6',
        'cls': 'open',
        'x': -0.7536999230552262,
        'y': 9.387991011393888,
        'yaw': 5.480333851262194,
    },
]

# Pose A/B: 3 yaws each → 6 KF/region (within 4–8) and 2 locations/region.
POSE_A_YAWS_REL = [-0.40, 0.0, 0.40]  # rad ≈ ±23°, 0
POSE_B_YAWS_REL = [-0.25, 0.0, 0.25]
OFFSET_M = 0.40
MAX_KF = 40
NAV_XY_TOL = 0.55
NAV_TIMEOUT = 150.0
SETTLE_SEC = 1.2


_SENSOR = QoSProfile(
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


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def yaw_norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_err(a: float, b: float) -> float:
    return abs(yaw_norm(a - b))


def save_scan(path: Path, scan: LaserScan) -> None:
    np.savez_compressed(
        str(path),
        ranges=np.asarray(scan.ranges, dtype=np.float32),
        intensities=np.asarray(scan.intensities, dtype=np.float32)
        if scan.intensities
        else np.zeros(0, np.float32),
        angle_min=np.float32(scan.angle_min),
        angle_max=np.float32(scan.angle_max),
        angle_increment=np.float32(scan.angle_increment),
        range_min=np.float32(scan.range_min),
        range_max=np.float32(scan.range_max),
        frame_id=np.array(scan.header.frame_id),
    )


class CaptureNode(Node):
    def __init__(self) -> None:
        super().__init__('phase2a_v1_capture')
        self.bridge = CvBridge()
        self.orb = make_orb(1000)
        self.rgb_buf = ImageRingBuffer(40)
        self.depth_buf = ImageRingBuffer(40)
        self.info = None
        self.amcl = None
        self.scan = None
        self.odom = None
        self.loc = 1
        self.task_results = []
        self.rgb_req = self.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
        self.goal_pub = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)
        self.nav_cancel = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self.create_subscription(Image, '/camera/front_up/color/image_raw', self._on_rgb, _SENSOR)
        self.create_subscription(Image, '/camera/front_up/depth/image_raw', self._on_depth, _SENSOR)
        self.create_subscription(
            CameraInfo, '/camera/front_up/color/camera_info', self._on_info, _SENSOR
        )
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, _SENSOR)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Int8, '/xw/localization_status', self._on_loc, _LATCH)
        self.create_subscription(TaskResult, '/xw/task/result', self._on_task, 10)
        self.nomotion = self.create_client(Empty, '/request_nomotion_update')
        self.motion = self.create_client(MotionCommand, '/xw/motion/command')
        self.get_state = self.create_client(GetState, '/xw/supervisor/get_state')
        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)
        self.rgb_req.publish(Bool(data=True))

        yaml_p, pgm_p = resolve_map_files(MAPS_DIR, MAP_NAME)
        self.map_hash = map_pair_hash(yaml_p, pgm_p)
        self.root = VISUAL_ROOT
        self.idx = []
        self.count = 0
        self.queries = []
        self._load_existing()

    def _load_existing(self) -> None:
        idx_p = self.root / 'descriptors' / 'index.json'
        if not idx_p.is_file():
            return
        try:
            idx = json.loads(idx_p.read_text(encoding='utf-8'))
        except json.JSONDecodeError:
            return
        nums = []
        for item in idx:
            kid = str(item.get('id') or '')
            kdir = self.root / 'keyframes' / kid
            if not (kdir / 'meta.yaml').is_file():
                continue
            self.idx.append(item)
            if kid.startswith('kf_'):
                try:
                    nums.append(int(kid.split('_')[-1]))
                except ValueError:
                    pass
        self.count = max(nums) if nums else 0
        qman = QUERY_ROOT / 'manifest.json'
        if qman.is_file():
            try:
                self.queries = json.loads(qman.read_text(encoding='utf-8'))
            except json.JSONDecodeError:
                self.queries = []
        self.get_logger().info(f'resume db count={self.count} n={len(self.idx)}')

    def _on_rgb(self, m):
        self.rgb_buf.push(m)

    def _on_depth(self, m):
        self.depth_buf.push(m)

    def _on_info(self, m):
        self.info = m

    def _on_amcl(self, m):
        self.amcl = m

    def _on_scan(self, m):
        self.scan = m

    def _on_odom(self, m):
        self.odom = m

    def _on_loc(self, m):
        self.loc = int(m.data)

    def _on_task(self, m):
        self.task_results.append(m)

    def spin_wait(self, sec: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self) -> Pose2D | None:
        try:
            tr = self._tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = tr.transform.translation
            return Pose2D(t.x, t.y, yaw_from_quat(tr.transform.rotation))
        except TransformException:
            pass
        if self.amcl is None:
            return None
        p = self.amcl.pose.pose
        return Pose2D(p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def wait_pose(self, timeout: float = 8.0) -> Pose2D | None:
        self.request_nomotion()
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is not None:
                return p
        return None

    def speed(self) -> float:
        if self.odom is None:
            return 0.0
        t = self.odom.twist.twist.linear
        return math.hypot(t.x, t.y)

    def battery(self) -> float | None:
        if not self.get_state.wait_for_service(timeout_sec=1.0):
            return None
        fut = self.get_state.call_async(GetState.Request())
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0 and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done() or fut.result() is None:
            return None
        try:
            return float(fut.result().state.power.battery_percent)
        except Exception:  # noqa: BLE001
            return None

    def request_nomotion(self) -> None:
        if self.nomotion.wait_for_service(timeout_sec=1.0):
            self.nomotion.call_async(Empty.Request())
            self.spin_wait(0.4)

    def goto(self, x: float, y: float, yaw: float, label: str) -> bool:
        cur = self.pose()
        if cur is not None and math.hypot(cur.x - x, cur.y - y) < NAV_XY_TOL:
            self.get_logger().info(f'{label}: already near, skip nav')
            return True
        before = time.monotonic()
        self.task_results.clear()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation = yaw_to_quat(yaw)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'nav → {label} ({x:.2f},{y:.2f})')
        t0 = time.monotonic()
        settled = 0.0
        while time.monotonic() - t0 < NAV_TIMEOUT and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is None:
                continue
            d = math.hypot(p.x - x, p.y - y)
            if d < NAV_XY_TOL and self.speed() < 0.08:
                settled += 0.05
                if settled >= SETTLE_SEC:
                    self.get_logger().info(f'{label}: arrived d={d:.2f}')
                    return True
            else:
                settled = 0.0
            for r in list(self.task_results):
                if r.capability == 'nav' and (r.stamp.sec + r.stamp.nanosec * 1e-9) > before - 1.0:
                    if r.code == 0:
                        # may have arrived even if slightly off
                        p2 = self.pose()
                        if p2 is not None and math.hypot(p2.x - x, p2.y - y) < 1.2:
                            self.get_logger().info(f'{label}: nav result OK')
                            return True
                    if r.code == 1:
                        self.get_logger().warn(f'{label}: nav failed {r.message}')
                        return False
        self.get_logger().warn(f'{label}: nav timeout')
        self.nav_cancel.publish(Bool(data=True))
        self.spin_wait(0.3)
        return False

    def rotate(self, delta_rad: float) -> bool:
        if abs(delta_rad) < math.radians(4):
            return True
        if not self.motion.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('motion service missing')
            return False
        cid = f'cap-{int(time.time() * 1000)}'
        req = MotionCommand.Request()
        req.command_id = cid
        req.angle_deg = float(math.degrees(delta_rad))
        req.distance_m = 0.0
        fut = self.motion.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0 and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        t1 = time.monotonic()
        while time.monotonic() - t1 < 18.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            for r in self.task_results:
                if r.capability == 'motion' and r.command_id == cid:
                    return r.code == 0
            if self.speed() < 0.05 and time.monotonic() - t1 > max(2.0, abs(delta_rad) / 0.45):
                return True
        self.get_logger().warn('rotate wait timeout')
        return self.speed() < 0.12

    def drive(self, dist_m: float) -> bool:
        if abs(dist_m) < 0.05:
            return True
        if not self.motion.wait_for_service(timeout_sec=2.0):
            return False
        cid = f'capd-{int(time.time() * 1000)}'
        req = MotionCommand.Request()
        req.command_id = cid
        req.angle_deg = 0.0
        req.distance_m = float(dist_m)
        fut = self.motion.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0 and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        t1 = time.monotonic()
        while time.monotonic() - t1 < 20.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            for r in self.task_results:
                if r.capability == 'motion' and r.command_id == cid:
                    return r.code == 0
            if self.speed() < 0.05 and time.monotonic() - t1 > max(2.0, abs(dist_m) / 0.18):
                return True
        return self.speed() < 0.12

    def grab(self, region: dict, location_id: str, yaw_rel: float) -> dict | None:
        if self.count >= MAX_KF:
            return None
        self.request_nomotion()
        self.spin_wait(0.6)
        p = self.pose()
        if p is None or self.info is None or self.scan is None:
            self.get_logger().warn('grab skip: missing pose/info/scan')
            return None
        dreg = math.hypot(p.x - float(region['x']), p.y - float(region['y']))
        if dreg > 2.0:
            self.get_logger().warn(
                f'grab skip: pose drifted {dreg:.2f}m from {region["id"]} '
                f'({p.x:.2f},{p.y:.2f})'
            )
            return None
        pair = pair_nearest(self.rgb_buf, self.depth_buf, prefer='rgb')
        # RGB is required; depth optional
        rgb_msg = None
        if pair is not None:
            rgb_msg = pair.rgb
        elif len(self.rgb_buf) > 0:
            rgb_msg = self.rgb_buf._buf[-1].msg
        if rgb_msg is None:
            self.get_logger().warn('grab skip: no rgb')
            return None
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = None
        pair_dt = 9.0
        depth_t = 0.0
        if pair is not None:
            pair_dt = pair.pair_dt_sec
            depth_t = pair.depth_t
            try:
                depth = self.bridge.imgmsg_to_cv2(pair.depth, 'passthrough')
            except Exception:  # noqa: BLE001
                depth = None
        orb = extract_orb(bgr, self.orb)
        if orb.n_features < 50:
            self.get_logger().warn(f'grab skip: orb={orb.n_features}')
            return None
        q = evaluate_quality(
            pose_ok=True,
            camera_info_ok=True,
            orb=orb,
            depth_u16=depth if depth is not None and depth.dtype == np.uint16 else None,
            pair_dt_sec=pair_dt,
            min_orb=50,
            max_pair_dt=0.05,
            min_depth_valid=0.08,
            min_orb_depth_cov=0.25,
        )
        q.retrieval_ready = True  # depth must not block Visual DB
        self.count += 1
        kid = f'kf_{self.count:06d}'
        kdir = self.root / 'keyframes' / kid
        kdir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(kdir / 'rgb.jpg'), bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if depth is not None and depth.dtype == np.uint16:
            cv2.imwrite(str(kdir / 'depth.png'), depth)
        np.save(str(kdir / 'descriptors.npy'), orb.descriptors)
        np.save(str(kdir / 'keypoints.npy'), pack_keypoints(orb.keypoints))
        save_scan(kdir / 'scan.npz', self.scan)
        T_bc = se3_from_xyz_rpy(0.251, 0.0, 0.49, -1.33, 0.0, -1.5708)
        T_mb = se3_from_xyz_rpy(p.x, p.y, 0.0, 0.0, 0.0, p.yaw)
        cam = se3_to_se2_yaw(T_mb @ T_bc)
        cov = self.amcl.pose.covariance if self.amcl is not None else [0.0] * 36
        meta = {
            'keyframe_id': kid,
            'timestamp': time.time(),
            'stamp_rgb': float(rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9),
            'stamp_depth': depth_t,
            'pair_dt_sec': pair_dt,
            'map_pose': {'x': p.x, 'y': p.y, 'yaw': p.yaw},
            'camera_pose': {
                'x': cam.x,
                'y': cam.y,
                'yaw': cam.yaw,
                'extrinsic_status': 'EXTRINSIC_UNCALIBRATED',
            },
            'camera_info': {
                'width': int(self.info.width),
                'height': int(self.info.height),
                'k': [float(x) for x in self.info.k],
                'd': [float(x) for x in self.info.d],
                'frame_id': str(self.info.header.frame_id),
            },
            'retrieval_ready': True,
            'geometry_ready': bool(q.geometry_ready),
            'quality': {
                'orb_features': q.orb_total,
                'orb_with_valid_depth': q.orb_with_valid_depth,
                'orb_depth_coverage_ratio': q.orb_depth_coverage_ratio,
                'depth_valid_ratio': q.depth_valid_ratio,
                'pair_dt_sec': pair_dt,
                'amcl_cov_xy': max(float(cov[0]), float(cov[7])),
                'amcl_cov_yaw': float(cov[35]),
                'reasons': q.reasons,
            },
            'map_hash': self.map_hash,
            'region_id': region['id'],
            'region_class': region['cls'],
            'location_id': location_id,
            'yaw_rel_rad': yaw_rel,
            'extrinsic_status': 'EXTRINSIC_UNCALIBRATED',
            'scan_ref': 'scan.npz',
        }
        (kdir / 'meta.yaml').write_text(yaml.safe_dump(meta, sort_keys=False), encoding='utf-8')
        rec = {
            'id': kid,
            'pose': meta['map_pose'],
            'retrieval_ready': True,
            'geometry_ready': bool(q.geometry_ready),
            'orb_features': q.orb_total,
            'region_id': region['id'],
            'region_class': region['cls'],
            'location_id': location_id,
        }
        self.idx.append(rec)
        qdir = QUERY_ROOT / kid
        qdir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(kdir / 'rgb.jpg', qdir / 'rgb.jpg')
        shutil.copy2(kdir / 'scan.npz', qdir / 'scan.npz')
        (qdir / 'gt.yaml').write_text(yaml.safe_dump(meta, sort_keys=False), encoding='utf-8')
        self.queries.append(
            {
                'id': kid,
                'region_id': region['id'],
                'region_class': region['cls'],
                'location_id': location_id,
                'pose': meta['map_pose'],
                'rgb': str(qdir / 'rgb.jpg'),
                'scan': str(qdir / 'scan.npz'),
                'keyframe_id': kid,
            }
        )
        (self.root / 'descriptors' / 'index.json').write_text(
            json.dumps(self.idx, indent=2), encoding='utf-8'
        )
        self.get_logger().info(
            f'saved {kid} region={region["id"]} loc={location_id} '
            f'geo={q.geometry_ready} orb={q.orb_total} '
            f'pose=({p.x:.2f},{p.y:.2f},{p.yaw:.2f})'
        )
        return rec


def backup_old_db() -> None:
    if not (VISUAL_ROOT / 'keyframes').is_dir():
        return
    kfs = list((VISUAL_ROOT / 'keyframes').glob('kf_*'))
    if not kfs:
        return
    # Keep a v1 in-progress DB; only archive pre-v1 same-cell dumps.
    has_region = False
    for d in kfs:
        mp = d / 'meta.yaml'
        if mp.is_file() and 'region_id:' in mp.read_text(encoding='utf-8', errors='ignore'):
            has_region = True
            break
    if has_region:
        print(f'resume existing multi-region DB ({len(kfs)} kf)')
        return
    bak = MAPS_DIR / MAP_NAME / f'visual_backup_charger_local_{time.strftime("%Y%m%d_%H%M%S")}'
    shutil.copytree(VISUAL_ROOT, bak)
    shutil.rmtree(VISUAL_ROOT / 'keyframes', ignore_errors=True)
    (VISUAL_ROOT / 'keyframes').mkdir(parents=True, exist_ok=True)
    print(f'backed up previous visual DB ({len(kfs)} kf) → {bak}')


def write_manifest(node: CaptureNode) -> None:
    yaml_p, pgm_p = resolve_map_files(MAPS_DIR, MAP_NAME)
    man = {
        'schema_version': 2,
        'map_name': MAP_NAME,
        'map_hash': node.map_hash,
        'pipeline': 'visual_laser',
        'extrinsic_status': 'EXTRINSIC_UNCALIBRATED',
        'tiers': ['retrieval_ready', 'geometry_ready'],
        'note': 'Depth geometry optional; retrieval_ready independent of geometry_ready',
        'created': time.time(),
        'map_yaml': str(yaml_p),
        'map_pgm': str(pgm_p),
    }
    (node.root / 'manifest.yaml').write_text(yaml.safe_dump(man, sort_keys=False), encoding='utf-8')


def write_stats(node: CaptureNode) -> None:
    feats = [i['orb_features'] for i in node.idx]
    geos = sum(1 for i in node.idx if i['geometry_ready'])
    regions = {}
    for i in node.idx:
        regions.setdefault(i['region_id'], 0)
        regions[i['region_id']] += 1
    usage = sum(p.stat().st_size for p in node.root.rglob('*') if p.is_file())
    stats = {
        'keyframe_count': len(node.idx),
        'retrieval_ready': len(node.idx),
        'geometry_ready': geos,
        'disk_bytes': usage,
        'regions': regions,
        'orb_features': {
            'mean': float(np.mean(feats)) if feats else None,
            'p10': float(np.percentile(feats, 10)) if feats else None,
            'p90': float(np.percentile(feats, 90)) if feats else None,
        },
        'map_hash': node.map_hash,
        'map_coverage': {
            'n_regions': len(regions),
            'region_ids': list(regions.keys()),
        },
    }
    (node.root / 'state').mkdir(parents=True, exist_ok=True)
    (node.root / 'state' / 'builder_stats.json').write_text(json.dumps(stats, indent=2), encoding='utf-8')
    QUERY_ROOT.mkdir(parents=True, exist_ok=True)
    (QUERY_ROOT / 'manifest.json').write_text(json.dumps(node.queries, indent=2), encoding='utf-8')
    print(json.dumps(stats, indent=2))


def capture_yaws(node: CaptureNode, region: dict, location_id: str, yaws_rel: list) -> int:
    got = 0
    p0 = node.pose()
    if p0 is None:
        return 0
    heading = p0.yaw
    for rel in yaws_rel:
        if node.count >= MAX_KF:
            break
        target = yaw_norm(heading + rel)
        cur = node.pose()
        if cur is None:
            break
        node.rotate(yaw_norm(target - cur.yaw))
        node.spin_wait(0.8)
        rec = node.grab(region, location_id, rel)
        if rec is not None:
            got += 1
        write_stats(node)
        STATE_PATH.write_text(
            json.dumps({'count': node.count, 'last': rec}, indent=2, default=str),
            encoding='utf-8',
        )
    return got


def main() -> None:
    BENCH.mkdir(parents=True, exist_ok=True)
    QUERY_ROOT.mkdir(parents=True, exist_ok=True)
    backup_old_db()
    VISUAL_ROOT.mkdir(parents=True, exist_ok=True)
    (VISUAL_ROOT / 'keyframes').mkdir(parents=True, exist_ok=True)
    (VISUAL_ROOT / 'descriptors').mkdir(parents=True, exist_ok=True)
    (VISUAL_ROOT / 'state').mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = CaptureNode()
    write_manifest(node)
    node.spin_wait(1.0)
    p0 = node.wait_pose(10.0)
    bat = node.battery()
    print(f'battery={bat} loc={node.loc} pose={p0}')
    if p0 is None:
        print('FAIL: no map→base_link pose; abort capture')
        node.rgb_req.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()
        return
    t_rgb = time.monotonic()
    while time.monotonic() - t_rgb < 8.0 and len(node.rgb_buf) == 0:
        node.spin_wait(0.1)
    print(f'rgb_buf={len(node.rgb_buf)} scan={node.scan is not None}')
    if node.loc != 0:
        print('WARN: localization_status != 0; TF/AMCL pose used for DB build only')

    log = []
    try:
        for region in REGIONS:
            if node.count >= MAX_KF:
                break
            n_have = sum(1 for i in node.idx if i.get('region_id') == region['id'])
            if n_have >= 4:
                print(f'skip {region["id"]}: already {n_have} kf')
                log.append({'event': 'skip_done', 'region': region['id'], 'n': n_have})
                continue
            bat = node.battery()
            if bat is not None and bat < 18:
                print(f'STOP: battery {bat}%')
                break
            ok = node.goto(region['x'], region['y'], region['yaw'], region['id'])
            log.append({'event': 'nav', 'region': region['id'], 'ok': ok})
            if not ok:
                print(f'skip region {region["id"]} (nav fail)')
                continue
            node.spin_wait(1.0)
            n_a = capture_yaws(node, region, f'{region["id"]}_A', POSE_A_YAWS_REL)
            log.append({'event': 'poseA', 'region': region['id'], 'n': n_a})
            if node.count >= MAX_KF:
                break
            # nearby location along current heading
            node.drive(OFFSET_M)
            node.spin_wait(1.0)
            n_b = capture_yaws(node, region, f'{region["id"]}_B', POSE_B_YAWS_REL)
            log.append({'event': 'poseB', 'region': region['id'], 'n': n_b})
            write_stats(node)
    finally:
        node.rgb_req.publish(Bool(data=False))
        write_stats(node)
        (BENCH / 'capture_log.json').write_text(json.dumps(log, indent=2), encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
