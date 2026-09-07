#!/usr/bin/env python3
"""Stage 1 — sparse keyframe DB builder with retrieval/geometry tiers."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int8
from tf2_ros import Buffer, TransformException, TransformListener

from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files
from xw_global_reloc.orb_utils import extract_orb, make_orb, pack_keypoints, visual_difference
from xw_global_reloc.pairing import ImageRingBuffer, pair_nearest
from xw_global_reloc.quality import evaluate_quality
from xw_global_reloc.transforms import Pose2D, se3_from_xyz_rpy, se3_to_se2_yaw


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


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class KeyframeDbBuilder(Node):
    def __init__(self) -> None:
        super().__init__('xw_keyframe_db_builder')
        self.declare_parameter('maps_dir', '/ros2_ws/maps')
        self.declare_parameter('map_name', 'vp')
        self.declare_parameter('db_root', '')
        self.declare_parameter('kf_translation_m', 0.45)
        self.declare_parameter('kf_yaw_rad', 0.30)
        self.declare_parameter('kf_visual_diff', 0.30)
        self.declare_parameter('max_cov_xy', 0.6)
        self.declare_parameter('max_cov_yaw', 0.35)
        self.declare_parameter('max_tf_age_sec', 1.5)
        self.declare_parameter('max_speed_mps', 0.20)
        self.declare_parameter('max_yaw_rate', 0.35)
        self.declare_parameter('min_depth_valid_ratio', 0.20)
        self.declare_parameter('min_orb_depth_coverage', 0.35)
        self.declare_parameter('min_orb_features', 80)
        self.declare_parameter('depth_scale_m', 0.001)
        self.declare_parameter('rgb_depth_max_dt_sec', 0.05)
        self.declare_parameter('require_loc_status_0', False)
        self.declare_parameter('enable', True)
        self.declare_parameter('max_keyframes', 40)
        self.declare_parameter('force_interval_sec', 2.5)  # dense local capture when stationary

        self._bridge = CvBridge()
        self._orb = make_orb(1000)
        self._rgb_buf = ImageRingBuffer(40)
        self._depth_buf = ImageRingBuffer(40)
        self._info: Optional[CameraInfo] = None
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._loc_status = 1
        self._odom: Optional[Odometry] = None
        self._last_pose: Optional[Pose2D] = None
        self._last_desc = None
        self._last_save_mono = 0.0
        self._count = 0
        self._retrieval_n = 0
        self._geometry_n = 0
        self._feat_sum = 0
        self._cov_sum = 0.0
        self._seen_rgb_t = set()

        maps_dir = Path(str(self.get_parameter('maps_dir').value))
        map_name = str(self.get_parameter('map_name').value)
        yaml_p, pgm_p = resolve_map_files(maps_dir, map_name)
        self._map_hash = map_pair_hash(yaml_p, pgm_p)
        db = str(self.get_parameter('db_root').value).strip()
        if not db:
            db = str(maps_dir / map_name / 'visual')
        self._root = Path(db)
        (self._root / 'keyframes').mkdir(parents=True, exist_ok=True)
        (self._root / 'descriptors').mkdir(parents=True, exist_ok=True)
        (self._root / 'state').mkdir(parents=True, exist_ok=True)
        self._manifest_path = self._root / 'manifest.yaml'
        self._write_manifest(yaml_p, pgm_p)

        self._rgb_req = self.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
        self._rgb_req.publish(Bool(data=True))

        self.create_subscription(Image, '/camera/front_up/color/image_raw', self._on_rgb, _SENSOR_QOS)
        self.create_subscription(Image, '/camera/front_up/depth/image_raw', self._on_depth, _SENSOR_QOS)
        self.create_subscription(CameraInfo, '/camera/front_up/color/camera_info', self._on_info, _SENSOR_QOS)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10)
        self.create_subscription(Int8, '/xw/localization_status', self._on_loc, _LATCH)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)
        self.create_timer(0.15, self._tick)
        self.create_timer(8.0, self._log_stats)
        self.get_logger().info(
            f'keyframe builder root={self._root} map_hash={self._map_hash[:12]}… '
            f'(retrieval/geometry tiers)'
        )

    def _write_manifest(self, yaml_p: Path, pgm_p: Path) -> None:
        man = {
            'schema_version': 2,
            'map_name': str(self.get_parameter('map_name').value),
            'map_yaml': str(yaml_p),
            'map_pgm': str(pgm_p),
            'map_hash': self._map_hash,
            'camera': 'front_up',
            'extrinsic_status': 'EXTRINSIC_UNCALIBRATED',
            'descriptor': 'ORB',
            'tiers': ['retrieval_ready', 'geometry_ready'],
            'created': time.time(),
        }
        self._manifest_path.write_text(yaml.safe_dump(man, sort_keys=False), encoding='utf-8')

    def _on_rgb(self, msg: Image) -> None:
        self._rgb_buf.push(msg)

    def _on_depth(self, msg: Image) -> None:
        self._depth_buf.push(msg)

    def _on_info(self, msg: CameraInfo) -> None:
        self._info = msg

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg

    def _on_loc(self, msg: Int8) -> None:
        self._loc_status = int(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _cov_ok(self) -> bool:
        if self._amcl is None:
            return False
        c = self._amcl.pose.covariance
        xy = max(float(c[0]), float(c[7]))
        yaw = float(c[35])
        return xy <= float(self.get_parameter('max_cov_xy').value) and yaw <= float(
            self.get_parameter('max_cov_yaw').value
        )

    def _speed_ok(self) -> bool:
        if self._odom is None:
            return True  # allow if odom missing but amcl ok
        t = self._odom.twist.twist.linear
        w = self._odom.twist.twist.angular.z
        speed = math.hypot(t.x, t.y)
        return speed <= float(self.get_parameter('max_speed_mps').value) and abs(w) <= float(
            self.get_parameter('max_yaw_rate').value
        )

    def _tf_ok(self) -> bool:
        try:
            tf = self._tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            # Stationary AMCL often stops refreshing TF stamps; accept identity lookup
            # if amcl_pose itself is recent.
            if self._amcl is not None:
                return True
            if tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0:
                return True
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            return age <= float(self.get_parameter('max_tf_age_sec').value)
        except TransformException:
            # Fall back: amcl_pose alone is enough for DB building when map TF flaky.
            return self._amcl is not None

    def _pose_ok(self) -> bool:
        if bool(self.get_parameter('require_loc_status_0').value) and self._loc_status != 0:
            return False
        return self._cov_ok() and self._tf_ok() and self._speed_ok() and self._amcl is not None

    def _pose_from_amcl(self) -> Pose2D:
        p = self._amcl.pose.pose
        return Pose2D(p.position.x, p.position.y, _yaw_from_quat(p.orientation))

    def _should_insert(self, pose: Pose2D, desc) -> bool:
        now = time.monotonic()
        force_iv = float(self.get_parameter('force_interval_sec').value)
        if force_iv > 0 and self._count < 12 and (now - self._last_save_mono) >= force_iv:
            return True
        if self._last_pose is None:
            return True
        dx = pose.x - self._last_pose.x
        dy = pose.y - self._last_pose.y
        dist = math.hypot(dx, dy)
        dyaw = abs(
            math.atan2(math.sin(pose.yaw - self._last_pose.yaw), math.cos(pose.yaw - self._last_pose.yaw))
        )
        vdiff = visual_difference(desc, self._last_desc)
        return (
            dist >= float(self.get_parameter('kf_translation_m').value)
            or dyaw >= float(self.get_parameter('kf_yaw_rad').value)
            or vdiff >= float(self.get_parameter('kf_visual_diff').value)
        )

    def _cam_pose(self, base: Pose2D) -> dict:
        T_bc = se3_from_xyz_rpy(0.251, 0.0, 0.49, -1.33, 0.0, -1.5708)
        T_mb = se3_from_xyz_rpy(base.x, base.y, 0.0, 0.0, 0.0, base.yaw)
        T_mc = T_mb @ T_bc
        cam = se3_to_se2_yaw(T_mc)
        return {
            'x': cam.x,
            'y': cam.y,
            'yaw': cam.yaw,
            'T_base_from_cam_xyz_rpy': [0.251, 0.0, 0.49, -1.33, 0.0, -1.5708],
            'extrinsic_status': 'EXTRINSIC_UNCALIBRATED',
        }

    def _tick(self) -> None:
        if not bool(self.get_parameter('enable').value):
            return
        if self._count >= int(self.get_parameter('max_keyframes').value):
            return
        if not self._pose_ok() or self._info is None:
            return
        pair = pair_nearest(self._rgb_buf, self._depth_buf, prefer='rgb')
        if pair is None:
            return
        key = round(pair.rgb_t, 3)
        if key in self._seen_rgb_t:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(pair.rgb, 'bgr8')
            depth = self._bridge.imgmsg_to_cv2(pair.depth, 'passthrough')
        except Exception:  # noqa: BLE001
            return
        if depth.dtype != np.uint16:
            return
        orb = extract_orb(bgr, self._orb)
        q = evaluate_quality(
            pose_ok=True,
            camera_info_ok=self._info is not None,
            orb=orb,
            depth_u16=depth,
            pair_dt_sec=pair.pair_dt_sec,
            min_orb=int(self.get_parameter('min_orb_features').value),
            max_pair_dt=float(self.get_parameter('rgb_depth_max_dt_sec').value),
            min_depth_valid=float(self.get_parameter('min_depth_valid_ratio').value),
            min_orb_depth_cov=float(self.get_parameter('min_orb_depth_coverage').value),
        )
        if not q.retrieval_ready:
            return
        pose = self._pose_from_amcl()
        if not self._should_insert(pose, orb.descriptors):
            return
        self._seen_rgb_t.add(key)
        self._save_kf(pose, bgr, depth, orb, q, pair)
        self._last_pose = pose
        self._last_desc = orb.descriptors
        self._last_save_mono = time.monotonic()

    def _save_kf(self, pose: Pose2D, bgr, depth, orb, q, pair) -> None:
        self._count += 1
        if q.retrieval_ready:
            self._retrieval_n += 1
        if q.geometry_ready:
            self._geometry_n += 1
        kid = f'kf_{self._count:06d}'
        kdir = self._root / 'keyframes' / kid
        kdir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(kdir / 'rgb.jpg'), bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        cv2.imwrite(str(kdir / 'depth.png'), depth)
        np.save(str(kdir / 'descriptors.npy'), orb.descriptors)
        np.save(str(kdir / 'keypoints.npy'), pack_keypoints(orb.keypoints))
        meta = {
            'keyframe_id': kid,
            'timestamp': time.time(),
            'stamp_rgb': pair.rgb_t,
            'stamp_depth': pair.depth_t,
            'pair_dt_sec': pair.pair_dt_sec,
            'map_pose': {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw},
            'camera_pose': self._cam_pose(pose),
            'camera_info': {
                'width': self._info.width,
                'height': self._info.height,
                'k': list(self._info.k),
                'd': list(self._info.d),
                'frame_id': self._info.header.frame_id,
            },
            'retrieval_ready': q.retrieval_ready,
            'geometry_ready': q.geometry_ready,
            'quality': {
                'orb_features': q.orb_total,
                'orb_with_valid_depth': q.orb_with_valid_depth,
                'orb_depth_coverage_ratio': q.orb_depth_coverage_ratio,
                'depth_valid_ratio': q.depth_valid_ratio,
                'pair_dt_sec': q.pair_dt_sec,
                'amcl_cov_xy': max(float(self._amcl.pose.covariance[0]), float(self._amcl.pose.covariance[7])),
                'amcl_cov_yaw': float(self._amcl.pose.covariance[35]),
                'reasons': q.reasons,
            },
            'map_hash': self._map_hash,
            'paths': {'rgb': 'rgb.jpg', 'depth': 'depth.png'},
            'extrinsic_status': 'EXTRINSIC_UNCALIBRATED',
        }
        (kdir / 'meta.yaml').write_text(yaml.safe_dump(meta, sort_keys=False), encoding='utf-8')
        idx_path = self._root / 'descriptors' / 'index.json'
        idx = json.loads(idx_path.read_text(encoding='utf-8')) if idx_path.is_file() else []
        idx.append(
            {
                'id': kid,
                'pose': meta['map_pose'],
                'orb_features': q.orb_total,
                'retrieval_ready': q.retrieval_ready,
                'geometry_ready': q.geometry_ready,
                'dir': kid,
            }
        )
        idx_path.write_text(json.dumps(idx, indent=2), encoding='utf-8')
        self._feat_sum += q.orb_total
        self._cov_sum += q.orb_depth_coverage_ratio
        self.get_logger().info(
            f'saved {kid} ret={q.retrieval_ready} geo={q.geometry_ready} '
            f'orb={q.orb_total} cov={q.orb_depth_coverage_ratio:.2f} dt_ms={pair.pair_dt_sec*1000:.1f}'
        )

    def _log_stats(self) -> None:
        usage = sum(p.stat().st_size for p in self._root.rglob('*') if p.is_file())
        feats = []
        covs = []
        for meta_p in (self._root / 'keyframes').glob('*/meta.yaml'):
            m = yaml.safe_load(meta_p.read_text(encoding='utf-8')) or {}
            q = m.get('quality') or {}
            if 'orb_features' in q:
                feats.append(float(q['orb_features']))
            if 'orb_depth_coverage_ratio' in q:
                covs.append(float(q['orb_depth_coverage_ratio']))
        poses = []
        for meta_p in (self._root / 'keyframes').glob('*/meta.yaml'):
            m = yaml.safe_load(meta_p.read_text(encoding='utf-8')) or {}
            p = m.get('map_pose') or {}
            if 'x' in p:
                poses.append((float(p['x']), float(p['y'])))
        coverage = None
        if len(poses) >= 2:
            xs = [p[0] for p in poses]
            ys = [p[1] for p in poses]
            coverage = {
                'x_span_m': max(xs) - min(xs),
                'y_span_m': max(ys) - min(ys),
                'n_poses': len(poses),
            }
        stats = {
            'keyframe_count': self._count,
            'retrieval_ready': self._retrieval_n,
            'geometry_ready': self._geometry_n,
            'disk_bytes': usage,
            'orb_features': {
                'mean': float(sum(feats) / len(feats)) if feats else None,
                'p10': float(np.percentile(feats, 10)) if feats else None,
                'p90': float(np.percentile(feats, 90)) if feats else None,
            },
            'orb_depth_coverage_mean': float(sum(covs) / len(covs)) if covs else None,
            'map_coverage': coverage,
            'map_hash': self._map_hash,
            'root': str(self._root),
        }
        (self._root / 'state' / 'builder_stats.json').write_text(json.dumps(stats, indent=2), encoding='utf-8')
        self.get_logger().info(f'stats {stats}')

    def destroy_node(self) -> bool:
        try:
            self._rgb_req.publish(Bool(data=False))
            self._log_stats()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyframeDbBuilder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
