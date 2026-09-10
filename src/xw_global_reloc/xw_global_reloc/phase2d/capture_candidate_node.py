#!/usr/bin/env python3
"""Phase2D-A2 capture node: on-demand Pose/Laser/Image gates → Candidate only.

IDLE: cheap latched state subscriptions only (no /map /scan scoring).
On /xw/visual_db/capture_candidate: arm heavy sensors briefly, run pipeline, disarm.
Never writes production visual/keyframes. P3 unchanged.
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from xw_global_reloc.laser_verify import DistanceField
from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files
from xw_global_reloc.phase2d.candidate_writer import CandidateWriter
from xw_global_reloc.phase2d.capture_pipeline import CaptureResult, run_capture_pipeline
from xw_global_reloc.phase2d.config_loader import load_phase2d_config
from xw_global_reloc.phase2d.coverage_model import build_coverage_model
from xw_global_reloc.phase2d.pose_quality_gate import PoseGateInput


def _parse_phase2c_state(raw: str) -> str:
    """Parse latched canonical JSON without importing xw_phase2c."""
    try:
        data = json.loads(raw or '{}')
    except json.JSONDecodeError:
        return str(raw or '').strip()
    if isinstance(data, dict):
        return str(data.get('state') or '').strip()
    return str(raw or '').strip()


_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
_MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp_age(node: Node, stamp) -> Optional[float]:
    try:
        now = node.get_clock().now()
        msg_t = rclpy.time.Time.from_msg(stamp)
        return max(0.0, (now - msg_t).nanoseconds * 1e-9)
    except Exception:  # noqa: BLE001
        return None


class VisualDbCaptureNode(Node):
    def __init__(self, cfg: Optional[Dict[str, Any]] = None) -> None:
        super().__init__('xw_visual_db_capture')
        self.declare_parameter('config_path', '')
        self.declare_parameter('dry_run_default', False)
        self.declare_parameter('maps_dir', '')
        self.declare_parameter('map_name', '')
        self.declare_parameter('follow_localization_mode', 'continuous')

        cfg_path = str(self.get_parameter('config_path').value or '').strip()
        self._cfg = cfg or load_phase2d_config(Path(cfg_path) if cfg_path else None)
        # Allow ROS overrides for maps
        md = str(self.get_parameter('maps_dir').value or '').strip()
        mn = str(self.get_parameter('map_name').value or '').strip()
        if md:
            self._cfg['maps_dir'] = md
        if mn:
            self._cfg['map_name'] = mn

        self._bridge = CvBridge()
        self._writer = CandidateWriter(self._cfg)
        self._coverage = build_coverage_model(self._cfg, load_descriptors=False)
        _bs = self._coverage.baseline_stats()
        self.get_logger().info(
            'coverage loaded active=%s cand=%s active_cells=%s'
            % (_bs.get('active_frames'), _bs.get('candidate_frames'), _bs.get('active_occupied_cells'))
        )
        self._lock = threading.Lock()
        self._capturing = False

        # Cheap always-on state
        self._loc_status: Optional[int] = None
        self._phase2c_state = ''
        self._phase2c_loc = ''
        self._follow = False
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._odom: Optional[Odometry] = None
        self._map_name = str(self._cfg.get('map_name') or 'vp')
        self._map_hash = ''
        self._refresh_map_hash()

        # Heavy / on-demand
        self._scan: Optional[LaserScan] = None
        self._map: Optional[OccupancyGrid] = None
        self._field: Optional[DistanceField] = None
        self._rgb: Optional[Image] = None
        self._scan_sub = None
        self._map_sub = None
        self._rgb_sub = None
        self._armed = False

        self._cb_svc = ReentrantCallbackGroup()
        self._cb_state = MutuallyExclusiveCallbackGroup()
        # Heavy sensors must run while capture_once blocks in _wait_armed.
        self._cb_sensor = MutuallyExclusiveCallbackGroup()

        # TF only while armed (TransformListener on /tf is expensive on Rockchip IDLE).
        self._tf = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self._tfl = None
        self._amcl_sub = None
        self._odom_sub = None

        self.create_subscription(
            Int8,
            str(self._cfg.get('localization_status_topic') or '/xw/localization_status'),
            self._on_loc,
            _LATCH,
            callback_group=self._cb_state,
        )
        self.create_subscription(
            String,
            str(self._cfg.get('phase2c_state_topic') or '/xw/localization/phase2c_state'),
            self._on_phase2c_state,
            _LATCH,
            callback_group=self._cb_state,
        )
        self.create_subscription(
            String,
            str(self._cfg.get('phase2c_loc_state_topic') or '/xw/localization/phase2c_loc_state'),
            self._on_phase2c_loc,
            _LATCH,
            callback_group=self._cb_state,
        )
        self.create_subscription(
            Bool,
            str(self._cfg.get('follow_enable_topic') or '/xw/follow/enable'),
            self._on_follow,
            _LATCH,
            callback_group=self._cb_state,
        )
        self.create_subscription(
            String,
            str(self._cfg.get('map_name_topic') or '/xw/nav/map_name'),
            self._on_map_name,
            _LATCH,
            callback_group=self._cb_state,
        )

        # Capture command: JSON {"dry_run": bool, "source": str}
        self.create_subscription(
            String,
            '/xw/visual_db/capture_candidate',
            self._on_capture_cmd,
            10,
            callback_group=self._cb_svc,
        )
        self._result_pub = self.create_publisher(String, '/xw/visual_db/capture_result', 10)
        self._rgb_req = self.create_publisher(
            Bool, str(self._cfg.get('capture', {}).get('rgb_request_topic') or '/xw/reloc/rgb_request'), _LATCH
        )

        self.create_service(
            Trigger, '/xw/visual_db/capture_candidate_svc', self._on_trigger, callback_group=self._cb_svc
        )

        self.get_logger().info(
            f'Phase2D-A2 capture ready candidate_root={self._writer.root} '
            f'map={self._map_name} hash={self._map_hash[:12]}…'
        )

    def _refresh_map_hash(self) -> None:
        try:
            y, p = resolve_map_files(Path(self._cfg['maps_dir']), self._map_name)
            self._map_hash = map_pair_hash(y, p)
        except Exception as exc:  # noqa: BLE001
            self._map_hash = ''
            self.get_logger().warn(f'map_hash unavailable: {exc}')

    def _on_loc(self, msg: Int8) -> None:
        self._loc_status = int(msg.data)

    def _on_phase2c_state(self, msg: String) -> None:
        self._phase2c_state = _parse_phase2c_state(msg.data)

    def _on_phase2c_loc(self, msg: String) -> None:
        self._phase2c_loc = str(msg.data or '').strip()

    def _on_follow(self, msg: Bool) -> None:
        self._follow = bool(msg.data)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_map_name(self, msg: String) -> None:
        name = str(msg.data or '').strip()
        if name and name != self._map_name:
            self._map_name = name
            self._cfg['map_name'] = name
            self._writer = CandidateWriter(self._cfg)
            self._coverage = build_coverage_model(self._cfg, load_descriptors=False)
            self._refresh_map_hash()

    def _legacy_freeze_active(self) -> bool:
        mode = str(self.get_parameter('follow_localization_mode').value or 'continuous')
        return bool(self._follow) and mode == 'legacy_freeze'

    def _tf_age(self, parent: str, child: str) -> Optional[float]:
        try:
            tf = self._tf.lookup_transform(parent, child, rclpy.time.Time())
            return _stamp_age(self, tf.header.stamp)
        except TransformException:
            return None

    def _arm_heavy(self) -> None:
        if self._armed:
            return
        self._scan = None
        self._rgb = None
        self._amcl = None
        self._odom = None
        # Keep map/field if already cached from prior arm (map is latched & expensive).
        if self._tfl is None:
            self._tfl = TransformListener(self._tf, self)
        if self._amcl_sub is None:
            self._amcl_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                str(self._cfg.get('amcl_topic') or 'amcl_pose'),
                self._on_amcl,
                10,
                callback_group=self._cb_sensor,
            )
        if self._odom_sub is None:
            self._odom_sub = self.create_subscription(
                Odometry,
                str(self._cfg.get('odom_topic') or '/odom'),
                self._on_odom,
                10,
                callback_group=self._cb_sensor,
            )
        if self._scan_sub is None:
            self._scan_sub = self.create_subscription(
                LaserScan,
                str(self._cfg.get('scan_topic') or '/scan'),
                self._on_scan,
                10,
                callback_group=self._cb_sensor,
            )
        if self._map_sub is None:
            self._map_sub = self.create_subscription(
                OccupancyGrid,
                str(self._cfg.get('map_topic') or '/map'),
                self._on_map,
                _MAP_QOS,
                callback_group=self._cb_sensor,
            )
        if self._rgb_sub is None:
            self._rgb_sub = self.create_subscription(
                Image,
                str(self._cfg.get('rgb_topic') or '/camera/front_up/color/image_raw'),
                self._on_rgb,
                _SENSOR,
                callback_group=self._cb_sensor,
            )
        try:
            self._rgb_req.publish(Bool(data=True))
        except Exception:  # noqa: BLE001
            pass
        self._armed = True

    def _disarm_heavy(self) -> None:
        for attr in ('_scan_sub', '_rgb_sub', '_amcl_sub', '_odom_sub'):
            sub = getattr(self, attr)
            if sub is not None:
                try:
                    self.destroy_subscription(sub)
                except Exception:  # noqa: BLE001
                    pass
                setattr(self, attr, None)
        # Keep map subscription briefly? Drop to minimize IDLE CPU — rebuild field next time.
        if self._map_sub is not None:
            try:
                self.destroy_subscription(self._map_sub)
            except Exception:  # noqa: BLE001
                pass
            self._map_sub = None
        if self._tfl is not None:
            for attr in ('tf_sub_', 'tf_static_sub_', 'tf_sub', 'tf_static_sub'):
                sub = getattr(self._tfl, attr, None)
                if sub is not None:
                    try:
                        self.destroy_subscription(sub)
                    except Exception:  # noqa: BLE001
                        pass
            self._tfl = None
        try:
            self._rgb_req.publish(Bool(data=False))
        except Exception:  # noqa: BLE001
            pass
        self._armed = False
        # Drop heavy payloads from IDLE memory path (field rebuild is capture-time cost).
        self._scan = None
        self._rgb = None
        self._map = None
        self._field = None
        self._amcl = None
        self._odom = None

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg
        try:
            self._field = DistanceField(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'DistanceField failed: {exc}')
            self._field = None

    def _on_rgb(self, msg: Image) -> None:
        self._rgb = msg

    def _wait_armed(self, timeout: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            if (
                self._scan is not None
                and self._map is not None
                and self._field is not None
                and self._rgb is not None
                and self._amcl is not None
            ):
                return True
            time.sleep(0.05)
        return (
            self._scan is not None
            and self._rgb is not None
            and self._amcl is not None
            and (self._field is not None or self._map is not None)
        )

    def _build_pose_input(self) -> PoseGateInput:
        cov_xy = cov_yaw = None
        x = y = yaw = None
        if self._amcl is not None:
            p = self._amcl.pose.pose
            x = float(p.position.x)
            y = float(p.position.y)
            yaw = _yaw_from_quat(p.orientation)
            c = self._amcl.pose.covariance
            cov_xy = max(float(c[0]), float(c[7]))
            cov_yaw = float(c[35])
        speed = yaw_rate = None
        if self._odom is not None:
            t = self._odom.twist.twist
            speed = math.hypot(float(t.linear.x), float(t.linear.y))
            yaw_rate = float(t.angular.z)
        scan_age = _stamp_age(self, self._scan.header.stamp) if self._scan is not None else None
        return PoseGateInput(
            localization_status=self._loc_status,
            phase2c_state=self._phase2c_state or self._phase2c_loc,
            phase2c_loc_state=self._phase2c_loc,
            follow_active=self._follow,
            legacy_freeze_active=self._legacy_freeze_active(),
            amcl_cov_xy=cov_xy,
            amcl_cov_yaw=cov_yaw,
            map_base_age_sec=self._tf_age('map', 'base_link'),
            map_odom_age_sec=self._tf_age('map', 'odom'),
            scan_age_sec=scan_age,
            speed_mps=speed,
            yaw_rate=yaw_rate,
            x=x,
            y=y,
            yaw=yaw,
            map_name=self._map_name,
            map_hash=self._map_hash,
            scan_present=self._scan is not None,
        )

    def capture_once(
        self,
        *,
        dry_run: bool = False,
        source: str = 'manual_test',
        build_session_id: str = '',
    ) -> CaptureResult:
        with self._lock:
            if self._capturing:
                return CaptureResult(status='ERROR', reason='capture_busy', dry_run=dry_run)
            self._capturing = True
        try:
            self._arm_heavy()
            timeout = float(self._cfg.get('capture', {}).get('arm_timeout_sec', 3.0))
            ok = self._wait_armed(timeout)
            if not ok:
                self.get_logger().warn(
                    'arm timeout: '
                    f'scan={self._scan is not None} map={self._map is not None} '
                    f'field={self._field is not None} rgb={self._rgb is not None} '
                    f'amcl={self._amcl is not None}'
                )

            pose_in = self._build_pose_input()
            bgr = None
            if self._rgb is not None:
                try:
                    bgr = self._bridge.imgmsg_to_cv2(self._rgb, 'bgr8')
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'rgb convert failed: {exc}')

            scan_stamp = None
            if self._scan is not None:
                scan_stamp = float(self._scan.header.stamp.sec) + float(self._scan.header.stamp.nanosec) * 1e-9

            result = run_capture_pipeline(
                cfg=self._cfg,
                pose_input=pose_in,
                scan=self._scan,
                occupancy_map=self._map,
                field=self._field,
                bgr=bgr,
                writer=self._writer,
                coverage=self._coverage,
                dry_run=dry_run,
                source=source,
                scan_stamp=scan_stamp,
                build_session_id=build_session_id or None,
            )
            self.get_logger().info(
                f'capture {result.status} reason={result.reason} '
                f'cell={result.decision.spatial_cell if result.decision else None} '
                f'yaw_bin={result.decision.yaw_bin if result.decision else None} '
                f'laser={result.laser.laser_score if result.laser else None} '
                f'written={result.written} dry_run={dry_run}'
            )
            return result
        finally:
            self._disarm_heavy()
            with self._lock:
                self._capturing = False

    def _publish_result(self, result: CaptureResult) -> None:
        self._result_pub.publish(String(data=json.dumps(result.to_dict(), separators=(',', ':'))))

    def _on_capture_cmd(self, msg: String) -> None:
        dry = bool(self.get_parameter('dry_run_default').value)
        source = 'manual_test'
        build_session_id = ''
        try:
            payload = json.loads(msg.data or '{}')
            if isinstance(payload, dict):
                if 'dry_run' in payload:
                    dry = bool(payload['dry_run'])
                source = str(payload.get('source') or source)
                build_session_id = str(payload.get('build_session_id') or '')
        except json.JSONDecodeError:
            if (msg.data or '').strip().lower() in ('dry_run', 'dry-run', '1', 'true'):
                dry = True
        # Run inline on MultiThreadedExecutor: arm/subs created on executor thread;
        # _cb_sensor callbacks run on the other thread while _wait_armed sleeps.
        result = self.capture_once(dry_run=dry, source=source, build_session_id=build_session_id)
        self._publish_result(result)

    def _on_trigger(self, request, response):  # noqa: ANN001
        dry = bool(self.get_parameter('dry_run_default').value)
        result = self.capture_once(dry_run=dry, source='manual_test')
        response.success = result.status == 'ACCEPTED'
        response.message = json.dumps(result.to_dict(), separators=(',', ':'))
        self._publish_result(result)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualDbCaptureNode()
    # 2 threads: capture cmd (_cb_svc) blocks in _wait_armed; sensors (_cb_sensor) need
    # a second thread. Keep thread count low for Rockchip IDLE.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
