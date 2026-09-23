#!/usr/bin/env python3
"""xw_last_good_pose_writer — ≤1 Hz; writes maps/<map>/state/last_good_pose.yaml.

Write only after all gates hold for stable_sec. Never writes during Follow
(legacy_freeze open-loop). Proposal only — readers must validate + laser-verify.
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import json

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, String
from tf2_ros import Buffer, TransformException, TransformListener

from xw_phase2c.last_good_pose import (
    LastGoodPose,
    compute_map_hash_cached,
    pose_delta,
    quality_from_cov,
    read_last_good_pose,
    write_last_good_pose,
    yaw_from_quat,
)
from xw_phase2c.laser_prior_verify import MIN_LASER_SCORE, verify_pose_with_laser


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


class LastGoodPoseWriter(Node):
    def __init__(self) -> None:
        super().__init__('xw_last_good_pose_writer')
        self.declare_parameter('maps_dir', '/ros2_ws/maps')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tick_hz', 1.0)
        self.declare_parameter('stable_sec', 5.0)
        self.declare_parameter('tf_stale_sec', 1.0)
        self.declare_parameter('cov_xy_good', 0.5)
        self.declare_parameter('cov_yaw_good', 0.25)
        self.declare_parameter('stable_xy_m', 0.15)
        self.declare_parameter('stable_yaw_rad', 0.12)
        self.declare_parameter('min_write_interval_sec', 10.0)
        # Frozen shared laser gate. Do not lower. Scored only on a write attempt.
        self.declare_parameter('min_laser_score', MIN_LASER_SCORE)
        self.declare_parameter('min_valid_beams', 20)
        self.declare_parameter('scan_fresh_sec', 1.5)
        self.declare_parameter('laser_check_period_sec', 3.0)
        # Production Follow uses legacy_freeze — block all follow writes by default.
        self.declare_parameter('block_write_during_follow', True)
        self.declare_parameter('enabled', True)

        # TF listener is created only for a write-gate check, never held in IDLE.
        self._tf: Optional[Buffer] = None
        self._tf_listener = None

        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._amcl_mono: Optional[float] = None
        self._loc_status = 1
        self._follow_en = False
        self._map_name = ''
        self._good_since: Optional[float] = None
        self._anchor: Optional[Tuple[float, float, float]] = None
        self._last_write_mono = 0.0
        self._last_laser_mono = 0.0
        self._last_hash = ''
        self._hash_cache: dict = {}
        self._tf_armed_mono = 0.0
        self._scan: Optional[LaserScan] = None
        self._map: Optional[OccupancyGrid] = None
        self._scan_sub = None
        self._map_sub = None
        self._sensors_armed_mono = 0.0

        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, _AMCL_QOS)
        self.create_subscription(Int8, '/xw/localization_status', self._on_status, _LATCH)
        self.create_subscription(Bool, '/xw/follow/enable', self._on_follow, _LATCH)
        self.create_subscription(String, '/xw/nav/map_name', self._on_map, _LATCH)
        self._diag_pub = self.create_publisher(String, '/xw/localization/last_good_write', 10)

        hz = min(1.0, max(0.2, float(self.get_parameter('tick_hz').value)))
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f'last_good_pose writer ready maps_dir={self.get_parameter("maps_dir").value} '
            f'tick_hz<={hz:.2f} stable_sec={self.get_parameter("stable_sec").value} '
            f'block_follow={self.get_parameter("block_write_during_follow").value} '
            f'min_laser={float(self.get_parameter("min_laser_score").value):.2f} '
            'IDLE: no TF/scan/map; laser only on write attempt'
        )

    def _now(self) -> float:
        return time.monotonic()

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg
        self._amcl_mono = time.monotonic()

    def _on_status(self, msg: Int8) -> None:
        self._loc_status = int(msg.data)

    def _on_follow(self, msg: Bool) -> None:
        self._follow_en = bool(msg.data)
        if self._follow_en:
            self._good_since = None
            self._anchor = None

    def _on_map(self, msg: String) -> None:
        name = (msg.data or '').strip()
        if name and name != self._map_name:
            self._map_name = name
            self._good_since = None
            self._anchor = None
            self.get_logger().info(f'active map={name}')

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

    def _tf_fresh(self, parent: str, child: str) -> bool:
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

    def _pose_tuple(self) -> Optional[Tuple[float, float, float]]:
        if self._amcl is None:
            return None
        p = self._amcl.pose.pose
        return (float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))

    def _cov_xy_yaw(self) -> Tuple[float, float]:
        if self._amcl is None:
            return 999.0, 999.0
        c = self._amcl.pose.covariance
        return max(float(c[0]), float(c[7])), float(c[35])

    def _gates_ok(self) -> Tuple[bool, str]:
        if not bool(self.get_parameter('enabled').value):
            return False, 'disabled'
        if self._loc_status != 0:
            return False, f'status={self._loc_status}'
        if bool(self.get_parameter('block_write_during_follow').value) and self._follow_en:
            return False, 'follow_freeze_block'
        if not self._map_name:
            return False, 'no_active_map'
        if self._amcl is None or self._amcl_mono is None:
            return False, 'no_amcl'
        # Stationary AMCL does not republish. status==0 keeps the last pose valid.
        if (self._now() - self._amcl_mono) > 30.0:
            return False, 'amcl_stale'
        xy, yaw = self._cov_xy_yaw()
        if xy >= float(self.get_parameter('cov_xy_good').value):
            return False, f'cov_xy={xy:.3f}'
        if yaw >= float(self.get_parameter('cov_yaw_good').value):
            return False, f'cov_yaw={yaw:.3f}'
        pose = self._pose_tuple()
        if pose is None:
            return False, 'no_pose'
        if self._anchor is None:
            self._anchor = pose
            return True, 'anchor_set'
        dxy, dyaw = pose_delta(pose, self._anchor)
        if dxy > float(self.get_parameter('stable_xy_m').value) or dyaw > float(
            self.get_parameter('stable_yaw_rad').value
        ):
            self._anchor = pose
            return False, 'pose_moved'
        return True, 'ok'

    def _arm_sensors(self) -> None:
        if self._scan_sub is None:
            self._scan_sub = self.create_subscription(LaserScan, '/scan', self._on_scan, 5)
        if self._map_sub is None:
            self._map_sub = self.create_subscription(OccupancyGrid, '/map', self._on_map_grid, _LATCH)

    def _disarm_sensors(self) -> None:
        if self._scan_sub is not None:
            self.destroy_subscription(self._scan_sub)
            self._scan_sub = None
        if self._map_sub is not None:
            self.destroy_subscription(self._map_sub)
            self._map_sub = None
        self._scan = None
        self._map = None

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_map_grid(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _scan_fresh(self) -> bool:
        if self._scan is None:
            return False
        stamp = self._scan.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return True
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds * 1e-9
        return age < float(self.get_parameter('scan_fresh_sec').value)

    def _scan_stamp(self) -> float:
        if self._scan is None:
            return 0.0
        st = self._scan.header.stamp
        return float(st.sec) + float(st.nanosec) * 1e-9

    def _publish_diag(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'), default=str)
        self._diag_pub.publish(msg)

    def _reject_laser(self, pose: Tuple[float, float, float], laser: dict, why: str) -> None:
        """Do not write. Never delete or overwrite a previous laser_verified pose."""
        existing = read_last_good_pose(str(self.get_parameter('maps_dir').value), self._map_name)
        kept = bool(existing and existing.laser_verified)
        self._publish_diag({
            'event': 'last_good_write_rejected',
            'reason': 'laser_inconsistent',
            'detail': why,
            'laser_score': laser.get('laser_score'),
            'matched_ratio': laser.get('matched_ratio'),
            'valid_beams': laser.get('valid_beams'),
            'pose': {'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
            'kept_previous_laser_verified': kept,
        })
        self.get_logger().warn(
            f'last_good_write_rejected: laser_inconsistent score={laser.get("laser_score")} '
            f'kept_verified={kept}'
        )

    def _tf_ready_now(self) -> bool:
        map_f = str(self.get_parameter('map_frame').value)
        odom_f = str(self.get_parameter('odom_frame').value)
        base_f = str(self.get_parameter('base_frame').value)
        return self._tf_fresh(map_f, odom_f) and self._tf_fresh(map_f, base_f)

    def _tick(self) -> None:
        ok, reason = self._gates_ok()
        now = self._now()
        if not ok:
            if reason != 'anchor_set':
                self._good_since = None
            elif self._good_since is None:
                self._good_since = now
            if self._tf is not None:
                self._disarm_tf()
            if self._scan_sub is not None:
                self._disarm_sensors()
            return
        if self._good_since is None:
            self._good_since = now
            return
        need = float(self.get_parameter('stable_sec').value)
        if now - self._good_since < need:
            return
        if now - self._last_write_mono < float(self.get_parameter('min_write_interval_sec').value):
            return
        pose = self._pose_tuple()
        if pose is None:
            return
        # Arm TF on this tick; check on a later tick so the executor can fill the buffer.
        if self._tf is None:
            self._arm_tf()
            self._tf_armed_mono = now
            return
        if not self._tf_ready_now():
            if now - self._tf_armed_mono > 2.0:
                self.get_logger().warn('write skipped: tf_map_odom/base not fresh')
                self._disarm_tf()
            return
        # Laser only after cheap gates + stable + TF, and not every tick.
        if now - self._last_laser_mono < float(self.get_parameter('laser_check_period_sec').value):
            if self._tf is not None:
                self._disarm_tf()
            return
        if self._scan_sub is None:
            self._arm_sensors()
            self._sensors_armed_mono = now
            return
        if self._scan is None or self._map is None or not self._scan_fresh():
            if now - self._sensors_armed_mono > 2.0:
                self.get_logger().warn('write skipped: scan/map not fresh')
                self._last_laser_mono = now
                self._disarm_tf()
                self._disarm_sensors()
            return
        min_score = float(self.get_parameter('min_laser_score').value)
        if min_score < MIN_LASER_SCORE:
            min_score = MIN_LASER_SCORE
        laser = verify_pose_with_laser(
            pose,
            self._scan,
            self._map,
            min_score=min_score,
            min_valid_beams=int(self.get_parameter('min_valid_beams').value),
        )
        self._last_laser_mono = now
        scan_stamp = self._scan_stamp()
        self._disarm_tf()
        self._disarm_sensors()
        beams = int(laser.get('valid_beams') or 0)
        if not laser.get('ok') or beams < int(self.get_parameter('min_valid_beams').value):
            self._reject_laser(pose, laser, str(laser.get('reason') or 'laser_gate'))
            return
        maps_dir = str(self.get_parameter('maps_dir').value)
        h = compute_map_hash_cached(maps_dir, self._map_name, self._hash_cache)
        if not h:
            self.get_logger().warn('write skipped: map_hash_unavailable')
            return
        self._last_hash = h
        xy, yaw_c = self._cov_xy_yaw()
        q = quality_from_cov(
            xy,
            xy,
            yaw_c,
            float(self.get_parameter('cov_xy_good').value),
            float(self.get_parameter('cov_yaw_good').value),
        )
        rec = LastGoodPose(
            map_name=self._map_name,
            map_hash=self._last_hash,
            timestamp=time.time(),
            x=pose[0],
            y=pose[1],
            yaw=pose[2],
            covariance=[xy, xy, yaw_c],
            source='amcl',
            quality=q,
            laser_verified=True,
            laser_score_at_write=float(laser.get('laser_score') or 0.0),
            laser_matched_ratio=float(laser.get('matched_ratio') or 0.0),
            laser_valid_beams=beams,
            scan_stamp=scan_stamp,
        )
        try:
            path = write_last_good_pose(maps_dir, rec)
            self._last_write_mono = now
            self._publish_diag({
                'event': 'last_good_write',
                'laser_verified': True,
                'laser_score_at_write': rec.laser_score_at_write,
                'matched_ratio': rec.laser_matched_ratio,
                'valid_beams': rec.laser_valid_beams,
                'pose': {'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
            })
            self.get_logger().info(
                f'wrote last_good_pose {path} xy=({pose[0]:.2f},{pose[1]:.2f}) '
                f'q={q:.2f} laser={rec.laser_score_at_write:.3f} hash={self._last_hash[:8]}…'
            )
        except OSError as exc:
            self.get_logger().error(f'write failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LastGoodPoseWriter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
