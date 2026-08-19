#!/usr/bin/env python3
"""Laser-Lock Dock: Nav2 staging → lidar barcode in robot frame → odom lock → rear dock."""

from __future__ import annotations

import json
import math
import threading
import time
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, String
from tf2_ros import Buffer, TransformException, TransformListener

from xw_interfaces.msg import PowerState, TaskProgress, TaskResult
from xw_interfaces.srv import WaypointManage
from xw_recharge.reflaction_detector import (
    DetectionTracker,
    DetectorParams,
    LaserChargerDetection,
    ReflactionDetector,
    in_base_window,
    transform_xy_yaw,
    wrap_angle,
)


class Phase(str, Enum):
    IDLE = 'idle'
    PREP = 'prep'
    NAV = 'nav'
    DETECT = 'detect'
    SWEEP = 'sweep'
    ALIGN = 'align'
    FLIP = 'flip'
    COMMIT = 'commit'
    RETRY = 'retry'
    SUCCESS = 'success'
    FAIL = 'fail'


PHASE_ZH = {
    Phase.IDLE: '待命',
    Phase.PREP: '准备',
    Phase.NAV: '前往接近点',
    Phase.DETECT: '认桩中',
    Phase.SWEEP: '短扫认桩',
    Phase.ALIGN: '中心线对准',
    Phase.FLIP: '掉头',
    Phase.COMMIT: '贴桩中',
    Phase.RETRY: '回退重试',
    Phase.SUCCESS: '已对接充电',
    Phase.FAIL: '失败',
}


def _yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RechargeNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_recharge')
        self._declare()
        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        p = DetectorParams(
            intensity_threshold=float(self.get_parameter('laser_intensity_threshold').value),
            code=tuple(float(x) for x in self.get_parameter('laser_code').value),
            code_tol=float(self.get_parameter('laser_code_tol').value),
        )
        self._detector = ReflactionDetector(p)
        self._tracker = DetectionTracker(
            int(self.get_parameter('laser_confirm_frames').value),
            float(self.get_parameter('laser_confirm_std_m').value),
        )

        self._phase = Phase.IDLE
        self._enabled = False
        self._message = '待命'
        self._retries = 0
        self._result = ''
        self._command_id = ''
        self._session_t0 = 0.0
        self._phase_t0 = 0.0
        self._map_name = ''
        self._loc = 1
        self._nav_active = False
        self._latest_scan: Optional[LaserScan] = None
        self._power = PowerState()
        self._charge_stable_t0 = 0.0
        self._charger: Optional[Tuple[float, float, float]] = None
        self._staging: Optional[Tuple[float, float, float]] = None
        self._dock_odom: Optional[Tuple[float, float, float]] = None
        self._align_odom: Optional[Tuple[float, float, float]] = None
        self._nav_ok: Optional[bool] = None
        self._nav_handle = None
        self._sweep_yaw0 = 0.0
        self._sweep_step = 0
        self._motion_yaw0 = 0.0
        self._motion_xy0 = (0.0, 0.0)
        self._retry_from = ''
        self._retry_step = 0
        self._last_det: Optional[LaserChargerDetection] = None

        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._cmd_pub = self.create_publisher(Twist, str(self.get_parameter('cmd_topic').value), 10)
        self._mode_pub = self.create_publisher(Int8, '/xw/chassis/charge_mode', latch)
        self._status_pub = self.create_publisher(String, '/xw/recharge/status', latch)
        self._staging_pub = self.create_publisher(PoseStamped, '/xw/recharge/staging', 5)
        self._det_pub = self.create_publisher(PoseStamped, '/xw/recharge/detection', 5)
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 5)

        self.create_subscription(Bool, '/xw/recharge/enable', self._on_enable, latch, callback_group=self._cb)
        self.create_subscription(Bool, '/xw/nav/enable', self._on_nav_enable, latch, callback_group=self._cb)
        self.create_subscription(String, '/xw/nav/map_name', self._on_map_name, latch, callback_group=self._cb)
        self.create_subscription(Int8, '/xw/localization_status', self._on_loc, latch, callback_group=self._cb)
        self.create_subscription(PowerState, '/xw/power', self._on_power, 10, callback_group=self._cb)
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            10,
            callback_group=self._cb,
        )

        self._wp_cli = self.create_client(WaypointManage, '/xw/map/waypoint', callback_group=self._cb)
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb
        )

        hz = float(self.get_parameter('control_hz').value)
        self.create_timer(1.0 / max(hz, 5.0), self._tick, callback_group=self._cb)
        self._set_charge_mode(0)
        self._publish_status()
        self.get_logger().info('xw_recharge ready (Laser-Lock Dock)')

    def _declare(self) -> None:
        defaults = {
            'scan_topic': '/scan',
            'lidar_frame': 'lidar_link',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'cmd_topic': '/xw/cmd/recharge',
            'approach_standoff': 0.70,
            'nav_xy_tol': 0.25,
            'align_standoff': 0.45,
            'lateral_ok': 0.04,
            'yaw_ok_deg': 8.0,
            'contact_distance': 0.40,
            'reverse_speed': 0.04,
            'align_linear': 0.10,
            'align_angular': 0.28,
            'flip_omega': 0.35,
            'sweep_omega': 0.25,
            'sweep_angle_deg': 40.0,
            'retreat_speed': 0.08,
            'retreat_distance': 0.25,
            'max_retries': 3,
            'detect_timeout_sec': 8.0,
            'align_timeout_sec': 25.0,
            'commit_timeout_sec': 20.0,
            'session_timeout_sec': 180.0,
            'loc2_wait_sec': 15.0,
            'charging_stable_sec': 1.5,
            'control_hz': 20.0,
            'laser_intensity_threshold': 200.0,
            'laser_code': [0.06, 0.025, 0.08, 0.025, 0.06],
            'laser_code_tol': 0.02,
            'laser_confirm_frames': 3,
            'laser_confirm_std_m': 0.03,
            'laser_window_min_x': 0.25,
            'laser_window_max_x': 1.5,
            'laser_window_min_y': -0.8,
            'laser_window_max_y': 0.8,
            'low_battery_auto': False,
            'low_battery_percent': 12.5,
            'diagnose_always': False,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _on_nav_enable(self, msg: Bool) -> None:
        self._nav_active = bool(msg.data)
        if not self._nav_active and self._enabled:
            self._fail('导航已关闭', canceled=True)

    def _on_map_name(self, msg: String) -> None:
        name = (msg.data or '').strip()
        if name:
            self._map_name = name

    def _on_loc(self, msg: Int8) -> None:
        self._loc = int(msg.data)

    def _on_power(self, msg: PowerState) -> None:
        self._power = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        if bool(self._p('diagnose_always')):
            self._try_detect(publish=True)

    def _on_enable(self, msg: Bool) -> None:
        if bool(msg.data):
            self._start('api')
        else:
            if self._enabled or self._phase not in (Phase.IDLE,):
                self._stop('canceled', canceled=True)

    def _start(self, reason: str) -> None:
        with self._lock:
            if self._phase == Phase.SUCCESS and self._power.charging:
                return
            if self._enabled and self._phase not in (Phase.IDLE, Phase.FAIL, Phase.SUCCESS):
                return
        if not self._nav_active:
            self._fail('请先进入导航', start_fail=True)
            return
        if self._loc == 3:
            self._fail('定位需重设（状态3）', start_fail=True)
            return
        if self._power.charging:
            self._enabled = True
            self._command_id = f'recharge-{int(time.time())}'
            self._enter(Phase.SUCCESS, '已在充电')
            self._emit_result(0, 'already charging')
            return
        charger = self._load_charger()
        if charger is None:
            self._fail('缺少 charger 航点', start_fail=True)
            return
        self._charger = charger
        standoff = float(self._p('approach_standoff'))
        yaw = charger[2]
        self._staging = (
            charger[0] - standoff * math.cos(yaw),
            charger[1] - standoff * math.sin(yaw),
            yaw,
        )
        self._publish_staging()
        self._enabled = True
        self._retries = 0
        self._result = ''
        self._command_id = f'recharge-{int(time.time())}'
        self._session_t0 = time.monotonic()
        self._tracker.reset()
        self._dock_odom = None
        self.get_logger().info(f'recharge start ({reason}) staging={self._staging}')
        self._enter(Phase.PREP, '检查定位与接近点')

    def _stop(self, message: str, canceled: bool = False, success: bool = False) -> None:
        was_enabled = self._enabled
        self._enabled = False
        self._set_charge_mode(0)
        self._cmd(0.0, 0.0)
        self._cancel_nav()
        self._tracker.reset()
        if success:
            self._phase = Phase.SUCCESS
            self._message = message
            self._result = 'success'
            self._emit_result(0, message)
        elif canceled:
            self._phase = Phase.IDLE
            self._message = '已取消'
            self._result = 'canceled'
            if was_enabled:
                self._emit_result(2, message)
        else:
            self._phase = Phase.FAIL
            self._message = message
            self._result = 'fail'
            self._emit_result(1, message)
        self._publish_status()

    def _fail(self, message: str, start_fail: bool = False, canceled: bool = False) -> None:
        self.get_logger().error(f'recharge fail: {message}')
        if start_fail:
            self._enabled = False
            self._phase = Phase.FAIL
            self._message = message
            self._result = 'fail'
            self._command_id = self._command_id or f'recharge-{int(time.time())}'
            self._emit_progress()
            self._emit_result(1, message)
            self._publish_status()
            return
        self._stop(message, canceled=canceled)

    def _enter(self, phase: Phase, message: str) -> None:
        self._phase = phase
        self._message = message
        self._phase_t0 = time.monotonic()
        self._cmd(0.0, 0.0)
        self._emit_progress()
        self._publish_status()
        self.get_logger().info(f'phase {phase.value}: {message}')

    def _tick(self) -> None:
        if self._phase in (Phase.IDLE, Phase.FAIL):
            self._cmd(0.0, 0.0)
            self._publish_status()
            return
        if self._phase == Phase.SUCCESS:
            self._cmd(0.0, 0.0)
            self._set_charge_mode(0)
            self._publish_status()
            return
        if not self._enabled:
            return
        now = time.monotonic()
        if now - self._session_t0 > float(self._p('session_timeout_sec')):
            self._fail('会话超时')
            return
        try:
            {
                Phase.PREP: self._run_prep,
                Phase.NAV: self._run_nav,
                Phase.DETECT: self._run_detect,
                Phase.SWEEP: self._run_sweep,
                Phase.ALIGN: self._run_align,
                Phase.FLIP: self._run_flip,
                Phase.COMMIT: self._run_commit,
                Phase.RETRY: self._run_retry,
            }[self._phase]()
        except KeyError:
            pass
        self._publish_status()

    def _run_prep(self) -> None:
        if self._loc == 2:
            if time.monotonic() - self._phase_t0 > float(self._p('loc2_wait_sec')):
                self._fail('定位自愈超时')
                return
            self._message = '等待定位自愈'
            return
        pose = self._pose_xyyaw(str(self._p('map_frame')))
        if pose and self._staging:
            dist = math.hypot(pose[0] - self._staging[0], pose[1] - self._staging[1])
            if dist < float(self._p('nav_xy_tol')) + 0.15:
                self._enter(Phase.DETECT, '已在接近点，开始认桩')
                return
        self._nav_ok = None
        self._enter(Phase.NAV, '前往接近点')
        threading.Thread(target=self._nav_worker, daemon=True).start()

    def _run_nav(self) -> None:
        if self._nav_ok is True:
            self._enter(Phase.DETECT, '到达接近点，激光认桩')
            return
        if self._nav_ok is False:
            self._fail('接近点导航失败')
            return
        pose = self._pose_xyyaw(str(self._p('map_frame')))
        if pose and self._staging:
            dist = math.hypot(pose[0] - self._staging[0], pose[1] - self._staging[1])
            self._message = f'前往接近点 · 剩余 {dist:.1f}m'

    def _run_detect(self) -> None:
        det = self._try_detect(publish=True)
        if det is not None and self._lock_dock(det):
            self._enter(Phase.ALIGN, '桩位已锁定，中心线对准')
            return
        if time.monotonic() - self._phase_t0 > float(self._p('detect_timeout_sec')):
            pose = self._pose_xyyaw(str(self._p('odom_frame')))
            self._sweep_yaw0 = pose[2] if pose else 0.0
            self._sweep_step = 0
            self._enter(Phase.SWEEP, '激光未锁定，短扫认桩')
            return
        self._message = '认桩中 · 激光未锁定'
        self._cmd(0.0, 0.0)

    def _run_sweep(self) -> None:
        det = self._try_detect(publish=True)
        if det is not None and self._lock_dock(det):
            self._enter(Phase.ALIGN, '短扫锁定桩位')
            return
        pose = self._pose_xyyaw(str(self._p('odom_frame')))
        if pose is None:
            return
        lim = math.radians(float(self._p('sweep_angle_deg')))
        w = float(self._p('sweep_omega'))
        dyaw = wrap_angle(pose[2] - self._sweep_yaw0)
        if self._sweep_step == 0:
            self._cmd(0.0, w)
            if dyaw > lim * 0.9:
                self._sweep_step = 1
                self._sweep_yaw0 = pose[2]
        elif self._sweep_step == 1:
            self._cmd(0.0, -w)
            if dyaw < -lim * 1.8:
                self._sweep_step = 2
                self._sweep_yaw0 = pose[2]
        else:
            self._cmd(0.0, w)
            if abs(dyaw) < 0.08 or time.monotonic() - self._phase_t0 > 20.0:
                self._fail('激光未检出反光条')

    def _run_align(self) -> None:
        det = self._try_detect(publish=True)
        if det is not None:
            self._lock_dock(det)
        if time.monotonic() - self._phase_t0 > float(self._p('align_timeout_sec')):
            self._retry_from = 'align'
            self._begin_retry('对准超时')
            return
        pose = self._pose_xyyaw(str(self._p('odom_frame')))
        if pose is None or self._align_odom is None:
            return
        gx, gy, gyaw = self._align_odom
        rx, ry, ryaw = pose
        dx, dy = gx - rx, gy - ry
        c, s = math.cos(ryaw), math.sin(ryaw)
        ex = c * dx + s * dy
        ey = -s * dx + c * dy
        eyaw = wrap_angle(gyaw - ryaw)
        dist = math.hypot(ex, ey)
        v_lim = float(self._p('align_linear'))
        w_lim = float(self._p('align_angular'))
        lat_ok = float(self._p('lateral_ok'))
        yaw_ok = math.radians(float(self._p('yaw_ok_deg')))
        if dist > 0.07:
            heading = math.atan2(ey, ex)
            if abs(heading) > 0.28:
                self._cmd(0.0, max(-w_lim, min(w_lim, 1.4 * heading)))
            else:
                v = max(-v_lim, min(v_lim, 0.6 * ex if abs(ex) > 0.04 else 0.05))
                w = max(-w_lim, min(w_lim, 1.8 * ey))
                self._cmd(max(0.0, v), w)
            self._message = f'中心线对准 · 横偏 {ey*100:.0f}cm'
            return
        if abs(eyaw) > yaw_ok:
            self._cmd(0.0, max(-w_lim, min(w_lim, 1.2 * eyaw)))
            self._message = '中心线对准 · 转正车头'
            return
        if abs(ey) > lat_ok:
            self._cmd(0.0, max(-w_lim, min(w_lim, 1.5 * ey)))
            return
        self._motion_yaw0 = ryaw
        self._enter(Phase.FLIP, '对准完成，掉头贴桩')

    def _run_flip(self) -> None:
        pose = self._pose_xyyaw(str(self._p('odom_frame')))
        if pose is None:
            return
        turned = abs(wrap_angle(pose[2] - self._motion_yaw0))
        if turned > math.pi - 0.10:
            self._cmd(0.0, 0.0)
            self._charge_stable_t0 = 0.0
            pose2 = self._pose_xyyaw(str(self._p('odom_frame')))
            self._motion_xy0 = (pose2[0], pose2[1]) if pose2 else (0.0, 0.0)
            self._set_charge_mode(1)
            self._enter(Phase.COMMIT, '倒车贴桩')
            return
        w = float(self._p('flip_omega'))
        self._cmd(0.0, w)

    def _run_commit(self) -> None:
        if self._power.charging:
            if self._charge_stable_t0 <= 0.0:
                self._charge_stable_t0 = time.monotonic()
            elif time.monotonic() - self._charge_stable_t0 >= float(self._p('charging_stable_sec')):
                self._succeed()
                return
        else:
            self._charge_stable_t0 = 0.0
        if time.monotonic() - self._phase_t0 > float(self._p('commit_timeout_sec')):
            self._retry_from = 'commit'
            self._begin_retry('贴桩超时未上电')
            return
        pose = self._pose_xyyaw(str(self._p('odom_frame')))
        moved = 0.0
        if pose:
            moved = math.hypot(pose[0] - self._motion_xy0[0], pose[1] - self._motion_xy0[1])
        mock = str(self._power.detail or '').startswith('mock')
        if mock and moved >= float(self._p('contact_distance')) * 0.85:
            self._succeed('模拟贴桩到位')
            return
        self._set_charge_mode(1)
        self._cmd(-abs(float(self._p('reverse_speed'))), 0.0)
        self._message = f'贴桩中 · 第 {self._retries + 1} 次'

    def _begin_retry(self, why: str) -> None:
        self._retries += 1
        if self._retries > int(self._p('max_retries')):
            self._fail(why + ' · 重试耗尽')
            return
        self._set_charge_mode(0)
        pose = self._pose_xyyaw(str(self._p('odom_frame')))
        self._motion_xy0 = (pose[0], pose[1]) if pose else (0.0, 0.0)
        self._motion_yaw0 = pose[2] if pose else 0.0
        self._retry_step = 0
        self._tracker.reset()
        self._enter(Phase.RETRY, f'{why} · 回退重试 {self._retries}')

    def _run_retry(self) -> None:
        pose = self._pose_xyyaw(str(self._p('odom_frame')))
        if pose is None:
            return
        moved = math.hypot(pose[0] - self._motion_xy0[0], pose[1] - self._motion_xy0[1])
        spd = float(self._p('retreat_speed'))
        dist = float(self._p('retreat_distance'))
        if self._retry_step == 0:
            # After commit, robot faces away: +vx leaves the dock. After align fail, still faces dock: -vx backs off.
            vx = spd if self._retry_from == 'commit' else -spd
            self._cmd(vx, 0.0)
            if moved >= dist:
                self._cmd(0.0, 0.0)
                if self._retry_from == 'commit':
                    self._retry_step = 1
                    self._motion_yaw0 = pose[2]
                else:
                    self._enter(Phase.DETECT, '回退后重新认桩')
            return
        turned = abs(wrap_angle(pose[2] - self._motion_yaw0))
        if turned > math.pi - 0.12:
            self._enter(Phase.DETECT, '已转回对桩，重新认桩')
            return
        self._cmd(0.0, float(self._p('flip_omega')))

    def _succeed(self, message: str = '已对接充电') -> None:
        self._set_charge_mode(0)
        self._cmd(0.0, 0.0)
        self._seed_initialpose()
        self._enabled = True
        self._phase = Phase.SUCCESS
        self._message = message
        self._result = 'success'
        self._emit_progress()
        self._emit_result(0, message)
        self._publish_status()
        self.get_logger().info(f'recharge success: {message}')

    def _seed_initialpose(self) -> None:
        if not self._charger:
            return
        x, y, yaw_wall = self._charger
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self._p('map_frame'))
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = _yaw_to_quat(wrap_angle(yaw_wall + math.pi))
        msg.pose.covariance[0] = 0.08
        msg.pose.covariance[7] = 0.08
        msg.pose.covariance[35] = 0.05
        self._initialpose_pub.publish(msg)

    def _try_detect(self, publish: bool = False) -> Optional[LaserChargerDetection]:
        scan = self._latest_scan
        if scan is None or not scan.intensities:
            self._tracker.update(None)
            return None
        raw = self._detector.detect(
            list(scan.ranges),
            list(scan.intensities),
            float(scan.angle_min),
            float(scan.angle_increment),
            frame_id=scan.header.frame_id or str(self._p('lidar_frame')),
        )
        if raw is None:
            self._tracker.update(None)
            return None
        src = raw.frame_id or str(self._p('lidar_frame'))
        try:
            tf = self._tf.lookup_transform(
                str(self._p('base_frame')), src, Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException:
            self._tracker.update(None)
            return None
        bx, by, byaw = transform_xy_yaw(
            raw.x, raw.y, raw.yaw,
            tf.transform.translation.x,
            tf.transform.translation.y,
            _quat_to_yaw(tf.transform.rotation),
        )
        if not in_base_window(
            bx, by,
            float(self._p('laser_window_min_x')),
            float(self._p('laser_window_max_x')),
            float(self._p('laser_window_min_y')),
            float(self._p('laser_window_max_y')),
        ):
            self._tracker.update(None)
            return None
        base = LaserChargerDetection(
            x=bx, y=by, yaw=byaw, range=math.hypot(bx, by),
            matched_segments=raw.matched_segments, frame_id=str(self._p('base_frame')),
        )
        locked = self._tracker.update(base)
        if locked and publish:
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = str(self._p('base_frame'))
            ps.pose.position.x = locked.x
            ps.pose.position.y = locked.y
            ps.pose.orientation = _yaw_to_quat(locked.yaw)
            self._det_pub.publish(ps)
            self._last_det = locked
        return locked

    def _lock_dock(self, det: LaserChargerDetection) -> bool:
        pose = self._pose_xyyaw(str(self._p('odom_frame')))
        if pose is None:
            return False
        dx, dy, dyaw = transform_xy_yaw(det.x, det.y, det.yaw, pose[0], pose[1], pose[2])
        self._dock_odom = (dx, dy, dyaw)
        standoff = float(self._p('align_standoff'))
        into_x, into_y = math.cos(dyaw), math.sin(dyaw)
        self._align_odom = (
            dx + standoff * into_x,
            dy + standoff * into_y,
            wrap_angle(dyaw + math.pi),
        )
        return True

    def _pose_xyyaw(self, frame: str) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf.lookup_transform(
                frame, str(self._p('base_frame')), Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException:
            return None
        return (
            tf.transform.translation.x,
            tf.transform.translation.y,
            _quat_to_yaw(tf.transform.rotation),
        )

    def _load_charger(self) -> Optional[Tuple[float, float, float]]:
        if not self._wp_cli.wait_for_service(timeout_sec=2.0):
            return None
        req = WaypointManage.Request()
        req.operation = 2
        req.map_name = self._map_name
        fut = self._wp_cli.call_async(req)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and not fut.done():
            time.sleep(0.05)
        if not fut.done() or fut.result() is None:
            return None
        res = fut.result()
        try:
            data = json.loads(res.data_json or '{}')
        except json.JSONDecodeError:
            return None
        wps: List[Dict[str, Any]] = list(data.get('waypoints') or [])
        if isinstance(data.get('charger'), dict):
            wps.append(data['charger'])
        for w in wps:
            name = str(w.get('name') or '').strip().lower()
            if name in ('charger', '充电桩'):
                try:
                    return float(w['x']), float(w['y']), float(w.get('yaw', w.get('theta', 0.0)) or 0.0)
                except (KeyError, TypeError, ValueError):
                    return None
        return None

    def _nav_worker(self) -> None:
        if self._staging is None:
            self._nav_ok = False
            return
        if not self._nav_client.wait_for_server(timeout_sec=15.0):
            self._nav_ok = False
            return
        x, y, yaw = self._staging
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(self._p('map_frame'))
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = _yaw_to_quat(yaw)
        send_fut = self._nav_client.send_goal_async(goal)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not send_fut.done():
            time.sleep(0.05)
        if not send_fut.done():
            self._nav_ok = False
            return
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self._nav_ok = False
            return
        self._nav_handle = gh
        result_fut = gh.get_result_async()
        while not result_fut.done():
            if not self._enabled or self._phase != Phase.NAV:
                try:
                    gh.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                return
            time.sleep(0.1)
        self._nav_handle = None
        result = result_fut.result()
        self._nav_ok = int(getattr(result, 'status', 0)) == 4

    def _cancel_nav(self) -> None:
        gh = self._nav_handle
        self._nav_handle = None
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    def _cmd(self, vx: float, wz: float) -> None:
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self._cmd_pub.publish(t)

    def _set_charge_mode(self, mode: int) -> None:
        msg = Int8()
        msg.data = int(mode)
        self._mode_pub.publish(msg)

    def _publish_staging(self) -> None:
        if not self._staging:
            return
        x, y, yaw = self._staging
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = str(self._p('map_frame'))
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = _yaw_to_quat(yaw)
        self._staging_pub.publish(ps)

    def _status_dict(self) -> Dict[str, Any]:
        stg = None
        if self._staging:
            stg = {'x': self._staging[0], 'y': self._staging[1], 'yaw': self._staging[2]}
        return {
            'enabled': bool(self._enabled) and self._phase not in (Phase.IDLE, Phase.FAIL),
            'active': bool(self._enabled) and self._phase not in (Phase.IDLE, Phase.SUCCESS, Phase.FAIL),
            'phase': self._phase.value,
            'message': self._message,
            'charging': bool(self._power.charging),
            'retries': int(self._retries),
            'result': self._result,
            'staging': stg,
            'label': PHASE_ZH.get(self._phase, self._phase.value),
        }

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self._status_dict(), ensure_ascii=False)
        self._status_pub.publish(msg)

    def _emit_progress(self) -> None:
        p = TaskProgress()
        p.stamp = self.get_clock().now().to_msg()
        p.command_id = self._command_id
        p.capability = 'recharge'
        p.phase = self._phase.value
        p.detail = self._message
        self._progress_pub.publish(p)

    def _emit_result(self, code: int, message: str) -> None:
        r = TaskResult()
        r.stamp = self.get_clock().now().to_msg()
        r.command_id = self._command_id
        r.capability = 'recharge'
        r.code = int(code)
        r.message = message
        r.data_json = json.dumps(self._status_dict(), ensure_ascii=False)
        self._result_pub.publish(r)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RechargeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._cmd(0.0, 0.0)  # noqa: SLF001
        node._set_charge_mode(0)  # noqa: SLF001
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
