#!/usr/bin/env python3
"""Body-follow session: locked track → realtime visual-servo /xw/cmd/follow.

Default path matches gen1 feel: bearing + distance → smoothed Twist @ 20 Hz.
Optional Nav2 dynamic-goal path remains behind use_nav2_follow:=true.
"""

from __future__ import annotations

import math
import threading
import time
from enum import Enum
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters, GetParameters
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from xw_interfaces.msg import PersonTracks, TaskProgress, TaskResult
from xw_interfaces.srv import SessionControl


class FollowState(str, Enum):
    IDLE = 'idle'
    TRACKING = 'tracking'
    COAST = 'coast'
    SEARCH = 'search'
    LOST = 'lost'


def _yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _slew(prev: float, target: float, max_delta: float) -> float:
    d = target - prev
    if d > max_delta:
        return prev + max_delta
    if d < -max_delta:
        return prev - max_delta
    return target


class FollowSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_follow_session')
        # Standoff / visual-servo (gen1-like speed + slew smoothing)
        self.declare_parameter('bearing_deadzone', 0.03)
        self.declare_parameter('stop_deadband_m', 0.12)
        self.declare_parameter('min_follow_distance', 0.55)
        self.declare_parameter('ema_alpha', 0.55)  # faster meas lock (was washing out yaw)
        self.declare_parameter('cmd_hz', 20.0)
        self.declare_parameter('lin_accel', 0.60)
        self.declare_parameter('lin_decel', 1.40)  # brake faster than accelerate (anti-overshoot)
        self.declare_parameter('ang_accel', 2.80)
        self.declare_parameter('cmd_ema', 0.65)
        self.declare_parameter('coast_hold_s', 1.0)
        self.declare_parameter('enable_search_spin', False)
        # Gen1-style bbox width ratios (PersonTrack.y)
        self.declare_parameter('stop_width_ratio', 0.52)
        self.declare_parameter('start_width_ratio', 0.20)
        self.declare_parameter('use_width_range', True)
        # Keep last good meas while TRACKING; only brake after this gap.
        # Must be > perception period (RGB/NPU often ~2–5 Hz under load).
        self.declare_parameter('fresh_timeout_s', 2.0)
        self.declare_parameter('coast_yaw', True)
        self.declare_parameter('coast_yaw_scale', 0.55)
        self.declare_parameter('desired_follow_distance', 1.15)
        self.declare_parameter('max_linear_x', 0.35)
        self.declare_parameter('max_angular_z', 0.90)
        self.declare_parameter('k_linear', 0.55)
        self.declare_parameter('k_angular', 1.45)
        # Gen1: turn-first — above this |bearing|, only yaw (no forward).
        self.declare_parameter('turn_first_bearing', 0.18)
        # Optional Nav2 dynamic-goal path (laggy “打点”)
        self.declare_parameter('goal_update_hz', 1.0)
        self.declare_parameter('goal_hysteresis_m', 0.35)
        self.declare_parameter('hfov_deg', 70.0)
        self.declare_parameter('camera_frame', 'camera_front_up_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lost_timeout_s', 3.5)
        self.declare_parameter('search_timeout_s', 20.0)
        self.declare_parameter('search_yaw_rate', 0.25)
        self.declare_parameter('use_nav2_follow', False)
        self.declare_parameter('follow_bt_xml', '')

        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active = False
        self._command_id = ''
        self._state = FollowState.IDLE
        self._last_target_pose: Optional[PoseStamped] = None
        self._last_goal_sent: Optional[PoseStamped] = None
        self._last_goal_pub_t = 0.0
        self._last_seen_t = 0.0
        self._search_started_t = 0.0
        self._search_dir = 1.0
        self._nav_goal_handle = None
        self._follow_thread: Optional[threading.Thread] = None
        self._follow_stop = threading.Event()
        self._suspend_nav = False
        self._last_cmd = Twist()
        self._out_lin = 0.0
        self._out_ang = 0.0
        self._smooth_lin = 0.0
        self._smooth_ang = 0.0
        self._filt_bearing = 0.0
        self._filt_distance = 0.0
        self._filt_width = 0.0
        self._raw_bearing = 0.0  # last fresh bearing for yaw (avoid EMA washout)
        self._have_meas = False
        self._last_tick_t = 0.0
        self._last_fresh_det_t = 0.0  # last time we saw a real (non-coast) update

        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        self._tracks_sub = None
        self._tick_timer = None

        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)
        self._goal_update_pub = self.create_publisher(PoseStamped, '/goal_update', 10)
        self._nav_cancel_pub = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self._cmd_pub = self.create_publisher(Twist, '/xw/cmd/follow', 10)

        self.create_subscription(Bool, '/xw/follow/enable', self._on_enable, latch, callback_group=self._cb)
        self.create_service(
            SessionControl, '/xw/session/follow/control', self._on_control, callback_group=self._cb
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb
        )
        self._amcl_set_params = self.create_client(
            SetParameters, '/amcl/set_parameters', callback_group=self._cb
        )
        self._amcl_get_params = self.create_client(
            GetParameters, '/amcl/get_parameters', callback_group=self._cb
        )
        self._amcl_saved: Optional[dict] = None

        mode = 'nav2-goal' if bool(self.get_parameter('use_nav2_follow').value) else 'visual-servo'
        self.get_logger().info(
            f'follow session ready ({mode}; cam={self.get_parameter("camera_frame").value}; '
            f'smooth=ema+slew)'
        )

    def _freeze_amcl(self, freeze: bool) -> None:
        """During visual follow, stop AMCL pose updates so map pose does not drift.

        People in the laser + continuous servo motion inflate AMCL covariance and
        pull the particle cloud. Freezing update_min_* keeps last map→odom.
        """
        if not self._amcl_set_params.service_is_ready():
            return
        try:
            if freeze:
                if self._amcl_get_params.service_is_ready() and self._amcl_saved is None:
                    req = GetParameters.Request()
                    req.names = ['update_min_d', 'update_min_a']
                    fut = self._amcl_get_params.call_async(req)
                    # best-effort; don't block follow arm
                    deadline = time.monotonic() + 0.4
                    while time.monotonic() < deadline and not fut.done():
                        time.sleep(0.02)
                    if fut.done() and fut.result() is not None:
                        vals = fut.result().values
                        if len(vals) >= 2:
                            self._amcl_saved = {
                                'update_min_d': float(vals[0].double_value),
                                'update_min_a': float(vals[1].double_value),
                            }
                req = SetParameters.Request()
                req.parameters = [
                    Parameter('update_min_d', Parameter.Type.DOUBLE, 100.0).to_parameter_msg(),
                    Parameter('update_min_a', Parameter.Type.DOUBLE, 100.0).to_parameter_msg(),
                ]
                self._amcl_set_params.call_async(req)
                self.get_logger().info('AMCL updates frozen for visual follow')
            else:
                d = 0.25
                a = 0.2
                if self._amcl_saved:
                    d = float(self._amcl_saved.get('update_min_d', d))
                    a = float(self._amcl_saved.get('update_min_a', a))
                    self._amcl_saved = None
                req = SetParameters.Request()
                req.parameters = [
                    Parameter('update_min_d', Parameter.Type.DOUBLE, d).to_parameter_msg(),
                    Parameter('update_min_a', Parameter.Type.DOUBLE, a).to_parameter_msg(),
                ]
                self._amcl_set_params.call_async(req)
                self.get_logger().info(f'AMCL updates restored (d={d:.2f} a={a:.2f})')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'AMCL freeze/restore failed: {exc}')

    def _bt_path(self) -> str:
        configured = str(self.get_parameter('follow_bt_xml').value or '').strip()
        if configured and Path(configured).is_file():
            return configured
        try:
            share = get_package_share_directory('xw_nav_session')
            p = Path(share) / 'behavior_trees' / 'follow_point.xml'
            if p.is_file():
                return str(p)
        except Exception:  # noqa: BLE001
            pass
        return ''

    def _on_enable(self, msg: Bool) -> None:
        self._set_active(bool(msg.data), 'enable-topic')

    def _on_control(self, req: SessionControl.Request, res: SessionControl.Response):
        self._command_id = req.command_id or 'follow-svc'
        self._set_active(bool(req.start), 'service')
        res.success = True
        res.message = 'follow started' if self._active else 'follow stopped'
        res.state = self._state.value
        return res

    def _use_nav2(self) -> bool:
        return bool(self.get_parameter('use_nav2_follow').value)

    def _arm_runtime(self) -> None:
        if self._tracks_sub is not None:
            return
        if self._use_nav2():
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tracks_sub = self.create_subscription(
            PersonTracks, '/xw/perception/tracks', self._on_tracks, 10, callback_group=self._cb
        )
        hz = max(10.0, float(self.get_parameter('cmd_hz').value))
        self._tick_timer = self.create_timer(1.0 / hz, self._tick, callback_group=self._cb)
        self._last_tick_t = time.monotonic()
        self.get_logger().info(
            f'follow runtime armed (tracks+{hz:.0f}Hz cmd{",+tf" if self._use_nav2() else ""})'
        )

    def _disarm_runtime(self) -> None:
        if self._tick_timer is not None:
            try:
                self._tick_timer.cancel()
            except Exception:  # noqa: BLE001
                pass
            try:
                self.destroy_timer(self._tick_timer)
            except Exception:  # noqa: BLE001
                pass
            self._tick_timer = None
        if self._tracks_sub is not None:
            try:
                self.destroy_subscription(self._tracks_sub)
            except Exception:  # noqa: BLE001
                pass
            self._tracks_sub = None
        if self._tf_listener is not None:
            try:
                self._tf_listener.unregister()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'tf unregister failed: {exc}')
            self._tf_listener = None
            self._tf_buffer = None
        with self._lock:
            self._last_target_pose = None
            self._last_cmd = Twist()
            self._out_lin = 0.0
            self._out_ang = 0.0
            self._smooth_lin = 0.0
            self._smooth_ang = 0.0
            self._have_meas = False

    def _publish_cmd(self, cmd: Twist) -> None:
        self._last_cmd = cmd
        self._cmd_pub.publish(cmd)

    def _stop_cmd(self) -> None:
        self._out_lin = 0.0
        self._out_ang = 0.0
        self._smooth_lin = 0.0
        self._smooth_ang = 0.0
        self._publish_cmd(Twist())

    def _set_active(self, active: bool, source: str) -> None:
        if active == self._active:
            return
        if active:
            self._arm_runtime()
            self._active = True
            self._follow_stop.clear()
            self._suspend_nav = False
            self._state = FollowState.TRACKING
            self._last_seen_t = time.monotonic()
            self._last_fresh_det_t = time.monotonic()
            self._last_goal_sent = None
            self._have_meas = False
            self._out_lin = 0.0
            self._out_ang = 0.0
            self._smooth_lin = 0.0
            self._smooth_ang = 0.0
            self._nav_cancel_pub.publish(Bool(data=True))
            if not self._use_nav2():
                self._freeze_amcl(True)
            self._emit_progress('follow_start', source)
            self.get_logger().info(f'follow active ({source})')
            if self._use_nav2():
                self._follow_thread = threading.Thread(
                    target=self._follow_nav_loop, daemon=True
                )
                self._follow_thread.start()
        else:
            self._active = False
            self._follow_stop.set()
            self._suspend_nav = True
            self._cancel_follow_nav()
            self._state = FollowState.IDLE
            self._stop_cmd()
            if not self._use_nav2():
                self._freeze_amcl(False)
            self._disarm_runtime()
            self._emit_result(0, 'follow stopped', source)
            self.get_logger().info(f'follow stopped ({source})')

    def _cancel_follow_nav(self) -> None:
        with self._lock:
            gh = self._nav_goal_handle
            self._nav_goal_handle = None
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'follow nav cancel failed: {exc}')

    def _update_filters(self, bearing: float, distance: float, width_ratio: float = 0.0) -> None:
        b = _clamp(float(bearing), -1.0, 1.0)
        d = max(0.0, float(distance))
        w = max(0.0, float(width_ratio))
        alpha = _clamp(float(self.get_parameter('ema_alpha').value), 0.05, 1.0)
        # Yaw uses mostly-raw bearing; range/width keep EMA.
        self._raw_bearing = b
        if not self._have_meas:
            self._filt_bearing = b
            self._filt_distance = d if d > 0.05 else float(self.get_parameter('desired_follow_distance').value)
            self._filt_width = w
            self._have_meas = True
            return
        # Fast track large bearing jumps (side-step) so yaw doesn't lag to ~0.
        if abs(b - self._filt_bearing) > 0.25:
            self._filt_bearing = b
        else:
            self._filt_bearing = (1.0 - alpha) * self._filt_bearing + alpha * b
        if d > 0.05:
            self._filt_distance = (1.0 - alpha) * self._filt_distance + alpha * d
        if w >= 0.05:
            self._filt_width = (1.0 - alpha) * self._filt_width + alpha * w

    def _desired_cmd(
        self, bearing: float, distance: float, width_ratio: float = 0.0
    ) -> Tuple[float, float]:
        """Visual servo: yaw on bearing; range via bbox width (gen1) or meters."""
        desired = float(self.get_parameter('desired_follow_distance').value)
        min_d = float(self.get_parameter('min_follow_distance').value)
        deadband = float(self.get_parameter('stop_deadband_m').value)
        max_lin = float(self.get_parameter('max_linear_x').value)
        max_ang = float(self.get_parameter('max_angular_z').value)
        k_lin = float(self.get_parameter('k_linear').value)
        k_ang = float(self.get_parameter('k_angular').value)
        b_dz = float(self.get_parameter('bearing_deadzone').value)
        use_w = bool(self.get_parameter('use_width_range').value)
        stop_wr = float(self.get_parameter('stop_width_ratio').value)
        start_wr = float(self.get_parameter('start_width_ratio').value)
        turn_first = float(self.get_parameter('turn_first_bearing').value)

        b = _clamp(bearing, -1.0, 1.0)
        d = max(0.0, distance)
        wr = max(0.0, width_ratio)

        if abs(b) < b_dz:
            ang = 0.0
        else:
            b_eff = math.copysign(max(0.0, abs(b) - b_dz), b)
            mag = abs(b_eff)
            # Stronger yaw when close / at image edge (partial body exits FOV fast).
            prox = 1.0 + 1.6 * _clamp((wr - 0.18) / 0.35, 0.0, 1.0) if wr >= 0.05 else 1.0
            edge = 1.0 + 0.9 * _clamp((mag - 0.25) / 0.55, 0.0, 1.0)
            ang = _clamp(
                -k_ang * prox * edge * b_eff * (0.85 + 0.55 * mag),
                -max_ang,
                max_ang,
            )
            # Hard floors so chassis actually rotates
            if mag >= 0.45 and abs(ang) < 0.45:
                ang = math.copysign(0.45, ang)
            elif mag >= 0.22 and abs(ang) < 0.28:
                ang = math.copysign(0.28, ang)
            elif mag >= 0.08 and abs(ang) < 0.16:
                ang = math.copysign(0.16, ang)

        lin = 0.0
        # Gen1 turn-first: do not creep forward while badly misaligned.
        if abs(b) >= turn_first:
            return 0.0, ang

        if use_w and wr >= 0.05:
            soft_wr = max(start_wr + 0.05, stop_wr - 0.10)
            close_by_w = wr >= stop_wr
            if close_by_w or d < min_d:
                lin = 0.0
            elif d > desired + 0.08:
                err = d - desired
                scale = _clamp(err / max(0.35, desired), 0.0, 1.0)
                lin = _clamp(k_lin * err * (0.50 + 0.50 * scale), 0.06, max_lin)
                if wr >= soft_wr:
                    lin *= _clamp((stop_wr - wr) / max(0.05, stop_wr - soft_wr), 0.0, 1.0)
            else:
                err = max(0.0, stop_wr - wr)
                span = max(0.08, stop_wr - start_wr)
                scale = _clamp(err / span, 0.0, 1.0)
                boost = 1.0 if wr <= start_wr else (0.35 + 0.65 * scale)
                lin = _clamp(k_lin * err * 1.05 * boost, 0.06 if scale > 0.10 else 0.0, max_lin)
                if wr >= soft_wr:
                    lin *= _clamp((stop_wr - wr) / max(0.05, stop_wr - soft_wr), 0.0, 0.85)
        else:
            if d < min_d or d <= desired:
                lin = 0.0
            else:
                err = d - desired
                if err < max(0.05, deadband * 0.35):
                    lin = 0.0
                else:
                    scale = _clamp(err / max(0.4, desired), 0.0, 1.0)
                    lin = _clamp(k_lin * err * (0.45 + 0.55 * scale), 0.05, max_lin)

        scale_b = 1.0 - 0.55 * min(1.0, abs(b) / 0.85)
        lin *= max(0.35, scale_b)
        return lin, ang

    def _track_to_camera_point(self, bearing: float, distance: float):
        half_hfov = math.radians(float(self.get_parameter('hfov_deg').value) * 0.5)
        angle = float(bearing) * half_hfov
        person_d = max(0.3, float(distance))
        standoff = float(self.get_parameter('desired_follow_distance').value)
        dist = max(0.25, person_d - standoff)
        x = dist * math.sin(angle)
        y = 0.0
        z = dist * math.cos(angle)
        return x, y, z

    def _pose_in_map(self, bearing: float, distance: float, frame_id: str) -> Optional[PoseStamped]:
        if self._tf_buffer is None:
            return None
        cam_frame = frame_id or str(self.get_parameter('camera_frame').value)
        map_frame = str(self.get_parameter('map_frame').value)
        x, y, z = self._track_to_camera_point(bearing, distance)
        try:
            tf = self._tf_buffer.lookup_transform(
                map_frame, cam_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except Exception as exc:  # noqa: BLE001
            now = time.monotonic()
            if now - getattr(self, '_last_tf_warn', 0.0) > 2.0:
                self._last_tf_warn = now
                self.get_logger().warn(f'TF {cam_frame}→{map_frame} failed: {exc}')
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        px, py, _ = self._quat_rotate(q.x, q.y, q.z, q.w, x, y, z)
        fx, fy, _ = self._quat_rotate(q.x, q.y, q.z, q.w, 0.0, 0.0, 1.0)
        yaw = math.atan2(fy, fx)
        ps = PoseStamped()
        ps.header.frame_id = map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(t.x + px)
        ps.pose.position.y = float(t.y + py)
        ps.pose.position.z = 0.0
        ps.pose.orientation = _yaw_to_quat(yaw)
        return ps

    @staticmethod
    def _quat_rotate(qx, qy, qz, qw, x, y, z):
        ix = qw * x + qy * z - qz * y
        iy = qw * y + qz * x - qx * z
        iz = qw * z + qx * y - qy * x
        iw = -qx * x - qy * y - qz * z
        return (
            ix * qw + iw * -qx + iy * -qz - iz * -qy,
            iy * qw + iw * -qy + iz * -qx - ix * -qz,
            iz * qw + iw * -qz + ix * -qy - iy * -qx,
        )

    def _on_tracks(self, msg: PersonTracks) -> None:
        if not self._active:
            return
        # Stick to locked target only — and ONLY fresh detections.
        # Coast/ghost tracks (low conf) must not refresh bearing or last_seen,
        # otherwise the robot spins forever on a stale angle.
        min_conf = 0.30
        target = None
        for t in msg.tracks:
            if getattr(t, 'is_target', False) and float(t.confidence) >= min_conf:
                target = t
                break
        if target is None:
            return

        # Reject tiny/edge ghosts that still pass as target (chair / noise).
        wr = float(getattr(target, 'y', 0.0) or 0.0)
        bearing = float(target.x)
        if wr < 0.07 and abs(bearing) > 0.60:
            return
        if wr < 0.05:
            return

        with self._lock:
            self._last_seen_t = time.monotonic()
            self._last_fresh_det_t = time.monotonic()
            self._update_filters(bearing, target.distance, wr)
            if self._state in (FollowState.SEARCH, FollowState.LOST, FollowState.COAST):
                was = self._state
                self._state = FollowState.TRACKING
                self._suspend_nav = False
                self._emit_progress('tracking', self._command_id)
                if (
                    self._use_nav2()
                    and was in (FollowState.SEARCH, FollowState.LOST)
                    and (self._follow_thread is None or not self._follow_thread.is_alive())
                ):
                    self._follow_thread = threading.Thread(
                        target=self._follow_nav_loop, daemon=True
                    )
                    self._follow_thread.start()

        if self._use_nav2():
            pose = self._pose_in_map(target.x, target.distance, msg.frame_id)
            if pose is None:
                return
            with self._lock:
                self._last_target_pose = pose
            self._maybe_publish_goal(pose)

    def _maybe_publish_goal(self, pose: PoseStamped) -> None:
        hz = max(0.2, float(self.get_parameter('goal_update_hz').value))
        period = 1.0 / hz
        hyst = float(self.get_parameter('goal_hysteresis_m').value)
        now = time.monotonic()
        if now - self._last_goal_pub_t < period:
            if self._last_goal_sent is not None:
                return
        if self._last_goal_sent is not None:
            dx = pose.pose.position.x - self._last_goal_sent.pose.position.x
            dy = pose.pose.position.y - self._last_goal_sent.pose.position.y
            if (dx * dx + dy * dy) < hyst * hyst:
                if now - self._last_goal_pub_t < period * 2:
                    return
        self._goal_update_pub.publish(pose)
        self._last_goal_sent = pose
        self._last_goal_pub_t = now

    def _follow_nav_loop(self) -> None:
        deadline = time.monotonic() + 8.0
        pose = None
        while time.monotonic() < deadline and not self._follow_stop.is_set() and self._active:
            with self._lock:
                pose = self._last_target_pose
            if pose is not None:
                break
            time.sleep(0.1)
        if pose is None or self._follow_stop.is_set() or not self._active:
            self.get_logger().warn('follow nav: no target pose — stay idle until tracks')
            return

        if not self._nav_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('navigate_to_pose not ready — is Nav2 running?')
            self._state = FollowState.LOST
            self._emit_result(1, 'nav2 not ready for follow', self._command_id)
            return

        bt = self._bt_path()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        if bt:
            goal.behavior_tree = bt
            self.get_logger().info(f'follow BT={bt}')
        else:
            self.get_logger().warn('follow_point.xml missing — using default BT')

        self._goal_update_pub.publish(pose)
        self._last_goal_sent = pose
        self._last_goal_pub_t = time.monotonic()

        send_fut = self._nav_client.send_goal_async(goal)
        wait_deadline = time.monotonic() + 10.0
        while time.monotonic() < wait_deadline and not send_fut.done():
            if self._follow_stop.is_set():
                return
            time.sleep(0.05)
        if not send_fut.done():
            self.get_logger().error('follow send_goal timed out')
            return
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn('follow goal rejected')
            return
        with self._lock:
            self._nav_goal_handle = gh
        self._emit_progress('follow_nav_executing', self._command_id)

        result_fut = gh.get_result_async()
        while not result_fut.done():
            if self._follow_stop.is_set() or not self._active or self._suspend_nav:
                try:
                    gh.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                return
            time.sleep(0.1)
        with self._lock:
            self._nav_goal_handle = None

        status = None
        try:
            if result_fut.done():
                status = int(result_fut.result().status)
        except Exception:  # noqa: BLE001
            status = None

        if (
            self._active
            and not self._follow_stop.is_set()
            and not self._suspend_nav
            and self._state in (FollowState.TRACKING, FollowState.COAST)
        ):
            delay = 1.0 if status == 6 else 0.3
            self.get_logger().info(
                f'follow NavigateToPose ended (status={status}) — restarting in {delay:.1f}s'
            )
            time.sleep(delay)
            if (
                self._active
                and not self._follow_stop.is_set()
                and not self._suspend_nav
            ):
                self._follow_nav_loop()

    def _tick(self) -> None:
        if not self._active:
            return
        now = time.monotonic()
        dt = now - self._last_tick_t if self._last_tick_t > 0 else 0.05
        self._last_tick_t = now
        dt = _clamp(dt, 0.01, 0.1)

        lost_t = float(self.get_parameter('lost_timeout_s').value)
        search_t = float(self.get_parameter('search_timeout_s').value)
        coast_hold = float(self.get_parameter('coast_hold_s').value)
        since = now - self._last_seen_t

        # --- state transitions ---
        if self._state == FollowState.TRACKING and since > coast_hold:
            self._state = FollowState.COAST
            self._emit_progress('coast', self._command_id)

        if self._state in (FollowState.TRACKING, FollowState.COAST) and since > lost_t:
            self._state = FollowState.SEARCH
            self._search_started_t = now
            self._suspend_nav = True
            self._cancel_follow_nav()
            self._emit_progress('search', self._command_id)
            if bool(self.get_parameter('enable_search_spin').value):
                self.get_logger().warn('target lost → gentle search')
            else:
                self.get_logger().warn('target lost → hold (no spin)')

        if self._state == FollowState.SEARCH:
            if since <= coast_hold:
                self._state = FollowState.TRACKING
                self._suspend_nav = False
                if (
                    self._use_nav2()
                    and (self._follow_thread is None or not self._follow_thread.is_alive())
                ):
                    self._follow_stop.clear()
                    self._follow_thread = threading.Thread(
                        target=self._follow_nav_loop, daemon=True
                    )
                    self._follow_thread.start()
            elif now - self._search_started_t > search_t:
                self._state = FollowState.LOST
                self._suspend_nav = True
                self._stop_cmd()
                self._cancel_follow_nav()
                self._emit_result(1, 'target lost', self._command_id)
                self.get_logger().error('follow LOST — waiting for reacquire (enable stays on)')
                return

        if self._state == FollowState.LOST:
            # Quiet wait; _on_tracks will revive to TRACKING
            self._stop_cmd()
            return

        if self._use_nav2():
            # Nav2 path owns motion except SEARCH spin
            if self._state == FollowState.SEARCH and bool(
                self.get_parameter('enable_search_spin').value
            ):
                phase = int((now - self._search_started_t) / 3.0)
                yaw = float(self.get_parameter('search_yaw_rate').value)
                cmd = Twist()
                cmd.angular.z = yaw if (phase % 2 == 0) else -yaw
                self._publish_cmd(cmd)
            return

        # --- visual-servo continuous cmd @ tick rate ---
        lin_acc = float(self.get_parameter('lin_accel').value)
        lin_dec = float(self.get_parameter('lin_decel').value)
        ang_acc = float(self.get_parameter('ang_accel').value)
        cmd_ema = _clamp(float(self.get_parameter('cmd_ema').value), 0.05, 1.0)

        if self._state == FollowState.SEARCH:
            if bool(self.get_parameter('enable_search_spin').value):
                phase = int((now - self._search_started_t) / 3.5)
                yaw = float(self.get_parameter('search_yaw_rate').value)
                tgt_lin, tgt_ang = 0.0, (yaw if (phase % 2 == 0) else -yaw)
            else:
                tgt_lin, tgt_ang = 0.0, 0.0
        elif not self._have_meas:
            tgt_lin, tgt_ang = 0.0, 0.0
        elif self._state == FollowState.COAST or (
            (now - self._last_fresh_det_t) > float(self.get_parameter('fresh_timeout_s').value)
        ):
            # No fresh box: do not drive forward on a ghost range.
            # Still yaw gently toward last bearing so a side-step doesn't exit FOV.
            tgt_lin = 0.0
            tgt_ang = 0.0
            if bool(self.get_parameter('coast_yaw').value) and abs(self._raw_bearing) > 0.10:
                max_ang = float(self.get_parameter('max_angular_z').value)
                k_ang = float(self.get_parameter('k_angular').value)
                scale = float(self.get_parameter('coast_yaw_scale').value)
                tgt_ang = _clamp(
                    -k_ang * scale * self._raw_bearing,
                    -0.70 * max_ang,
                    0.70 * max_ang,
                )
        else:
            # Prefer raw bearing for yaw so side-step doesn't get EMA'd to ~0.
            yaw_b = self._raw_bearing if abs(self._raw_bearing) > 0.05 else self._filt_bearing
            tgt_lin, tgt_ang = self._desired_cmd(
                yaw_b, self._filt_distance, self._filt_width
            )

        # Asymmetric slew: decelerate faster than accelerate (capture showed lx lag).
        lin_rate = lin_dec if abs(tgt_lin) < abs(self._out_lin) - 1e-6 else lin_acc
        self._out_lin = _slew(self._out_lin, tgt_lin, lin_rate * dt)
        self._out_ang = _slew(self._out_ang, tgt_ang, ang_acc * dt)

        # Hard snap to zero when target is stop — kill residual overshoot in EMA.
        if tgt_lin <= 0.01 and self._filt_width >= float(self.get_parameter('stop_width_ratio').value) - 0.02:
            self._out_lin = 0.0
            self._smooth_lin = 0.0

        # Second low-pass on published cmd for silkier chassis feel
        self._smooth_lin = (1.0 - cmd_ema) * self._smooth_lin + cmd_ema * self._out_lin
        self._smooth_ang = (1.0 - cmd_ema) * self._smooth_ang + cmd_ema * self._out_ang

        pub_lin = self._smooth_lin
        pub_ang = self._smooth_ang
        if abs(pub_lin) < 0.015:
            pub_lin = 0.0
            self._smooth_lin = 0.0
            self._out_lin = 0.0
        if abs(pub_ang) < 0.02:
            pub_ang = 0.0
            self._smooth_ang = 0.0
            self._out_ang = 0.0

        cmd = Twist()
        cmd.linear.x = float(pub_lin)
        cmd.angular.z = float(pub_ang)
        self._publish_cmd(cmd)

    def _emit_progress(self, phase: str, command_id: str = '') -> None:
        p = TaskProgress()
        p.stamp = self.get_clock().now().to_msg()
        p.command_id = command_id or self._command_id or 'follow'
        p.capability = 'follow'
        p.phase = phase
        self._progress_pub.publish(p)

    def _emit_result(self, code: int, message: str, command_id: str = '') -> None:
        r = TaskResult()
        r.stamp = self.get_clock().now().to_msg()
        r.command_id = command_id or self._command_id or 'follow'
        r.capability = 'follow'
        r.code = int(code)
        r.message = message
        self._result_pub.publish(r)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowSessionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._set_active(False, 'shutdown')  # noqa: SLF001
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
