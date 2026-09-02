#!/usr/bin/env python3
"""Body-follow session: locked track → realtime visual-servo /xw/cmd/follow.

Default path matches gen1 feel: bearing + distance → Twist every detection
frame. Optional Nav2 dynamic-goal path remains behind use_nav2_follow:=true.
"""

from __future__ import annotations

import math
import threading
import time
from enum import Enum
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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


class FollowSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_follow_session')
        # Standoff / visual-servo (gen1-style)
        self.declare_parameter('desired_follow_distance', 0.9)
        self.declare_parameter('max_linear_x', 0.28)
        self.declare_parameter('max_angular_z', 0.55)
        self.declare_parameter('k_linear', 0.45)
        self.declare_parameter('k_angular', 0.85)
        self.declare_parameter('align_bearing_thr', 0.45)
        self.declare_parameter('stop_deadband_m', 0.12)
        self.declare_parameter('min_follow_distance', 0.45)
        # Optional Nav2 dynamic-goal path (laggy “打点”)
        self.declare_parameter('goal_update_hz', 1.0)
        self.declare_parameter('goal_hysteresis_m', 0.35)
        self.declare_parameter('hfov_deg', 70.0)
        self.declare_parameter('camera_frame', 'camera_front_down_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lost_timeout_s', 2.5)
        self.declare_parameter('search_timeout_s', 8.0)
        self.declare_parameter('search_yaw_rate', 0.35)
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
        self._suspend_nav = False  # True during SEARCH/LOST — don't auto-restart NavigateToPose
        self._last_cmd = Twist()

        # Heavy runtime (TF / tracks / 10Hz tick) is armed only while follow is enabled.
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

        mode = 'nav2-goal' if bool(self.get_parameter('use_nav2_follow').value) else 'visual-servo'
        self.get_logger().info(
            f'follow session ready ({mode}; cam={self.get_parameter("camera_frame").value})'
        )

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
        """Subscribe to tracks (and TF if Nav2 follow) and start tick only while following."""
        if self._tracks_sub is not None:
            return
        if self._use_nav2():
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tracks_sub = self.create_subscription(
            PersonTracks, '/xw/perception/tracks', self._on_tracks, 10, callback_group=self._cb
        )
        self._tick_timer = self.create_timer(0.1, self._tick, callback_group=self._cb)
        self.get_logger().info(
            f'follow runtime armed (tracks+tick{",+tf" if self._use_nav2() else ""})'
        )

    def _disarm_runtime(self) -> None:
        """Drop TF/tracks/tick so idle follow does not burn CPU on /tf."""
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

    def _publish_cmd(self, cmd: Twist) -> None:
        self._last_cmd = cmd
        self._cmd_pub.publish(cmd)

    def _stop_cmd(self) -> None:
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
            self._last_goal_sent = None
            # Soft-cancel point/patrol — free /xw/cmd/nav so visual follow can drive
            self._nav_cancel_pub.publish(Bool(data=True))
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

    def _visual_servo_cmd(self, bearing: float, distance: float) -> Twist:
        """Gen1-style P-control from normalized bearing (-1..1) and depth (m)."""
        out = Twist()
        desired = float(self.get_parameter('desired_follow_distance').value)
        min_d = float(self.get_parameter('min_follow_distance').value)
        deadband = float(self.get_parameter('stop_deadband_m').value)
        max_lin = float(self.get_parameter('max_linear_x').value)
        max_ang = float(self.get_parameter('max_angular_z').value)
        k_lin = float(self.get_parameter('k_linear').value)
        k_ang = float(self.get_parameter('k_angular').value)
        align_thr = float(self.get_parameter('align_bearing_thr').value)

        b = max(-1.0, min(1.0, float(bearing)))
        d = max(0.0, float(distance))

        # +bearing = person on right → turn right → negative angular.z (ROS ENU)
        ang = -k_ang * b
        # Deadzone near center to avoid twitch
        if abs(b) < 0.05:
            ang = 0.0
        out.angular.z = max(-max_ang, min(max_ang, ang))

        # Only advance when roughly centered and beyond standoff
        if d < min_d or d < (desired - deadband):
            out.linear.x = 0.0
        elif abs(b) > align_thr:
            # Turn-first: small creep if far, else rotate in place
            out.linear.x = 0.0 if d < desired + 0.6 else max_lin * 0.25
        else:
            err = d - desired
            if err <= deadband:
                out.linear.x = 0.0
            else:
                # Approach slowdown near standoff (gen1-like)
                scale = min(1.0, err / max(0.4, desired))
                out.linear.x = max(0.0, min(max_lin, k_lin * err * (0.35 + 0.65 * scale)))

        return out

    def _track_to_camera_point(self, bearing: float, distance: float):
        """Optical-ish camera_front_down_link: Z forward, X right, Y down."""
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
        px, py, pz = self._quat_rotate(q.x, q.y, q.z, q.w, x, y, z)
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
        target = None
        for t in msg.tracks:
            if getattr(t, 'is_target', False):
                target = t
                break
        if target is None:
            for t in msg.tracks:
                if t.is_primary:
                    target = t
                    break
        if target is None:
            return

        with self._lock:
            self._last_seen_t = time.monotonic()
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
        else:
            # Realtime visual servo — no map / Nav2
            if self._state in (FollowState.TRACKING, FollowState.COAST):
                self._publish_cmd(self._visual_servo_cmd(target.x, target.distance))

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
        """Optional NavigateToPose + follow_point BT path."""
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
        lost_t = float(self.get_parameter('lost_timeout_s').value)
        search_t = float(self.get_parameter('search_timeout_s').value)
        since = now - self._last_seen_t

        if self._state == FollowState.TRACKING and since > 0.4:
            self._state = FollowState.COAST
            self._emit_progress('coast', self._command_id)
            if not self._use_nav2():
                self._stop_cmd()

        if self._state in (FollowState.TRACKING, FollowState.COAST) and since > lost_t:
            self._state = FollowState.SEARCH
            self._search_started_t = now
            self._search_dir = 1.0
            self._suspend_nav = True
            self._cancel_follow_nav()
            self._emit_progress('search', self._command_id)
            self.get_logger().warn('target lost → search rotate')

        if self._state == FollowState.SEARCH:
            if since <= lost_t * 0.5:
                self._state = FollowState.TRACKING
                self._suspend_nav = False
                self._stop_cmd()
                if (
                    self._use_nav2()
                    and (self._follow_thread is None or not self._follow_thread.is_alive())
                ):
                    self._follow_stop.clear()
                    self._follow_thread = threading.Thread(
                        target=self._follow_nav_loop, daemon=True
                    )
                    self._follow_thread.start()
                return
            if now - self._search_started_t > search_t:
                self._state = FollowState.LOST
                self._suspend_nav = True
                self._stop_cmd()
                self._cancel_follow_nav()
                self._emit_result(1, 'target lost', self._command_id)
                self.get_logger().error('follow LOST — stopping follow task')
                return
            phase = int((now - self._search_started_t) / 2.5)
            yaw = float(self.get_parameter('search_yaw_rate').value)
            cmd = Twist()
            cmd.angular.z = yaw if (phase % 2 == 0) else -yaw
            self._publish_cmd(cmd)

        if self._state == FollowState.LOST:
            self._stop_cmd()

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
