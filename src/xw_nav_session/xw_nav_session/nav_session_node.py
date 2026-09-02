#!/usr/bin/env python3
"""Navigation session: bring up Nav2, single/multi goal, TaskProgress/Result."""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from xw_interfaces.msg import TaskProgress, TaskResult
from xw_interfaces.srv import SessionControl, WaypointManage


def _yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _point_list_name(map_name: str) -> str:
    name = (map_name or '').strip()
    if name.endswith('_pointList'):
        return name
    return f'{name}_pointList'


class NavSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_nav_session')
        self.declare_parameter('maps_dir', os.environ.get('XW_MAPS', '/ros2_ws/maps'))
        self.declare_parameter('nav2_params', '')
        self.declare_parameter('nav2_launch_pkg', 'xw_nav_session')
        self.declare_parameter('use_nav2', True)

        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._start_lock = threading.Lock()
        self._active = False
        self._command_id = ''
        self._map_name = ''
        self._proc: Optional[subprocess.Popen] = None
        self._proc_log_f = None
        self._nav_goal_handle = None
        self._patrol_thread: Optional[threading.Thread] = None
        self._patrol_stop = threading.Event()
        self._patrol_active = False
        self._follow_en = False
        self._recharge_en = False

        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        self.create_subscription(Bool, '/xw/nav/enable', self._on_enable, latch, callback_group=self._cb)
        self.create_subscription(PoseStamped, '/xw/goal_pose', self._on_goal, 10, callback_group=self._cb)
        self.create_subscription(String, '/xw/nav/patrol_cmd', self._on_patrol_cmd, 10, callback_group=self._cb)
        self.create_subscription(Bool, '/xw/nav/cancel', self._on_cancel, 10, callback_group=self._cb)
        self.create_subscription(
            Bool, '/xw/follow/enable', self._on_follow_enable, latch, callback_group=self._cb
        )
        self.create_subscription(
            Bool, '/xw/recharge/enable', self._on_recharge_enable, latch, callback_group=self._cb
        )
        self.create_service(SessionControl, '/xw/session/nav/control', self._on_control, callback_group=self._cb)

        self._wp_cli = self.create_client(WaypointManage, '/xw/map/waypoint', callback_group=self._cb)
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb
        )
        self._nav_lifecycle_cli = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes',
            callback_group=self._cb,
        )
        self._nav_active_cli = self.create_client(
            Trigger,
            '/lifecycle_manager_navigation/is_active',
            callback_group=self._cb,
        )
        self._loc_lifecycle_cli = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_localization/manage_nodes',
            callback_group=self._cb,
        )
        self._loc_active_cli = self.create_client(
            Trigger,
            '/lifecycle_manager_localization/is_active',
            callback_group=self._cb,
        )

        # map_name from supervisor payload is not on enable topic — publish side-channel
        self.create_subscription(
            String, '/xw/nav/map_name', self._on_map_name, latch, callback_group=self._cb
        )

        self.get_logger().info('nav session ready')

    def _params_file(self) -> str:
        configured = str(self.get_parameter('nav2_params').value or '').strip()
        if configured and Path(configured).is_file():
            return configured
        share = get_package_share_directory('xw_nav_session')
        return str(Path(share) / 'config' / 'nav2_params.yaml')

    def _maps_dir(self) -> Path:
        return Path(str(self.get_parameter('maps_dir').value))

    def _map_yaml_path(self, map_name: str) -> Optional[Path]:
        name = (map_name or '').strip()
        if not name:
            return None
        p = self._maps_dir() / f'{name}.yaml'
        return p if p.is_file() else None

    def _on_map_name(self, msg: String) -> None:
        name = (msg.data or '').strip()
        if name:
            with self._lock:
                self._map_name = name
            self.get_logger().info(f'nav map_name={name}')

    def _on_enable(self, msg: Bool) -> None:
        self._set_active(bool(msg.data), 'enable-topic')

    def _on_control(self, req: SessionControl.Request, res: SessionControl.Response):
        self._command_id = req.command_id or 'nav-svc'
        self._set_active(bool(req.start), 'service')
        res.success = True
        res.message = 'nav started' if self._active else 'nav stopped'
        res.state = 'active' if self._active else 'idle'
        return res

    def _on_cancel(self, msg: Bool) -> None:
        if msg.data:
            # Soft cancel only — never stop Nav2 process here
            self._cancel_navigation('cancel-topic')

    def _on_follow_enable(self, msg: Bool) -> None:
        """Follow task preempts point/patrol; Nav2 process stays up."""
        en = bool(msg.data)
        was = self._follow_en
        self._follow_en = en
        if en and not was:
            self._cancel_navigation('follow-preempt')
            self.get_logger().info('follow on → cancelled point/patrol (Nav2 kept)')
        elif not en and was:
            self.get_logger().info('follow off → point/patrol accepted again')

    def _on_recharge_enable(self, msg: Bool) -> None:
        en = bool(msg.data)
        was = self._recharge_en
        self._recharge_en = en
        if en and not was:
            self._cancel_navigation('recharge-preempt')
            self.get_logger().info('recharge on → cancelled point/patrol (Nav2 kept)')
        elif not en and was:
            self.get_logger().info('recharge off → point/patrol accepted again')

    def _proc_alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _set_active(self, active: bool, source: str) -> None:
        # Serialize start/stop — enable-topic and SessionControl race otherwise
        # and kill each other's Nav2 process group mid-bringup.
        with self._start_lock:
            self._set_active_locked(active, source)

    def _set_active_locked(self, active: bool, source: str) -> None:
        # If UI thinks nav is on but Nav2 died / was killed externally, force restart.
        if active and self._active and self._proc_alive():
            return
        if not active and not self._active and not self._proc_alive():
            # Still sweep orphans left by external launches / crashed process groups.
            self._stop_nav2()
            return

        if active:
            if self._start_nav2():
                with self._lock:
                    self._active = True
                self._emit_progress('active', source)
                self.get_logger().info(f'nav active ({source}) map={self._map_name}')
            else:
                with self._lock:
                    self._active = False
                self._emit_result(1, 'Nav2 start failed', source)
        else:
            self._cancel_navigation(source)
            self._stop_nav2()
            with self._lock:
                self._active = False
            self._emit_result(0, 'stopped', source)
            self.get_logger().info(f'nav stopped ({source})')

    def _start_nav2(self) -> bool:
        if not bool(self.get_parameter('use_nav2').value):
            self.get_logger().warn('use_nav2=false — session only (no Nav2 process)')
            return True

        self._stop_nav2()
        with self._lock:
            map_name = self._map_name
        yaml_path = self._map_yaml_path(map_name)
        if yaml_path is None:
            self.get_logger().error(
                f'map yaml missing for map_name={map_name!r} under {self._maps_dir()}'
            )
            return False

        params = self._params_file()
        if not Path(params).is_file():
            self.get_logger().error(f'nav2 params missing: {params}')
            return False

        log_dir = Path(os.environ.get('XW_WS', '/ros2_ws')) / 'log'
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            pass
        log_path = log_dir / 'nav2_session.launch.log'

        cmd = [
            'ros2', 'launch', 'xw_nav_session', 'nav2.launch.py',
            f'map:={yaml_path}',
            f'params_file:={params}',
            'use_sim_time:=false',
            'autostart:=true',
        ]
        try:
            log_f = open(log_path, 'w', encoding='utf-8')  # noqa: SIM115
            self._proc = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
                stdout=log_f,
                stderr=subprocess.STDOUT,
            )
            self._proc_log_f = log_f
        except OSError as exc:
            self.get_logger().error(f'Popen nav2 failed: {exc}')
            self._proc = None
            return False

        time.sleep(3.0)
        if self._proc.poll() is not None:
            self.get_logger().error(
                f'nav2 exited early code={self._proc.returncode} log={log_path}'
            )
            self._proc = None
            return False
        self.get_logger().info(f'nav2 started pid={self._proc.pid} map={yaml_path}')
        # Seed /initialpose immediately (and once more after 1.5 s): planner and
        # global_costmap activation block until map→base TF exists, which AMCL only
        # publishes once it has received /initialpose.
        if map_name:
            try:
                self._seed_initial_pose(map_name)
            except Exception:  # noqa: BLE001
                pass
            time.sleep(1.5)
            try:
                self._seed_initial_pose(map_name)
            except Exception:  # noqa: BLE001
                pass
        if not self._ensure_nav2_active(deadline_sec=120.0, map_name=map_name):
            self.get_logger().error(f'Nav2 lifecycle did not become active log={log_path}')
            self._stop_nav2()
            return False
        return True

    def _seed_initial_pose(self, map_name: str) -> None:
        """Publish /initialpose from charger waypoint (or first WP) so AMCL can localize."""
        x = 0.0
        y = 0.0
        yaw = 0.0
        source = 'origin'
        charger = self._load_charger_pose(map_name)
        if charger is not None:
            x, y, yaw = charger
            source = 'charger'
        else:
            poses = self._load_waypoint_poses(map_name, None)
            if poses:
                x = float(poses[0].pose.position.x)
                y = float(poses[0].pose.position.y)
                q = poses[0].pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
                source = 'first_waypoint'

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = _yaw_to_quat(yaw)
        msg.pose.covariance[0] = 1.0
        msg.pose.covariance[7] = 1.0
        msg.pose.covariance[35] = 0.5
        self._initialpose_pub.publish(msg)
        self.get_logger().info(
            f'seeded /initialpose from {source}: ({x:.2f},{y:.2f},yaw={yaw:.2f}) '
            f'(large cov — refine on UI if robot is not there)'
        )

    def _load_charger_pose(self, map_name: str) -> Optional[tuple]:
        if not self._wp_cli.wait_for_service(timeout_sec=2.0):
            return None
        req = WaypointManage.Request()
        req.operation = 2
        req.map_name = map_name
        fut = self._wp_cli.call_async(req)
        if not self._wait_future(fut, 5.0) or fut.result() is None:
            return None
        try:
            data = json.loads(fut.result().data_json or '{}')
        except json.JSONDecodeError:
            return None
        charger = data.get('charger')
        if isinstance(charger, dict):
            try:
                return (
                    float(charger['x']),
                    float(charger['y']),
                    float(charger.get('yaw', charger.get('theta', 0.0)) or 0.0),
                )
            except (KeyError, TypeError, ValueError):
                pass
        for w in data.get('waypoints') or []:
            if str(w.get('name') or '').lower() == 'charger':
                try:
                    return (
                        float(w['x']),
                        float(w['y']),
                        float(w.get('yaw', w.get('theta', 0.0)) or 0.0),
                    )
                except (KeyError, TypeError, ValueError):
                    return None
        return None

    def _wait_future(self, fut, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not fut.done():
            time.sleep(0.05)
        return fut.done()

    def _cli_is_active(self, cli, timeout_sec: float = 3.0) -> bool:
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            return False
        fut = cli.call_async(Trigger.Request())
        if not self._wait_future(fut, timeout_sec):
            return False
        res = fut.result()
        return bool(res is not None and res.success)

    def _nav2_lifecycle_is_active(self, timeout_sec: float = 3.0) -> bool:
        """Both localization (map/amcl) and navigation LMs must be active."""
        if not self._cli_is_active(self._nav_active_cli, timeout_sec=timeout_sec):
            return False
        # Require localization so /map + map→odom exist (orphans can fake nav LM).
        return self._cli_is_active(self._loc_active_cli, timeout_sec=timeout_sec)

    def _nav2_manage(self, command: int, timeout_sec: float = 60.0) -> bool:
        if not self._nav_lifecycle_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('lifecycle_manager_navigation/manage_nodes unavailable')
            return False
        req = ManageLifecycleNodes.Request()
        req.command = command
        fut = self._nav_lifecycle_cli.call_async(req)
        if not self._wait_future(fut, timeout_sec):
            self.get_logger().warn(f'manage_nodes cmd={command} timed out')
            return False
        res = fut.result()
        ok = bool(res is not None and res.success)
        self.get_logger().info(f'manage_nodes cmd={command} success={ok}')
        return ok

    def _ensure_nav2_active(self, deadline_sec: float = 90.0, map_name: str = '') -> bool:
        """Wait for Nav2 lifecycle; retry STARTUP if bringup aborted (Rock 5T DDS load)."""
        if not bool(self.get_parameter('use_nav2').value):
            return True

        deadline = time.monotonic() + deadline_sec
        attempts = 0
        seeded = False
        last_mgmt = time.monotonic()
        last_seed = 0.0
        name = (map_name or self._map_name or '').strip()
        while time.monotonic() < deadline:
            if self._proc is not None and self._proc.poll() is not None:
                self.get_logger().error('nav2 process died during bringup wait')
                return False
            loc_ok = self._cli_is_active(self._loc_active_cli, timeout_sec=1.5)
            if name and time.monotonic() - last_seed >= 2.0:
                # Keep re-seeding until active: volatile /initialpose published
                # before AMCL subscribed is dropped, and planner/costmap
                # activation blocks on map→base (AMCL only publishes after
                # consuming an initial pose).
                self._seed_initial_pose(name)
                last_seed = time.monotonic()
                seeded = True
            if loc_ok and self._cli_is_active(self._nav_active_cli, timeout_sec=1.5):
                if self._nav_client.wait_for_server(timeout_sec=5.0):
                    if not seeded and name:
                        self._seed_initial_pose(name)
                    self.get_logger().info('Nav2 lifecycle active + navigate_to_pose ready')
                    return True
                self.get_logger().warn('lifecycle active but navigate_to_pose not ready yet')
            attempts += 1
            # First autostart often fails configuring bt_navigator under CPU load.
            # Time-gated: let the initial autostart a full bringup window before
            # interrupting it with RESET/STARTUP (observed ~7 s retry cutting off a
            # healthy bringup that finished ~20 s in).
            if (attempts >= 2 and attempts % 2 == 0 and
                    time.monotonic() - last_mgmt >= 75.0):
                last_mgmt = time.monotonic()
                self.get_logger().warn(
                    f'Nav2 not active — retry STARTUP (attempt {attempts})'
                )
                # RESET is best-effort; STARTUP is what re-runs configure/activate.
                self._nav2_manage(ManageLifecycleNodes.Request.RESET, timeout_sec=30.0)
                # Only nudge localization if it is NOT already active — STARTUP on an
                # already-active map_server aborts bringup and flips is_active=false.
                if not self._cli_is_active(self._loc_active_cli, timeout_sec=1.5):
                    if self._loc_lifecycle_cli.wait_for_service(timeout_sec=2.0):
                        req = ManageLifecycleNodes.Request()
                        req.command = ManageLifecycleNodes.Request.STARTUP
                        fut = self._loc_lifecycle_cli.call_async(req)
                        self._wait_future(fut, 60.0)
                time.sleep(0.5)
                self._nav2_manage(ManageLifecycleNodes.Request.STARTUP, timeout_sec=75.0)
                # Re-seed after STARTUP — AMCL may have been reset.
                if name:
                    time.sleep(0.5)
                    self._seed_initial_pose(name)
                    seeded = True
            else:
                time.sleep(2.0)
        return self._nav2_lifecycle_is_active(timeout_sec=2.0) and self._nav_client.wait_for_server(
            timeout_sec=3.0
        )

    _NAV2_SWEEP_PATTERNS = (
        'nav2.launch.py',
        '/nav2_amcl/amcl',
        '/nav2_map_server/map_server',
        '/nav2_bt_navigator/bt_navigator',
        '/nav2_controller/controller_server',
        '/nav2_planner/planner_server',
        '/nav2_behaviors/behavior_server',
        '/nav2_smoother/smoother_server',
        '/nav2_velocity_smoother/velocity_smoother',
        '/nav2_collision_monitor/collision_monitor',
        '/nav2_waypoint_follower/waypoint_follower',
        '/nav2_lifecycle_manager/lifecycle_manager',
    )

    def _sweep_orphan_nav2(self) -> None:
        """Best-effort kill of Nav2 leftovers (external launch / broken process group)."""
        try:
            out = subprocess.check_output(['ps', '-eo', 'pid=,cmd='], text=True)
        except (OSError, subprocess.CalledProcessError):
            return
        my_pid = os.getpid()
        victims: List[int] = []
        for line in out.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                pid_s, cmd = line.split(None, 1)
                pid = int(pid_s)
            except ValueError:
                continue
            if pid == my_pid:
                continue
            if any(pat in cmd for pat in self._NAV2_SWEEP_PATTERNS):
                victims.append(pid)
        if not victims:
            return
        self.get_logger().warn(f'sweeping orphan Nav2 pids={victims}')
        for pid in victims:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        time.sleep(1.5)
        for pid in victims:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def _stop_nav2(self) -> None:
        proc = self._proc
        self._proc = None
        log_f = getattr(self, '_proc_log_f', None)
        self._proc_log_f = None
        if proc is not None:
            try:
                pgid = os.getpgid(proc.pid)
            except ProcessLookupError:
                pgid = None
            if pgid is not None:
                try:
                    os.killpg(pgid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        os.killpg(pgid, signal.SIGKILL)
                        proc.wait(timeout=3)
                except (ProcessLookupError, OSError, subprocess.TimeoutExpired):
                    pass
        # Always sweep — orphans from duplicate launches break lifecycle/is_active.
        self._sweep_orphan_nav2()
        if log_f is not None:
            try:
                log_f.close()
            except OSError:
                pass

    def _on_goal(self, msg: PoseStamped) -> None:
        if not self._active:
            self.get_logger().warn('goal ignored (nav inactive)')
            return
        if self._follow_en:
            self.get_logger().warn('goal rejected (follow active — stop follow first)')
            self._emit_result(1, 'rejected: follow active', 'goal')
            return
        if self._recharge_en:
            self.get_logger().warn('goal rejected (recharge active)')
            self._emit_result(1, 'rejected: recharge active', 'goal')
            return
        # Single goal preempts patrol / prior point goal; must clear _patrol_stop
        # or _navigate_blocking cancels the new goal immediately (~100ms).
        self._cancel_navigation('goal-preempt')
        self._patrol_stop.clear()
        self._command_id = self._command_id or 'goal'
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        self._emit_progress('goal_accepted', self._command_id, {'x': x, 'y': y})
        threading.Thread(
            target=self._run_single_goal,
            args=(msg, self._command_id),
            daemon=True,
        ).start()

    def _on_patrol_cmd(self, msg: String) -> None:
        if not self._active:
            self.get_logger().warn('patrol ignored (nav inactive)')
            return
        if self._follow_en:
            self.get_logger().warn('patrol rejected (follow active — stop follow first)')
            self._emit_result(1, 'rejected: follow active', 'patrol')
            return
        if self._recharge_en:
            self.get_logger().warn('patrol rejected (recharge active)')
            self._emit_result(1, 'rejected: recharge active', 'patrol')
            return
        try:
            payload = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.get_logger().error('patrol_cmd JSON invalid')
            return
        action = str(payload.get('action') or 'start').lower()
        if action in ('stop', 'cancel'):
            self._cancel_navigation('patrol-stop')
            self._emit_result(0, 'patrol stopped', self._command_id)
            return

        map_name = str(payload.get('map_name') or self._map_name).strip()
        loop = bool(payload.get('loop', False))
        names = payload.get('waypoints')  # optional ordered names
        self._command_id = str(payload.get('command_id') or 'patrol')
        poses = self._load_waypoint_poses(map_name, names)
        if not poses:
            self._emit_result(1, f'no waypoints for {map_name}', self._command_id)
            return
        self._stop_patrol_loop()
        self._patrol_stop.clear()
        self._patrol_active = True
        self._patrol_thread = threading.Thread(
            target=self._run_patrol,
            args=(poses, loop, self._command_id),
            daemon=True,
        )
        self._patrol_thread.start()

    def _load_waypoint_poses(
        self, map_name: str, names: Optional[List[str]]
    ) -> List[PoseStamped]:
        if not self._wp_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('waypoint service unavailable')
            return []
        req = WaypointManage.Request()
        req.operation = 2
        req.map_name = map_name
        fut = self._wp_cli.call_async(req)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not fut.done():
            time.sleep(0.05)
        if not fut.done() or fut.result() is None:
            return []
        res = fut.result()
        try:
            data = json.loads(res.data_json or '{}')
        except json.JSONDecodeError:
            return []
        wps: List[Dict[str, Any]] = list(data.get('waypoints') or [])
        if data.get('charger') and isinstance(data['charger'], dict):
            c = data['charger']
            wps.insert(0, {
                'name': 'charger',
                'x': c.get('x'),
                'y': c.get('y'),
                'yaw': c.get('yaw', c.get('theta', 0.0)),
            })
        if names:
            by_name = {str(w.get('name') or w.get('id') or ''): w for w in wps}
            ordered = [by_name[n] for n in names if n in by_name]
        else:
            ordered = wps

        poses: List[PoseStamped] = []
        for w in ordered:
            try:
                x = float(w['x'])
                y = float(w['y'])
            except (KeyError, TypeError, ValueError):
                continue
            yaw = float(w.get('yaw', w.get('theta', 0.0)) or 0.0)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation = _yaw_to_quat(yaw)
            poses.append(ps)
        return poses

    def _run_patrol(self, poses: List[PoseStamped], loop: bool, command_id: str) -> None:
        self._emit_progress('patrol_start', command_id, {'count': len(poses), 'loop': loop})
        idx = 0
        while not self._patrol_stop.is_set() and self._active:
            pose = poses[idx % len(poses)]
            pose.header.stamp = self.get_clock().now().to_msg()
            self._emit_progress(
                'patrol_waypoint',
                command_id,
                {'index': idx % len(poses), 'x': pose.pose.position.x, 'y': pose.pose.position.y},
            )
            ok = self._navigate_blocking(pose, command_id)
            if self._patrol_stop.is_set():
                break
            if not ok:
                self._emit_result(1, f'patrol failed at index {idx % len(poses)}', command_id)
                self._patrol_active = False
                return
            idx += 1
            if not loop and idx >= len(poses):
                self._emit_result(0, 'patrol complete', command_id)
                self._patrol_active = False
                return
        self._patrol_active = False

    def _run_single_goal(self, pose: PoseStamped, command_id: str) -> None:
        pose.header.stamp = self.get_clock().now().to_msg()
        if not pose.header.frame_id:
            pose.header.frame_id = 'map'
        ok = self._navigate_blocking(pose, command_id)
        if ok:
            self._emit_result(0, 'goal succeeded', command_id, {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
            })
        else:
            self._emit_result(1, 'goal failed/aborted', command_id)

    def _navigate_blocking(self, pose: PoseStamped, command_id: str) -> bool:
        if not bool(self.get_parameter('use_nav2').value):
            self.get_logger().warn('use_nav2=false — simulating success')
            time.sleep(0.2)
            return True

        # Recover if lifecycle bringup aborted earlier (nodes stuck inactive).
        if not self._nav2_lifecycle_is_active(timeout_sec=1.5):
            self.get_logger().warn('Nav2 inactive at goal time — attempting recover')
            if not self._ensure_nav2_active(deadline_sec=90.0, map_name=self._map_name):
                self.get_logger().error('navigate_to_pose unavailable (Nav2 inactive)')
                return False

        if not self._nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('navigate_to_pose action server not ready')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_fut = self._nav_client.send_goal_async(goal)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not send_fut.done():
            time.sleep(0.05)
        if not send_fut.done():
            self.get_logger().error('send_goal timed out')
            return False
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn('goal rejected')
            return False

        with self._lock:
            self._nav_goal_handle = gh

        self._emit_progress('executing', command_id)
        result_fut = gh.get_result_async()
        while not result_fut.done():
            if self._patrol_stop.is_set() or not self._active:
                try:
                    gh.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                return False
            time.sleep(0.1)

        with self._lock:
            self._nav_goal_handle = None

        result = result_fut.result()
        # status 4 = SUCCEEDED in action_msgs
        status = int(getattr(result, 'status', 0))
        return status == 4

    def _stop_patrol_loop(self) -> None:
        self._patrol_stop.set()
        self._patrol_active = False

    def _cancel_navigation(self, source: str) -> None:
        self._stop_patrol_loop()
        with self._lock:
            gh = self._nav_goal_handle
            self._nav_goal_handle = None
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'cancel failed: {exc}')
        self.get_logger().info(f'nav cancel ({source})')

    def _emit_progress(self, phase: str, command_id: str = '', data: Optional[dict] = None) -> None:
        p = TaskProgress()
        p.stamp = self.get_clock().now().to_msg()
        p.command_id = command_id or self._command_id
        p.capability = 'nav'
        p.phase = phase
        if data:
            p.detail = json.dumps(data, ensure_ascii=False)
        self._progress_pub.publish(p)

    def _emit_result(
        self,
        code: int,
        message: str,
        command_id: str = '',
        data: Optional[dict] = None,
    ) -> None:
        r = TaskResult()
        r.stamp = self.get_clock().now().to_msg()
        r.command_id = command_id or self._command_id
        r.capability = 'nav'
        r.code = int(code)
        r.message = message
        if data:
            r.data_json = json.dumps(data, ensure_ascii=False)
        self._result_pub.publish(r)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavSessionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._cancel_navigation('shutdown')  # noqa: SLF001
        node._stop_nav2()  # noqa: SLF001
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
