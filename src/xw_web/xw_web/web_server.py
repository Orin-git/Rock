#!/usr/bin/env python3
"""SPA + ROS JSON bridge (HTTP) for Xiaowei Gen2 console."""

from __future__ import annotations

import functools
import json
import os
import socket
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import parse_qs, urlparse

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from rosidl_runtime_py.utilities import get_message

from xw_interfaces.msg import RobotState, TaskProgress, TaskResult
from xw_interfaces.srv import MapManage, MotionCommand, SetMode, SetRunMode, WaypointManage

# URDF xw_gen2.urdf — relative to base_link (fill sensors later without UI rewrite)
_SENSOR_LAYOUT = [
    {'id': 'lidar', 'frame': 'lidar_link', 'xyz': [0.0, 0.0, 0.22], 'status': 'live', 'label': '激光雷达'},
    {
        'id': 'camera_front',
        'frame': 'camera_front_link',
        'xyz': [0.18, 0.0, 0.25],
        'status': 'live',
        'label': '前视深度相机',
    },
    {
        'id': 'camera_front_2',
        'frame': 'camera_front_2_link',
        'xyz': [0.18, 0.05, 0.25],
        'status': 'live',
        'label': '前视深度相机二号',
    },
    {
        'id': 'ultrasonic',
        'frame': 'ultrasonic_front_link',
        'xyz': [0.22, 0.0, 0.08],
        'status': 'placeholder',
        'label': '超声波（占位）',
    },
    {
        'id': 'imu',
        'frame': 'imu_link',
        'xyz': [0.0, 0.0, 0.12],
        'status': 'placeholder',
        'label': 'IMU（占位）',
    },
    {
        'id': 'chassis',
        'frame': 'base_link',
        'xyz': [0.0, 0.0, 0.0],
        'status': 'partial',
        'label': '底盘 / odom',
    },
]

_LATCHED_BOOL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Heavy / binary message types: only expose metadata to avoid CPU spikes.
_HEAVY_TYPES = frozenset(
    {
        'sensor_msgs/msg/Image',
        'sensor_msgs/msg/CompressedImage',
        'sensor_msgs/msg/PointCloud2',
        'sensor_msgs/msg/PointCloud',
        'nav_msgs/msg/OccupancyGrid',
        'sensor_msgs/msg/LaserScan',
        'sensor_msgs/msg/MultiEchoLaserScan',
        'visualization_msgs/msg/MarkerArray',
        'tf2_msgs/msg/TFMessage',
    }
)

_TOPIC_HINTS: Dict[str, str] = {
    '/xw/robot_state': '机器人总状态：模式、run_mode、急停、安全闸、定位、电池摘要',
    '/xw/supervisor/set_mode': '切换工作模式 IDLE/建图/导航/跟随/跌倒',
    '/xw/supervisor/set_run_mode': '切换运行形态：0量产 / 1开发者（默认开发者）',
    '/xw/supervisor/get_state': '查询 Supervisor 当前状态',
    '/xw/power': '电源状态：电量、电压、充电/回充对接',
    '/xw/event': '系统事件流（Supervisor 广播）',
    '/xw/task/progress': '任务进度：能力会话当前阶段',
    '/xw/task/result': '任务结果：成功/失败码与说明',
    '/xw/cmd/teleop': '网页遥控速度指令（→ 仲裁）',
    '/xw/cmd/motion': '点动（角度/距离）速度指令',
    '/xw/cmd/follow': '跟随会话速度指令',
    '/xw/cmd/gated': '仲裁后送入安全门的速度',
    '/cmd_vel': '安全门输出 → 底盘最终速度',
    '/scan': '激光雷达扫描（原始）',
    'scan': '激光雷达扫描（原始）',
    '/odom': '里程计位姿与速度',
    'odom': '里程计位姿与速度',
    '/xw/chassis/motor_disabled': '底盘失能：true=不可控，false=使能可遥控',
    '/safety_status': '安全闸通过状态',
    'safety_status': '安全闸通过状态',
    '/obstacle_status': '障碍描述文本',
    'obstacle_status': '障碍描述文本',
    '/ultrasonic_array': '超声波测距阵列',
    '/xw/slam/enable': '建图会话使能',
    '/xw/nav/enable': '导航会话使能',
    '/xw/follow/enable': '跟随会话使能',
    '/xw/fall/enable': '跌倒巡视会话使能',
    '/xw/goal_pose': '导航目标位姿',
    '/xw/perception/tracks': '人体跟踪检测结果',
    '/xw/perception/fall': '跌倒感知状态',
    '/xw/fall/status': '跌倒会话状态',
    '/xw/motion/status': '点动执行状态',
    '/camera/front/color/image_raw': '前视彩色图（raw，算法用，不进 Foxglove）',
    '/camera/front/color/image_raw/compressed': '前视彩色预览（JPEG，Foxglove Desktop 看）',
    '/camera/front/color/camera_info': '前视彩色内参',
    '/camera/front/depth/image_raw': '前视深度图（本机算法/安全门）',
    '/camera/front/depth/camera_info': '前视深度内参',
    '/camera/front/depth/points': '前视点云（调试开关 enable_pointcloud，默认关）',
    '/map': '占用栅格地图',
    '/tf': '动态坐标变换',
    '/tf_static': '静态坐标变换',
    '/robot_description': '机器人 URDF 描述',
    '/joint_states': '关节状态',
    '/parameter_events': '参数变更事件（系统）',
    '/rosout': 'ROS 日志流',
}

_NODE_HINTS: Dict[str, str] = {
    'xw_supervisor': '总控：模式切换、状态汇总、会话使能',
    'xw_web_bridge': '网页 HTTP 桥（本节点）',
    'xw_map_manager': '地图/航点存储与管理',
    'xw_cmd_arbiter': '多路速度指令仲裁',
    'xw_safety_gate': '激光/超声安全门，输出 cmd_vel',
    'xw_chassis': '底盘驱动、里程计、电源、急停',
    'xw_motion': '角度/距离点动',
    'xw_slam_session': '建图会话',
    'xw_nav_session': '导航会话',
    'xw_follow_session': '跟随会话',
    'xw_fall_session': '跌倒巡视会话',
    'xw_perception_stub': '感知（人体/跌倒 stub）',
    'xw_perception': '感知（YOLOv8-pose RKNN → tracks/fall）',
    'xw_sensors': '传感器桩/桥接',
    'xw_depth_topic_bridge': '深度相机话题适配（/camera/front）',
    'camera_publisher': 'Angstrong 深度相机驱动',
    'xw_health': '话题健康监测',
    'robot_state_publisher': '根据 URDF 发布 TF',
}


def _state_to_dict(msg: RobotState) -> Dict[str, Any]:
    return {
        'mode': int(msg.mode),
        'mode_name': msg.mode_name,
        'run_mode': int(msg.run_mode),
        'emergency_stop': bool(msg.emergency_stop),
        'safety_ok': bool(msg.safety_ok),
        'localization_ok': bool(msg.localization_ok),
        'active_map': msg.active_map,
        'profile': msg.profile,
        'detail': msg.detail,
        'power': {
            'battery_percent': float(msg.power.battery_percent),
            'voltage': float(msg.power.voltage),
            'charging': bool(msg.power.charging),
            'docked': bool(msg.power.docked),
            'detail': msg.power.detail,
        },
    }


def _topic_hint(name: str) -> str:
    if name in _TOPIC_HINTS:
        return _TOPIC_HINTS[name]
    base = name.split('/')[-1]
    if base and f'/{base}' in _TOPIC_HINTS:
        return _TOPIC_HINTS[f'/{base}']
    if name.startswith('/xw/cmd/'):
        return '速度指令通道（进入仲裁器）'
    if name.startswith('/xw/session/'):
        return '能力会话控制相关话题'
    if name.startswith('/xw/'):
        return '小维业务话题'
    return '系统/通用话题'


def _node_hint(name: str) -> str:
    short = name.split('/')[-1]
    if short in _NODE_HINTS:
        return _NODE_HINTS[short]
    if short.startswith('xw_'):
        return '小维栈节点'
    if 'lifecycle' in short:
        return '生命周期管理节点'
    return 'ROS 节点'


def _jsonable(obj: Any, *, depth: int = 0, max_depth: int = 6, max_list: int = 24) -> Any:
    """Convert ROS msg / nested structures to JSON-safe data with hard caps."""
    if depth > max_depth:
        return '…'
    if obj is None or isinstance(obj, (bool, int, float)):
        return obj
    if isinstance(obj, str):
        return obj if len(obj) <= 400 else obj[:400] + f'…(+{len(obj) - 400})'
    if isinstance(obj, bytes):
        return f'<bytes len={len(obj)}>'
    if isinstance(obj, (list, tuple)):
        n = len(obj)
        head = [_jsonable(x, depth=depth + 1, max_depth=max_depth, max_list=max_list) for x in obj[:max_list]]
        if n > max_list:
            head.append(f'…(+{n - max_list} items)')
        return head
    # ROS message or simple object
    slots = getattr(obj, 'get_fields_and_field_types', None)
    if callable(slots):
        out: Dict[str, Any] = {}
        for field in slots().keys():
            try:
                out[field] = _jsonable(getattr(obj, field), depth=depth + 1, max_depth=max_depth, max_list=max_list)
            except Exception as exc:  # noqa: BLE001
                out[field] = f'<err {exc}>'
        return out
    if isinstance(obj, dict):
        return {
            str(k): _jsonable(v, depth=depth + 1, max_depth=max_depth, max_list=max_list)
            for k, v in list(obj.items())[:80]
        }
    return str(obj)[:200]


def _watch_qos(msg_type_name: str) -> QoSProfile:
    # Prefer low-cost depth-1; sensor types use sensor data profile.
    if msg_type_name.startswith('sensor_msgs/') or msg_type_name in (
        'nav_msgs/msg/Odometry',
        'tf2_msgs/msg/TFMessage',
    ):
        q = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        return q
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_web_bridge')
        self._lock = threading.Lock()
        self._state: Dict[str, Any] = {
            'mode': 0,
            'mode_name': 'IDLE',
            'run_mode': 1,
            'emergency_stop': False,
            'safety_ok': True,
            'localization_ok': True,
            'active_map': '',
            'profile': 'normal',
            'detail': 'waiting',
            'power': {
                'battery_percent': 0.0,
                'voltage': 0.0,
                'charging': False,
                'docked': False,
                'detail': '',
            },
        }
        self._tasks: List[str] = []
        self._obstacle: Dict[str, Any] = {
            'blocked': False,
            'any_sector_blocked': False,
            'safety_ok': True,
            'reason': 'waiting',
            'depth_m': None,
            'sectors': {
                'front': {'blocked': False, 'range_m': None, 'source': None},
                'rear': {'blocked': False, 'range_m': None, 'source': None},
                'left': {'blocked': False, 'range_m': None, 'source': None},
                'right': {'blocked': False, 'range_m': None, 'source': None},
            },
        }

        # On-demand single-topic probe (only one active subscription at a time).
        self._watch_topic: Optional[str] = None
        self._watch_type: Optional[str] = None
        self._watch_sub = None
        self._watch_data: Any = None
        self._watch_recv_count = 0
        self._watch_last_recv = 0.0
        self._watch_started = 0.0
        self._watch_error: Optional[str] = None
        self._watch_heavy = False
        self._watch_lease_until = 0.0
        self._watch_min_interval = 0.25  # seconds between accepted samples
        self._last_graph_cache: Dict[str, Any] = {'topics': [], 'nodes': [], 'ts': 0.0}
        self._graph_min_interval = 2.0

        self.create_subscription(RobotState, '/xw/robot_state', self._on_state, 10)
        self.create_subscription(TaskProgress, '/xw/task/progress', self._on_progress, 10)
        self.create_subscription(TaskResult, '/xw/task/result', self._on_result, 10)
        self.create_subscription(String, 'obstacle_status', self._on_obstacle, 10)
        self._teleop_pub = self.create_publisher(Twist, '/xw/cmd/teleop', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)
        self._set_mode = self.create_client(SetMode, '/xw/supervisor/set_mode')
        self._set_run_mode = self.create_client(SetRunMode, '/xw/supervisor/set_run_mode')
        self._map_mgr = self.create_client(MapManage, '/xw/map/manage')
        self._wp_mgr = self.create_client(WaypointManage, '/xw/map/waypoint')
        self._set_pointcloud = self.create_client(SetBool, '/xw/camera/set_pointcloud')
        self._set_fall = self.create_client(SetBool, '/xw/supervisor/set_fall')
        self._motion_cli = self.create_client(MotionCommand, '/xw/motion/command')
        self._pointcloud_enabled = False
        self._fall_enabled = False
        self.create_subscription(
            Bool,
            '/xw/camera/pointcloud_enabled',
            self._on_pointcloud_enabled,
            _LATCHED_BOOL_QOS,
        )
        self.create_subscription(
            Bool,
            '/xw/fall/enable',
            self._on_fall_enabled,
            _LATCHED_BOOL_QOS,
        )
        self.create_timer(5.0, self._watch_housekeep)
        domain = os.environ.get('ROS_DOMAIN_ID', '?')
        self.get_logger().info(f'web ROS bridge ready (DOMAIN={domain})')

    def _on_pointcloud_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._pointcloud_enabled = bool(msg.data)

    def _on_fall_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._fall_enabled = bool(msg.data)

    def pointcloud_status(self) -> Dict[str, Any]:
        with self._lock:
            enabled = bool(self._pointcloud_enabled)
        ready = self._set_pointcloud.service_is_ready()
        return {
            'ok': True,
            'enabled': enabled,
            'service_ready': ready,
            'topic': '/camera/front/depth/points',
            'hint': '导航自动开；手动开关会持久化。Foxglove 3D → Point Cloud',
        }

    def set_pointcloud(self, enabled: bool) -> Dict[str, Any]:
        if not self._set_pointcloud.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'set_pointcloud service unavailable (depth bridge down?)'}
        req = SetBool.Request()
        req.data = bool(enabled)
        fut = self._set_pointcloud.call_async(req)
        for _ in range(60):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'set_pointcloud timeout'}
        res = fut.result()
        with self._lock:
            self._pointcloud_enabled = bool(enabled) if res.success else self._pointcloud_enabled
        self._push_task(f'[pointcloud] {res.message}')
        return {
            'ok': bool(res.success),
            'enabled': bool(enabled) if res.success else self._pointcloud_enabled,
            'message': res.message,
            'topic': '/camera/front/depth/points',
        }

    def fall_status(self) -> Dict[str, Any]:
        with self._lock:
            enabled = bool(self._fall_enabled)
        ready = self._set_fall.service_is_ready()
        return {
            'ok': True,
            'enabled': enabled,
            'service_ready': ready,
            'topic': '/xw/fall/enable',
            'hint': '正交开关：可与 IDLE/导航同时开；跟随复用同一感知管线',
        }

    def set_fall(self, enabled: bool) -> Dict[str, Any]:
        if not self._set_fall.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'set_fall service unavailable (supervisor down?)'}
        req = SetBool.Request()
        req.data = bool(enabled)
        fut = self._set_fall.call_async(req)
        for _ in range(60):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'set_fall timeout'}
        res = fut.result()
        with self._lock:
            self._fall_enabled = bool(enabled) if res.success else self._fall_enabled
        self._push_task(f'[fall] {res.message}')
        return {
            'ok': bool(res.success),
            'enabled': bool(enabled) if res.success else self._fall_enabled,
            'message': res.message,
            'topic': '/xw/fall/enable',
        }

    def _push_task(self, line: str) -> None:
        with self._lock:
            self._tasks.insert(0, line)
            del self._tasks[80:]

    def _on_state(self, msg: RobotState) -> None:
        with self._lock:
            self._state = _state_to_dict(msg)

    def _on_obstacle(self, msg: String) -> None:
        raw = (msg.data or '').strip()
        if not raw:
            return
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            parsed = {'reason': raw, 'blocked': 'block' in raw.lower()}
        if not isinstance(parsed, dict):
            return
        sectors_in = parsed.get('sectors') if isinstance(parsed.get('sectors'), dict) else {}
        def _sec(key: str) -> Dict[str, Any]:
            s = sectors_in.get(key) if isinstance(sectors_in.get(key), dict) else {}
            return {
                'blocked': bool(s.get('blocked', False)),
                'range_m': s.get('range_m'),
                'source': s.get('source'),
                'stop_m': s.get('stop_m'),
            }
        with self._lock:
            self._obstacle = {
                'blocked': bool(parsed.get('blocked', False)),
                'any_sector_blocked': bool(parsed.get('any_sector_blocked', parsed.get('blocked', False))),
                'safety_ok': bool(parsed.get('safety_ok', not parsed.get('blocked', False))),
                'reason': str(parsed.get('reason') or ''),
                'depth_m': parsed.get('depth_m'),
                'sectors': {
                    'front': _sec('front'),
                    'rear': _sec('rear'),
                    'left': _sec('left'),
                    'right': _sec('right'),
                },
            }

    def _on_progress(self, msg: TaskProgress) -> None:
        self._push_task(f'[progress] {msg.capability} {msg.phase}')

    def _on_result(self, msg: TaskResult) -> None:
        self._push_task(f'[result] {msg.capability} code={msg.code} {msg.message}')

    def service_status(self) -> Dict[str, Any]:
        """Quick view of whether core ROS services are up (for web domain hub)."""
        supervisor = self._set_mode.service_is_ready()
        map_mgr = self._map_mgr.service_is_ready()
        with self._lock:
            st = dict(self._state)
        stack_up = bool(supervisor and map_mgr)
        return {
            'supervisor_up': bool(supervisor),
            'map_manager_up': bool(map_mgr),
            'stack_up': stack_up,
            'mode_name': st.get('mode_name', ''),
            'detail': st.get('detail', ''),
        }

    def foxglove_status(self) -> Dict[str, Any]:
        """TCP probe of foxglove_bridge WebSocket port (default 8765)."""
        port = int(os.environ.get('FOXGLOVE_PORT', '8765') or 8765)
        host = os.environ.get('FOXGLOVE_PROBE_HOST', '127.0.0.1')
        up = False
        err = ''
        try:
            with socket.create_connection((host, port), timeout=0.35):
                up = True
        except OSError as exc:
            err = str(exc)
        pkg = False
        try:
            for base in (
                Path('/opt/ros/humble/share/foxglove_bridge'),
                Path(os.environ.get('ROS_DISTRO', 'humble') and f'/opt/ros/{os.environ.get("ROS_DISTRO", "humble")}/share/foxglove_bridge'),
            ):
                if base.is_dir():
                    pkg = True
                    break
        except Exception:  # noqa: BLE001
            pkg = False
        return {
            'port': port,
            'up': up,
            'package_hint': pkg,
            'ws_url_hint': f'ws://<board-ip>:{port}',
            'error': err if not up else '',
        }

    def snapshot(self) -> Dict[str, Any]:
        svc = self.service_status()
        fox = self.foxglove_status()
        with self._lock:
            return {
                'ok': True,
                'state': dict(self._state),
                'obstacle': dict(self._obstacle),
                'tasks': list(self._tasks[:40]),
                'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
                'robot_id': os.environ.get('ROBOT_ID', ''),
                'services': svc,
                'foxglove': fox,
                'watching': self._watch_topic,
            }

    def publish_teleop(self, linear_x: float, angular_z: float) -> Dict[str, Any]:
        t = Twist()
        t.linear.x = float(linear_x)
        t.angular.z = float(angular_z)
        self._teleop_pub.publish(t)
        return {'ok': True}

    def call_motion(
        self,
        angle_deg: float,
        distance_m: float,
        command_id: str = '',
    ) -> Dict[str, Any]:
        if not self._motion_cli.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'motion service unavailable (xw_motion down?)'}
        req = MotionCommand.Request()
        req.angle_deg = float(angle_deg)
        req.distance_m = float(distance_m)
        req.command_id = command_id or f'ui-{int(time.time() * 1000)}'
        fut = self._motion_cli.call_async(req)
        for _ in range(80):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'motion command timeout'}
        res = fut.result()
        self._push_task(
            f'[motion] id={req.command_id} ang={angle_deg} dist={distance_m} → {res.message}'
        )
        return {
            'ok': bool(res.success),
            'message': res.message,
            'command_id': req.command_id,
            'angle_deg': float(angle_deg),
            'distance_m': float(distance_m),
        }

    def publish_goal(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str = 'map',
    ) -> Dict[str, Any]:
        """Publish navigation goal → /xw/goal_pose (consumed by xw_nav_session)."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id or 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        half = float(yaw) * 0.5
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        self._goal_pub.publish(msg)
        self._push_task(
            f'[goal] frame={msg.header.frame_id} x={x:.3f} y={y:.3f} yaw={yaw:.3f}'
        )
        return {
            'ok': True,
            'topic': '/xw/goal_pose',
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
            'frame_id': msg.header.frame_id,
        }

    def sensor_hub_status(self) -> Dict[str, Any]:
        """Topic presence for nav sensor panel (live + placeholders)."""
        try:
            pairs = self.get_topic_names_and_types()
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'message': str(exc), 'sensors': {}, 'layout': _SENSOR_LAYOUT}
        names = {str(n) for n, _ in pairs}

        def has(*topics: str) -> bool:
            return any(t in names for t in topics)

        sensors = {
            'lidar': {
                'id': 'lidar',
                'label': '激光雷达',
                'status': 'live' if has('/scan', 'scan') else 'missing',
                'topics': ['/scan'],
                'present': has('/scan', 'scan'),
            },
            'depth_camera': {
                'id': 'depth_camera',
                'label': '前视深度相机',
                'status': 'live'
                if has(
                    '/camera/front/color/image_raw/compressed',
                    '/camera/front/depth/image_raw',
                )
                else 'missing',
                'topics': [
                    '/camera/front/color/image_raw/compressed',
                    '/camera/front/depth/image_raw',
                ],
                'present': has(
                    '/camera/front/color/image_raw/compressed',
                    '/camera/front/depth/image_raw',
                ),
                'pointcloud_enabled': bool(self._pointcloud_enabled),
                'preview': '/camera/front/color/image_raw/compressed',
            },
            'depth_camera_2': {
                'id': 'depth_camera_2',
                'label': '前视深度相机二号',
                'status': 'live'
                if has(
                    '/camera/front_2/color/image_raw/compressed',
                    '/camera/front_2/depth/image_raw',
                )
                else 'missing',
                'topics': [
                    '/camera/front_2/color/image_raw/compressed',
                    '/camera/front_2/depth/image_raw',
                ],
                'present': has(
                    '/camera/front_2/color/image_raw/compressed',
                    '/camera/front_2/depth/image_raw',
                ),
                'preview': '/camera/front_2/color/image_raw/compressed',
            },
            'ultrasonic': {
                'id': 'ultrasonic',
                'label': '超声波',
                'status': 'placeholder',
                'topics': ['/ultrasonic_array'],
                'present': has('/ultrasonic_array'),
                'hint': '后续接入测距阵列',
            },
            'imu': {
                'id': 'imu',
                'label': 'IMU',
                'status': 'placeholder',
                'topics': ['/imu/data', '/imu'],
                'present': has('/imu/data', '/imu'),
                'hint': '后续接入姿态融合',
            },
            'chassis': {
                'id': 'chassis',
                'label': '底盘',
                'status': 'live' if has('/odom', 'odom', '/cmd_vel') else 'partial',
                'topics': ['/odom', '/cmd_vel', '/xw/cmd/teleop'],
                'present': has('/odom', 'odom') or has('/cmd_vel'),
                'hint': '速度经仲裁 → 安全门 → /cmd_vel',
            },
        }
        return {
            'ok': True,
            'sensors': sensors,
            'layout': _SENSOR_LAYOUT,
            'contracts': {
                'goal': '/xw/goal_pose',
                'nav_enable': '/xw/nav/enable',
                'map': '/map',
                'scan': '/scan',
            },
        }

    def set_mode(self, mode: int, payload: Optional[Dict[str, Any]], command_id: str) -> Dict[str, Any]:
        if not self._set_mode.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'set_mode service unavailable'}
        req = SetMode.Request()
        req.mode = int(mode)
        req.payload_json = json.dumps(payload or {})
        req.command_id = command_id or f'web-{self.get_clock().now().nanoseconds}'
        fut = self._set_mode.call_async(req)
        for _ in range(60):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'set_mode timeout'}
        res = fut.result()
        self._push_task(f'[set_mode] {res.message} active={res.active_mode}')
        return {
            'ok': bool(res.success),
            'message': res.message,
            'active_mode': int(res.active_mode),
        }

    def set_run_mode(self, run_mode: int) -> Dict[str, Any]:
        """0 production / 1 developer — Gen2 default is developer."""
        if not self._set_run_mode.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'set_run_mode service unavailable', 'run_mode': 1}
        req = SetRunMode.Request()
        req.run_mode = int(run_mode)
        fut = self._set_run_mode.call_async(req)
        for _ in range(60):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'set_run_mode timeout', 'run_mode': 1}
        res = fut.result()
        label = '量产' if int(res.run_mode) == 0 else '开发者'
        self._push_task(f'[run_mode] {label} ({res.message})')
        return {
            'ok': bool(res.success),
            'message': res.message,
            'run_mode': int(res.run_mode),
            'label': label,
        }

    def map_manage(
        self, operation: int, map_name: str = '', new_name: str = '', data_json: str = ''
    ) -> Dict[str, Any]:
        if not self._map_mgr.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'map service unavailable'}
        req = MapManage.Request()
        req.operation = int(operation)
        req.map_name = map_name
        req.new_name = new_name
        req.data_json = data_json
        fut = self._map_mgr.call_async(req)
        # SAVE / GET_MAP / UPDATE_MAP may transfer large PGM payloads
        wait_rounds = 800 if int(operation) in (1, 5, 6) else 500
        for _ in range(wait_rounds):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'map timeout'}
        res = fut.result()
        self._push_task(f'[map] op={operation} {res.message}')
        return {
            'ok': bool(res.success),
            'message': res.message,
            'map_list': list(res.map_list),
            'data_json': res.data_json,
        }

    def waypoint_manage(
        self,
        operation: int,
        map_name: str = '',
        waypoint_name: str = '',
        new_name: str = '',
        data_json: str = '',
    ) -> Dict[str, Any]:
        if not self._wp_mgr.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'waypoint service unavailable', 'names': []}
        req = WaypointManage.Request()
        req.operation = int(operation)
        req.map_name = map_name
        req.waypoint_name = waypoint_name
        req.new_name = new_name
        req.data_json = data_json
        fut = self._wp_mgr.call_async(req)
        for _ in range(160):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'waypoint timeout', 'names': []}
        res = fut.result()
        self._push_task(f'[waypoint] op={operation} {res.message}')
        return {
            'ok': bool(res.success),
            'message': res.message,
            'names': list(res.names),
            'data_json': res.data_json,
        }

    # ── Graph listing (metadata only — no message subscriptions) ──────────

    def list_graph(self, force: bool = False) -> Dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            cache = self._last_graph_cache
            if not force and (now - float(cache.get('ts') or 0.0)) < self._graph_min_interval:
                return {
                    'ok': True,
                    'topics': list(cache.get('topics') or []),
                    'nodes': list(cache.get('nodes') or []),
                    'cached': True,
                    'watching': self._watch_topic,
                }

        try:
            topic_names = self.get_topic_names_and_types()
            node_names = self.get_node_names_and_namespaces()
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'message': str(exc), 'topics': [], 'nodes': []}

        topics: List[Dict[str, Any]] = []
        for name, types in sorted(topic_names, key=lambda x: x[0]):
            if name.startswith('/_'):
                continue
            t0 = types[0] if types else ''
            topics.append(
                {
                    'name': name,
                    'types': list(types),
                    'hint': _topic_hint(name),
                    'heavy': t0 in _HEAVY_TYPES,
                }
            )

        nodes: List[Dict[str, Any]] = []
        seen = set()
        for n, ns in sorted(node_names, key=lambda x: (x[1], x[0])):
            full = f'{ns.rstrip("/")}/{n}' if ns and ns != '/' else f'/{n}'
            if full in seen:
                continue
            seen.add(full)
            nodes.append({'name': full, 'short': n, 'namespace': ns or '/', 'hint': _node_hint(n)})

        with self._lock:
            self._last_graph_cache = {'topics': topics, 'nodes': nodes, 'ts': now}
            watching = self._watch_topic

        return {
            'ok': True,
            'topics': topics,
            'nodes': nodes,
            'cached': False,
            'watching': watching,
        }

    # ── On-demand topic probe ─────────────────────────────────────────────

    def _watch_housekeep(self) -> None:
        with self._lock:
            topic = self._watch_topic
            lease = self._watch_lease_until
        if topic and time.monotonic() > lease:
            self.get_logger().info(f'watch lease expired for {topic}')
            self.unwatch_topic()

    def _destroy_watch_sub(self) -> None:
        sub = self._watch_sub
        self._watch_sub = None
        if sub is not None:
            try:
                self.destroy_subscription(sub)
            except Exception:  # noqa: BLE001
                pass

    def unwatch_topic(self) -> Dict[str, Any]:
        with self._lock:
            self._destroy_watch_sub()
            self._watch_topic = None
            self._watch_type = None
            self._watch_data = None
            self._watch_recv_count = 0
            self._watch_last_recv = 0.0
            self._watch_started = 0.0
            self._watch_error = None
            self._watch_heavy = False
            self._watch_lease_until = 0.0
        return {'ok': True, 'watching': None}

    def watch_topic(self, topic: str, type_name: str = '', lease_sec: float = 45.0) -> Dict[str, Any]:
        topic = (topic or '').strip()
        if not topic:
            return {'ok': False, 'message': 'topic required'}
        if not topic.startswith('/'):
            topic = '/' + topic

        # Resolve type from graph if not provided.
        if not type_name:
            for name, types in self.get_topic_names_and_types():
                if name == topic and types:
                    type_name = types[0]
                    break
        if not type_name:
            return {'ok': False, 'message': f'unknown topic or type: {topic}'}

        heavy = type_name in _HEAVY_TYPES
        try:
            msg_cls = get_message(type_name)
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'message': f'cannot load type {type_name}: {exc}'}

        with self._lock:
            if self._watch_topic == topic and self._watch_type == type_name and self._watch_sub is not None:
                self._watch_lease_until = time.monotonic() + max(10.0, float(lease_sec))
                return {
                    'ok': True,
                    'topic': topic,
                    'type': type_name,
                    'heavy': heavy,
                    'hint': _topic_hint(topic),
                    'reused': True,
                }
            self._destroy_watch_sub()
            self._watch_topic = topic
            self._watch_type = type_name
            self._watch_data = None
            self._watch_recv_count = 0
            self._watch_last_recv = 0.0
            self._watch_started = time.monotonic()
            self._watch_error = None
            self._watch_heavy = heavy
            self._watch_lease_until = time.monotonic() + max(10.0, float(lease_sec))

            def _cb(msg: Any, _topic: str = topic) -> None:
                now = time.monotonic()
                with self._lock:
                    if self._watch_topic != _topic:
                        return
                    if now - self._watch_last_recv < self._watch_min_interval:
                        return  # drop excess samples — save CPU
                    self._watch_last_recv = now
                    self._watch_recv_count += 1
                    if self._watch_heavy:
                        self._watch_data = self._summarize_heavy(msg, type_name)
                    else:
                        self._watch_data = _jsonable(msg)

            try:
                qos = _watch_qos(type_name)
                # Fall back: for latched topics sensor profile may miss — try sensor then default.
                try:
                    self._watch_sub = self.create_subscription(msg_cls, topic, _cb, qos)
                except Exception:
                    self._watch_sub = self.create_subscription(msg_cls, topic, _cb, 1)
            except Exception as exc:  # noqa: BLE001
                self._watch_error = str(exc)
                self._watch_sub = None
                return {'ok': False, 'message': f'subscribe failed: {exc}'}

        return {
            'ok': True,
            'topic': topic,
            'type': type_name,
            'heavy': heavy,
            'hint': _topic_hint(topic),
            'reused': False,
            'note': 'depth=1 · sample≤4Hz · auto-unsub ~lease' if not heavy else 'heavy type: summary only · sample≤4Hz',
        }

    def _summarize_heavy(self, msg: Any, type_name: str) -> Dict[str, Any]:
        summary: Dict[str, Any] = {'_type': type_name, '_note': '大消息，仅摘要（避免 CPU 峰值）'}
        if type_name == 'sensor_msgs/msg/LaserScan':
            ranges = list(getattr(msg, 'ranges', []) or [])
            finite = [r for r in ranges if r == r and 0.0 < r < 1e6]  # not NaN
            summary.update(
                {
                    'frame_id': getattr(getattr(msg, 'header', None), 'frame_id', ''),
                    'angle_min': float(getattr(msg, 'angle_min', 0.0)),
                    'angle_max': float(getattr(msg, 'angle_max', 0.0)),
                    'range_min': float(getattr(msg, 'range_min', 0.0)),
                    'range_max': float(getattr(msg, 'range_max', 0.0)),
                    'n_ranges': len(ranges),
                    'min_valid': min(finite) if finite else None,
                    'max_valid': max(finite) if finite else None,
                    'sample_ranges': [round(float(r), 3) if r == r else None for r in ranges[:: max(1, len(ranges) // 16)][:16]],
                }
            )
            return summary
        if type_name == 'nav_msgs/msg/OccupancyGrid':
            info = getattr(msg, 'info', None)
            data = list(getattr(msg, 'data', []) or [])
            summary.update(
                {
                    'frame_id': getattr(getattr(msg, 'header', None), 'frame_id', ''),
                    'width': int(getattr(info, 'width', 0) or 0),
                    'height': int(getattr(info, 'height', 0) or 0),
                    'resolution': float(getattr(info, 'resolution', 0.0) or 0.0),
                    'n_cells': len(data),
                }
            )
            return summary
        if type_name in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'):
            summary.update(
                {
                    'frame_id': getattr(getattr(msg, 'header', None), 'frame_id', ''),
                    'width': int(getattr(msg, 'width', 0) or 0),
                    'height': int(getattr(msg, 'height', 0) or 0),
                    'encoding': str(getattr(msg, 'encoding', '') or getattr(msg, 'format', '')),
                    'data_len': len(getattr(msg, 'data', b'') or b''),
                }
            )
            return summary
        if type_name in ('sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/PointCloud'):
            summary.update(
                {
                    'frame_id': getattr(getattr(msg, 'header', None), 'frame_id', ''),
                    'width': int(getattr(msg, 'width', 0) or 0),
                    'height': int(getattr(msg, 'height', 0) or 0),
                    'point_step': int(getattr(msg, 'point_step', 0) or 0),
                    'data_len': len(getattr(msg, 'data', b'') or b''),
                }
            )
            return summary
        if type_name == 'tf2_msgs/msg/TFMessage':
            transforms = list(getattr(msg, 'transforms', []) or [])
            sample = []
            for t in transforms[:12]:
                sample.append(
                    {
                        'parent': getattr(t.header, 'frame_id', ''),
                        'child': getattr(t, 'child_frame_id', ''),
                    }
                )
            summary.update({'n_transforms': len(transforms), 'sample': sample})
            return summary
        # generic heavy: only field names + short scalars
        return _jsonable(msg, max_depth=3, max_list=8)

    def peek_topic(self, renew_lease_sec: float = 45.0) -> Dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            if not self._watch_topic:
                return {'ok': True, 'watching': None, 'data': None}
            self._watch_lease_until = now + max(10.0, float(renew_lease_sec))
            elapsed = max(1e-3, now - (self._watch_started or now))
            hz = self._watch_recv_count / elapsed if self._watch_recv_count else 0.0
            age = (now - self._watch_last_recv) if self._watch_last_recv else None
            return {
                'ok': True,
                'watching': self._watch_topic,
                'type': self._watch_type,
                'heavy': self._watch_heavy,
                'hint': _topic_hint(self._watch_topic or ''),
                'recv_count': self._watch_recv_count,
                'approx_hz': round(hz, 2),
                'age_sec': round(age, 3) if age is not None else None,
                'error': self._watch_error,
                'data': self._watch_data,
                'waiting': self._watch_data is None and not self._watch_error,
            }


class ApiHandler(SimpleHTTPRequestHandler):
    bridge: Optional[BridgeNode] = None

    def end_headers(self) -> None:
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(204)
        self.end_headers()

    def log_message(self, fmt: str, *args) -> None:
        return

    def _json(self, code: int, obj: Dict[str, Any]) -> None:
        body = json.dumps(obj, ensure_ascii=False, default=str).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> Dict[str, Any]:
        n = int(self.headers.get('Content-Length') or 0)
        if n <= 0:
            return {}
        raw = self.rfile.read(n).decode('utf-8', errors='replace')
        try:
            return json.loads(raw) if raw else {}
        except json.JSONDecodeError:
            return {}

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)
        if path == '/api/health':
            if not self.bridge:
                return self._json(503, {'ok': False, 'bridge': False})
            snap = self.bridge.snapshot()
            return self._json(
                200,
                {
                    'ok': True,
                    'bridge': True,
                    'ros_domain_id': snap.get('ros_domain_id'),
                    'services': snap.get('services'),
                    'foxglove': snap.get('foxglove'),
                },
            )
        if path == '/api/state':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.snapshot())
        if path == '/api/foxglove':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, {'ok': True, **self.bridge.foxglove_status()})
        if path == '/api/graph':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            force = (qs.get('force') or ['0'])[0] in ('1', 'true', 'yes')
            return self._json(200, self.bridge.list_graph(force=force))
        if path == '/api/topic/peek':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.peek_topic())
        if path == '/api/pointcloud':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.pointcloud_status())
        if path == '/api/fall':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.fall_status())
        if path == '/api/sensors':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.sensor_hub_status())
        return super().do_GET()

    def do_POST(self) -> None:  # noqa: N802
        if not self.bridge:
            return self._json(503, {'ok': False, 'message': 'bridge offline'})
        path = urlparse(self.path).path
        data = self._read_json()
        if path == '/api/teleop':
            return self._json(
                200,
                self.bridge.publish_teleop(
                    float(data.get('linear_x', 0.0)),
                    float(data.get('angular_z', 0.0)),
                ),
            )
        if path == '/api/motion':
            try:
                angle = float(data.get('angle_deg', data.get('angle', 0.0)))
                dist = float(data.get('distance_m', data.get('distance', 0.0)))
            except (TypeError, ValueError):
                return self._json(400, {'ok': False, 'message': 'angle_deg/distance_m required'})
            return self._json(
                200,
                self.bridge.call_motion(
                    angle,
                    dist,
                    str(data.get('command_id') or ''),
                ),
            )
        if path == '/api/goal':
            try:
                x = float(data.get('x'))
                y = float(data.get('y'))
            except (TypeError, ValueError):
                return self._json(400, {'ok': False, 'message': 'x/y required'})
            yaw = data.get('yaw', data.get('theta', 0.0))
            try:
                yaw_f = float(yaw if yaw is not None else 0.0)
            except (TypeError, ValueError):
                yaw_f = 0.0
            return self._json(
                200,
                self.bridge.publish_goal(
                    x,
                    y,
                    yaw_f,
                    str(data.get('frame_id') or 'map'),
                ),
            )
        if path == '/api/set_mode':
            payload = data.get('payload') if isinstance(data.get('payload'), dict) else {}
            return self._json(
                200,
                self.bridge.set_mode(
                    int(data.get('mode', 0)),
                    payload,
                    str(data.get('command_id') or ''),
                ),
            )
        if path == '/api/run_mode':
            return self._json(
                200,
                self.bridge.set_run_mode(int(data.get('run_mode', 1))),
            )
        if path == '/api/map':
            return self._json(
                200,
                self.bridge.map_manage(
                    int(data.get('operation', 2)),
                    str(data.get('map_name') or ''),
                    str(data.get('new_name') or ''),
                    str(data.get('data_json') or ''),
                ),
            )
        if path == '/api/waypoint':
            return self._json(
                200,
                self.bridge.waypoint_manage(
                    int(data.get('operation', 5)),
                    str(data.get('map_name') or ''),
                    str(data.get('waypoint_name') or ''),
                    str(data.get('new_name') or ''),
                    str(data.get('data_json') or ''),
                ),
            )
        if path == '/api/pointcloud':
            enabled = data.get('enabled')
            if enabled is None:
                enabled = data.get('enable')
            if enabled is None:
                return self._json(400, {'ok': False, 'message': 'missing enabled'})
            return self._json(200, self.bridge.set_pointcloud(bool(enabled)))
        if path == '/api/fall':
            enabled = data.get('enabled')
            if enabled is None:
                enabled = data.get('enable')
            if enabled is None:
                return self._json(400, {'ok': False, 'message': 'missing enabled'})
            return self._json(200, self.bridge.set_fall(bool(enabled)))
        if path == '/api/topic/watch':
            return self._json(
                200,
                self.bridge.watch_topic(
                    str(data.get('topic') or ''),
                    str(data.get('type') or ''),
                    float(data.get('lease_sec') or 45.0),
                ),
            )
        if path == '/api/topic/unwatch':
            return self._json(200, self.bridge.unwatch_topic())
        return self._json(404, {'ok': False, 'message': 'not found'})


def _resolve_web_root(param_root: str) -> Path:
    candidates = [
        Path(param_root) if param_root else None,
        Path('/ros2_ws/install/xw_web/share/xw_web/public'),
        Path(os.environ.get('XW_WS', '/ros2_ws')) / 'src' / 'xw_web' / 'public',
        Path(__file__).resolve().parents[2] / 'public',
    ]
    for c in candidates:
        if c and c.is_dir():
            return c.resolve()
    return Path('/ros2_ws/src/xw_web/public')


def main(args=None) -> None:
    rclpy.init(args=args)
    bridge = BridgeNode()
    bridge.declare_parameter('port', 9000)
    bridge.declare_parameter('web_root', '')

    port = int(bridge.get_parameter('port').value)
    root = _resolve_web_root(str(bridge.get_parameter('web_root').value))
    bridge.get_logger().info(f'HTTP SPA+API :{port} root={root}')

    ApiHandler.bridge = bridge
    handler = functools.partial(ApiHandler, directory=str(root))
    httpd = ThreadingHTTPServer(('0.0.0.0', port), handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    try:
        executor.spin()
    finally:
        httpd.shutdown()
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
