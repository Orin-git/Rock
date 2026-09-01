#!/usr/bin/env python3
"""SPA + ROS JSON bridge (HTTP) for Xiaowei Gen2 console."""

from __future__ import annotations

import functools
import json
import os
import re
import socket
import subprocess
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import math

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
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

# Task lines → desktop pet / UI log: short plain Chinese only.
_CAPABILITY_ZH = {
    'nav': '导航',
    'navigation': '导航',
    'navigating': '导航',
    'slam': '建图',
    'mapping': '建图',
    'explore': '自主建图',
    'follow': '跟随',
    'following': '跟随',
    'recharge': '回充',
    'motion': '',
    'fall': '跌倒监测',
    'fall_detect': '跌倒监测',
    'idle': '空闲',
    'supervisor': '',
    'localization': '定位',
    'gesture': '手势',
    'pointcloud': '点云',
    'map': '地图',
    'waypoint': '航点',
    'patrol': '巡航',
    'goal': '前往',
    'initialpose': '定位',
    'set_mode': '模式',
    'run_mode': '形态',
}

_PHASE_ZH = {
    'idle': '待命',
    'active': '进行中',
    'executing': '进行中',
    'started': '开始走动',
    'driving': '走动中',
    'done': '走到啦',
    'fwd': '往前走中',
    'back': '往后走中',
    'turn': '转身中',
    'goal_accepted': '开始前往',
    'patrol_start': '开始巡航',
    'patrol_next': '去下一个点',
    'follow_start': '开始跟着你',
    'tracking': '跟着你',
    'follow_nav_executing': '跟着你走',
    'coast': '靠近你',
    'search': '找你中',
    'nav': '去充电桩',
    'detect': '找充电桩',
    'align': '对准中',
    'flip': '掉头中',
    'commit': '贴桩中',
    'retry': '再试一次',
    'success': '充上电了',
    'fail': '回充失败',
    'prep': '准备中',
    'lock': '对准中',
    'retreat': '退开一点',
}

_MODE_ZH = {
    'IDLE': '空闲',
    'MAPPING': '建图',
    'NAVIGATING': '导航',
    'FOLLOWING': '跟随',
    'FALL_DETECT': '跌倒监测',
    'UNKNOWN': '未知',
}

_MSG_ZH = {
    'ok': '好了',
    'done': '走到啦',
    'started': '开始走动',
    'driving': '走动中',
    'preempted': '换了新动作',
    'recharge on': '开始回充',
    'recharge off': '回充已停',
    'follow on': '开始跟着你',
    'follow off': '不跟了',
    'follow started': '开始跟着你',
    'follow stopped': '不跟了',
    'fall=on': '跌倒监测开了',
    'fall=off': '跌倒监测关了',
    'nav started': '开始导航',
    'nav stopped': '导航停了',
    'stopped': '停了',
    'goal succeeded': '到啦',
    'goal failed/aborted': '没走到',
    'patrol complete': '巡航走完了',
    'patrol stopped': '巡航停了',
    'target lost': '找不到人了',
    'nav2 not ready for follow': '导航还没好，先别跟',
    'rejected: follow active': '正跟着人，稍后再试',
    'rejected: recharge active': '正在回充，稍后再试',
    'Nav2 start failed': '导航没启动起来',
    'cannot recharge while mapping': '建图中不能回充',
    'cannot follow while mapping': '建图中不能跟随',
    'explore on': '开始自主建图',
    'explore off': '自主建图已停',
    'enter navigation with a map first (set_mode 2)': '请先进入导航再回充',
    'follow requires navigation map (set_mode 2 with map_name first)': '请先进入导航再跟随',
    'motor disabled (MCU Flag_Stop)': '电机已停，动不了',
    'production': '量产',
    'developer': '开发者',
    'busy': '正忙着',
    'noop': '不用动',
    'no odom yet': '还没准备好',
    'not found': '没找到',
    'renamed': '改名好了',
    'deleted': '删掉了',
    'https :9443 up': '手势开了',
    'https stopped': '手势关了',
}


def _zh_capability(name: str) -> str:
    key = (name or '').strip().lower()
    return _CAPABILITY_ZH.get(key, name or '')


def _zh_phase(phase: str) -> str:
    raw = (phase or '').strip()
    if not raw:
        return '进行中'
    low = raw.lower()
    if low in _PHASE_ZH:
        return _PHASE_ZH[low]
    # motion: "fwd 0.12/0.50m" / "back ..." → no numbers (dedupe-friendly)
    if low.startswith('fwd '):
        return '往前走中'
    if low.startswith('back '):
        return '往后走中'
    if low.startswith('turn'):
        return '转身中'
    return raw


def _zh_message(text: str) -> str:
    raw = (text or '').strip()
    if not raw:
        return ''
    if raw in _MSG_ZH:
        return _MSG_ZH[raw]
    if raw in _MODE_ZH:
        return _MODE_ZH[raw]
    low = raw.lower()
    if low in _MSG_ZH:
        return _MSG_ZH[low]
    m = re.search(r'\(\s*(fwd|back)\s+([\d.]+)\s*m\s*\)', raw, re.I)
    if m:
        dist = m.group(2)
        try:
            d = float(dist)
            dist_s = f'{d:.1f}'.rstrip('0').rstrip('.')
        except ValueError:
            dist_s = dist
        return f'往后走 {dist_s}米' if m.group(1).lower() == 'back' else f'往前走 {dist_s}米'
    if raw.startswith('fall='):
        return '跌倒监测开了' if 'on' in raw else '跌倒监测关了'
    if raw.startswith('busy in '):
        return '正忙着，稍后再试'
    if raw.startswith('accepted '):
        return '收到，开始动'
    if raw.startswith('no waypoints for '):
        return '这个地图还没有航点'
    if raw.startswith('patrol failed at index '):
        return '巡航没走完'
    if raw.startswith('timeout'):
        return '走动超时了'
    if 'service unavailable' in low:
        return '服务还没好'
    if 'timeout' in low:
        return '等超时了'
    return raw


def _is_clean_zh(text: str) -> bool:
    s = (text or '').strip()
    if not s or not any('\u4e00' <= c <= '\u9fff' for c in s):
        return False
    if any(c.isascii() and c.isalpha() for c in s):
        return False
    if '=' in s or '[' in s or ']' in s:
        return False
    return True


def _to_task_zh(line: str) -> str:
    raw = (line or '').strip()
    if not raw:
        return ''
    if _is_clean_zh(raw):
        return raw
    low = raw.lower()
    if 'set_mode' in low or 'active=' in low:
        m = re.search(r'(IDLE|MAPPING|NAVIGATING|FOLLOWING|FALL_DETECT|\b[0-4]\b)', raw, re.I)
        if m:
            key = m.group(1).upper()
            return f'切换到{_MODE_ZH.get(key, _MODE_ZH.get(m.group(1), "新状态"))}'
        return '切换模式了'
    m = re.match(r'^\[progress\]\s*(\w+)\s+(.+)$', raw, re.I)
    if m:
        cap, phase = m.group(1).lower(), m.group(2).strip()
        if cap == 'motion':
            return _zh_phase(phase)
        body = _zh_phase(phase)
        prefix = _zh_capability(cap)
        return body if not prefix else (body if body.startswith(prefix) else f'{prefix}：{body}')
    m = re.match(r'^\[result\]\s*(\w+)\s+code=(\d+)\s*(.*)$', raw, re.I)
    if m:
        cap, code, msg = m.group(1).lower(), int(m.group(2)), m.group(3).strip()
        body = _zh_message(msg) or _zh_phase(msg)
        if cap == 'motion':
            if code == 0:
                return body if _is_clean_zh(body) else '走到啦'
            return body if _is_clean_zh(body) else '没走成'
        prefix = _zh_capability(cap) or '任务'
        if code == 0:
            return body if _is_clean_zh(body) else f'{prefix}好了'
        return body if _is_clean_zh(body) else f'{prefix}失败'
    m = re.match(r'^\[(\w+)\]\s*(.*)$', raw)
    if m:
        return _to_task_zh(f'{m.group(1)} {m.group(2)}'.strip())
    zh = _zh_message(raw)
    if _is_clean_zh(zh):
        return zh
    # Strip Latin leftovers from mixed lines
    stripped = re.sub(r'[A-Za-z][A-Za-z0-9_./-]*', ' ', raw)
    stripped = re.sub(r'[=\[\]<>(){}|]+', ' ', stripped)
    stripped = re.sub(r'\s+', ' ', stripped).strip()
    if _is_clean_zh(stripped):
        return stripped
    return ''


def _format_task_progress(msg: TaskProgress) -> str:
    cap = (msg.capability or '').strip().lower()
    detail = (msg.detail or '').strip()
    phase = (msg.phase or '').strip()
    if cap == 'motion' or phase.lower() in ('started', 'driving', 'done') or phase.lower().startswith(('fwd ', 'back ', 'turn')):
        return _zh_phase(phase) if phase else '走动中'
    if detail and not detail.startswith('{'):
        # recharge already Chinese
        if any('\u4e00' <= ch <= '\u9fff' for ch in detail):
            return detail
        return _zh_message(detail) or _zh_phase(phase)
    body = _zh_phase(phase)
    prefix = _zh_capability(cap)
    if not prefix:
        return body
    if body.startswith(prefix):
        return body
    return f'{prefix}：{body}'


def _format_task_result(msg: TaskResult) -> str:
    cap = (msg.capability or '').strip().lower()
    body = _zh_message(msg.message)
    code = int(getattr(msg, 'code', 0) or 0)
    if cap == 'motion':
        if code == 0:
            return body if body in ('走到啦', '不用动', '好了') else (body or '走到啦')
        if code == 2:
            return '走动取消了'
        return body or '没走成'
    prefix = _zh_capability(cap) or '任务'
    if code == 0:
        return body if body and any('\u4e00' <= ch <= '\u9fff' for ch in body) else f'{prefix}好了'
    if code == 2:
        return f'{prefix}取消了'
    return (body or f'{prefix}失败')


# URDF xw_gen2.urdf — relative to base_link (fill sensors later without UI rewrite)
_SENSOR_LAYOUT = [
    {'id': 'lidar', 'frame': 'lidar_link', 'xyz': [0.0, 0.0, 0.22], 'status': 'live', 'label': '激光雷达'},
    {
        'id': 'camera_front_up',
        'frame': 'camera_front_up_link',
        'xyz': [0.18, 0.0, 0.25],
        'status': 'live',
        'label': '前上深度相机',
    },
    {
        'id': 'camera_front_down',
        'frame': 'camera_front_down_link',
        'xyz': [0.18, 0.0, 0.28],
        'status': 'live',
        'label': '前下深度相机',
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
        'xyz': [0.0, 0.0, 0.10],
        'status': 'live',
        'label': '独立 IMU（WT901C485）',
    },
    {
        'id': 'chassis',
        'frame': 'base_link',
        'xyz': [0.0, 0.0, 0.0],
        'status': 'live',
        'label': '底盘 / odom',
    },
]

GESTURE_PORT = 9443
GESTURE_IDLE_EXIT_S = 10.0


def _gesture_paths() -> Tuple[str, str, str]:
    ws = Path(os.environ.get('XW_WS', '/ros2_ws'))
    share = Path(get_package_share_directory('xw_web'))
    src_public = ws / 'src' / 'xw_web' / 'public'
    web = src_public if src_public.is_dir() else share / 'public'
    src_certs = ws / 'src' / 'xw_web' / 'certs' / 'gesture'
    share_certs = share / 'certs' / 'gesture'
    certs = src_certs if (src_certs / 'cert.pem').is_file() else share_certs
    exe = Path(get_package_prefix('xw_web')) / 'lib' / 'xw_web' / 'gesture_https'
    return str(exe), str(web), str(certs)


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
    '/xw/supervisor/set_follow': '人体跟随正交开关（不拆 Nav2）',
    '/xw/supervisor/set_recharge': '自动回充正交开关（不拆 Nav2）',
    '/xw/supervisor/set_explore': '自主建图正交开关（建图模式 + frontier）',
    '/xw/supervisor/set_fall': '跌倒检测正交开关',
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
    '/xw/cmd/active_source': '当前胜出的速度源名（teleop/nav/…）',
    '/xw/cmd/nav': '导航/CollisionMonitor 输出 → 仲裁',
    '/xw/cmd/recharge': '回充近场速度 → 仲裁',
    '/xw/recharge/enable': '自动回充任务使能',
    '/xw/recharge/status': '回充阶段/成败 JSON（网页状态条）',
    '/xw/recharge/staging': '回充接近点（map）',
    '/xw/recharge/detection': '激光认桩位姿（base_link）',
    '/xw/explore/enable': '自主建图任务使能',
    '/xw/explore/status': '自主建图阶段 JSON',
    '/xw/explore/map_name': '自主建图保存名',
    '/xw/explore/finished': '探索完成信号',
    '/xw/chassis/charge_mode': '底盘 TX[1] 回充模式闩锁',
    '/xw/localization_status': '定位健康 0正常/1未就绪/2漂移自愈/3需重定位',
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
    '/camera/front_up/color/image_raw': '前上彩色图（raw，算法用，不进 Foxglove）',
    '/camera/front_up/color/image_raw/compressed': '前上彩色预览（JPEG，Foxglove Desktop 看）',
    '/camera/front_up/color/camera_info': '前上彩色内参',
    '/camera/front_up/depth/image_raw': '前上深度图（本机算法/安全门）',
    '/camera/front_up/depth/camera_info': '前上深度内参',
    '/camera/front_up/depth/points': '前上点云（调试开关 enable_pointcloud，默认关）',
    '/camera/front_up/depth/points_nav': '前上导航点云（Crop+Voxel+SOR+Radius）',
    '/camera/front_down/depth/points_nav': '前下导航点云（Crop+Voxel+SOR+Radius）',
    '/camera/front_down/color/image_raw': '前下彩色图（raw）',
    '/camera/front_down/color/image_raw/compressed': '前下彩色预览（JPEG）',
    '/camera/front_down/color/camera_info': '前下彩色内参',
    '/camera/front_down/depth/image_raw': '前下深度图',
    '/camera/front_down/depth/camera_info': '前下深度内参',
    '/camera/front_down/depth/points': '前下点云（随点云开关）',
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
    'xw_gesture_https': '手势遥控 HTTPS:9443（按需）',
    'xw_gesture_https_bridge': '手势遥控 ROS 桥（按需）',
    'xw_map_manager': '地图/航点存储与管理',
    'xw_cmd_arbiter': '多路速度指令仲裁',
    'xw_safety_gate': '激光/超声安全门，输出 cmd_vel',
    'xw_chassis': '底盘驱动、里程计、电源、急停',
    'xw_motion': '角度/距离点动',
    'xw_slam_session': '建图会话',
    'xw_nav_session': '导航会话',
    'xw_follow_session': '跟随会话',
    'xw_recharge': '自动回充（Laser-Lock Dock）',
    'xw_explore': '自主建图（frontier）',
    'xw_fall_session': '跌倒巡视会话',
    'xw_perception_stub': '感知（人体/跌倒 stub）',
    'xw_perception': '感知（YOLOv8-pose RKNN → tracks/fall）',
    'xw_sensors': '传感器桩/桥接',
    'xw_depth_topic_bridge': '深度相机话题适配（/camera/front_up）',
    'camera_publisher': 'Angstrong 深度相机驱动',
    'xw_health': '话题健康监测',
    'xw_localization_health': '定位健康 0–3 与自愈',
    'xw_pc_nav_filter': '导航点云 Crop+Voxel+SOR+Radius 过滤',
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
        'localization_status': int(getattr(msg, 'localization_status', 0 if msg.localization_ok else 1)),
        'active_map': msg.active_map,
        'profile': msg.profile,
        'detail': msg.detail,
        'power': {
            'battery_percent': float(msg.power.battery_percent),
            'voltage': float(msg.power.voltage),
            'charging': bool(msg.power.charging),
            'docked': bool(msg.power.docked),
            'charging_current': float(getattr(msg.power, 'charging_current', 0.0) or 0.0),
            'ir_red': int(getattr(msg.power, 'ir_red', 0) or 0),
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


def _json_safe_number(obj: Any) -> Any:
    """Browser JSON.parse rejects NaN/Infinity; map non-finite floats to null."""
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    return obj


def _jsonable(obj: Any, *, depth: int = 0, max_depth: int = 6, max_list: int = 24) -> Any:
    """Convert ROS msg / nested structures to JSON-safe data with hard caps."""
    if depth > max_depth:
        return '…'
    if obj is None or isinstance(obj, (bool, int)):
        return obj
    if isinstance(obj, float):
        return _json_safe_number(obj)
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
            'localization_status': 0,
            'active_map': '',
            'profile': 'normal',
            'detail': 'waiting',
            'power': {
                'battery_percent': 0.0,
                'voltage': 0.0,
                'charging': False,
                'docked': False,
                'charging_current': 0.0,
                'ir_red': 0,
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
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self._patrol_pub = self.create_publisher(String, '/xw/nav/patrol_cmd', 10)
        self._nav_cancel_pub = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self._set_mode = self.create_client(SetMode, '/xw/supervisor/set_mode')
        self._set_run_mode = self.create_client(SetRunMode, '/xw/supervisor/set_run_mode')
        self._map_mgr = self.create_client(MapManage, '/xw/map/manage')
        self._wp_mgr = self.create_client(WaypointManage, '/xw/map/waypoint')
        self._set_pointcloud = self.create_client(SetBool, '/xw/camera/set_pointcloud')
        self._set_fall = self.create_client(SetBool, '/xw/supervisor/set_fall')
        self._set_follow = self.create_client(SetBool, '/xw/supervisor/set_follow')
        self._motion_cli = self.create_client(MotionCommand, '/xw/motion/command')
        self._pointcloud_enabled = False
        self._fall_enabled = False
        self._follow_enabled = False
        self._gesture_proc: Optional[subprocess.Popen] = None
        self._gesture_lock = threading.Lock()
        self._recharge: Dict[str, Any] = {
            'enabled': False,
            'active': False,
            'phase': 'idle',
            'message': '待命',
            'charging': False,
            'retries': 0,
            'result': '',
            'staging': None,
            'label': '待命',
        }
        self._explore: Dict[str, Any] = {
            'enabled': False,
            'active': False,
            'phase': 'idle',
            'message': '待命',
            'map_name': '',
            'iteration': 0,
        }
        self._set_recharge = self.create_client(SetBool, '/xw/supervisor/set_recharge')
        self._set_explore = self.create_client(SetBool, '/xw/supervisor/set_explore')
        self._explore_map_pub = self.create_publisher(String, '/xw/explore/map_name', _LATCHED_BOOL_QOS)
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
        self.create_subscription(
            Bool,
            '/xw/follow/enable',
            self._on_follow_enabled,
            _LATCHED_BOOL_QOS,
        )
        self.create_subscription(
            Bool,
            '/xw/recharge/enable',
            self._on_recharge_enabled,
            _LATCHED_BOOL_QOS,
        )
        self.create_subscription(
            Bool,
            '/xw/explore/enable',
            self._on_explore_enabled,
            _LATCHED_BOOL_QOS,
        )
        self.create_subscription(String, '/xw/recharge/status', self._on_recharge_status, _LATCHED_BOOL_QOS)
        self.create_subscription(String, '/xw/explore/status', self._on_explore_status, _LATCHED_BOOL_QOS)
        self.create_timer(5.0, self._watch_housekeep)
        domain = os.environ.get('ROS_DOMAIN_ID', '?')
        self.get_logger().info(f'web ROS bridge ready (DOMAIN={domain})')

    def _on_pointcloud_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._pointcloud_enabled = bool(msg.data)

    def _on_fall_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._fall_enabled = bool(msg.data)

    def _on_follow_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._follow_enabled = bool(msg.data)

    def _on_recharge_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._recharge['enabled'] = bool(msg.data)
            if not msg.data and self._recharge.get('phase') not in ('fail', 'success'):
                self._recharge['active'] = False

    def _on_recharge_status(self, msg: String) -> None:
        raw = (msg.data or '').strip()
        if not raw:
            return
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return
        if not isinstance(parsed, dict):
            return
        with self._lock:
            self._recharge.update(parsed)

    def _on_explore_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._explore['enabled'] = bool(msg.data)
            if not msg.data and self._explore.get('phase') not in ('fail', 'success'):
                self._explore['active'] = False

    def _on_explore_status(self, msg: String) -> None:
        raw = (msg.data or '').strip()
        if not raw:
            return
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return
        if not isinstance(parsed, dict):
            return
        with self._lock:
            self._explore.update(parsed)

    def pointcloud_status(self) -> Dict[str, Any]:
        with self._lock:
            enabled = bool(self._pointcloud_enabled)
        ready = self._set_pointcloud.service_is_ready()
        return {
            'ok': True,
            'enabled': enabled,
            'service_ready': ready,
            'topic': '/camera/front_up/depth/points',
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
        self._push_task(f'点云调试已{"开启" if enabled else "关闭"}')
        return {
            'ok': bool(res.success),
            'enabled': bool(enabled) if res.success else self._pointcloud_enabled,
            'message': res.message,
            'topic': '/camera/front_up/depth/points',
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
        self._push_task(_zh_message(res.message) or f'跌倒监测已{"开启" if enabled else "关闭"}')
        return {
            'ok': bool(res.success),
            'enabled': bool(enabled) if res.success else self._fall_enabled,
            'message': res.message,
            'topic': '/xw/fall/enable',
        }

    def follow_status(self) -> Dict[str, Any]:
        with self._lock:
            enabled = bool(self._follow_enabled)
            mode = (self._state or {}).get('mode')
        ready = self._set_follow.service_is_ready()
        return {
            'ok': True,
            'enabled': enabled,
            'service_ready': ready,
            'topic': '/xw/follow/enable',
            'mode': mode,
            'hint': '正交任务：需已进导航；开跟随只取消点位/巡航，不拆 Nav2',
        }

    def set_follow(self, enabled: bool) -> Dict[str, Any]:
        if not self._set_follow.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'set_follow service unavailable (supervisor down?)'}
        req = SetBool.Request()
        req.data = bool(enabled)
        fut = self._set_follow.call_async(req)
        for _ in range(60):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'set_follow timeout'}
        res = fut.result()
        with self._lock:
            if res.success:
                self._follow_enabled = bool(enabled)
        self._push_task(_zh_message(res.message) or f'跟随已{"开启" if enabled else "关闭"}')
        return {
            'ok': bool(res.success),
            'enabled': bool(self._follow_enabled),
            'message': res.message,
            'topic': '/xw/follow/enable',
        }

    def _gesture_port_up(self) -> bool:
        try:
            with socket.create_connection(('127.0.0.1', GESTURE_PORT), timeout=0.35):
                return True
        except OSError:
            return False

    def _gesture_proc_alive(self) -> bool:
        p = self._gesture_proc
        return p is not None and p.poll() is None

    def gesture_status(self) -> Dict[str, Any]:
        alive = self._gesture_proc_alive()
        up = self._gesture_port_up()
        return {
            'ok': True,
            'enabled': bool(up),
            'managed': bool(alive),
            'port': GESTURE_PORT,
            'url_path': '/gesture_control.html',
            'idle_exit_s': GESTURE_IDLE_EXIT_S,
            'hint': '按需启动：遥控页打开 HOLO PILOT 时拉起；关页约 10 秒无请求后自动退出',
        }

    def set_gesture(self, enabled: bool) -> Dict[str, Any]:
        with self._gesture_lock:
            if enabled:
                return self._start_gesture_locked()
            return self._stop_gesture_locked()

    def _start_gesture_locked(self) -> Dict[str, Any]:
        if self._gesture_port_up():
            st = self.gesture_status()
            st['message'] = 'already up'
            return st
        if self._gesture_proc is not None and self._gesture_proc.poll() is not None:
            self._gesture_proc = None
        exe, web, certs = _gesture_paths()
        if not os.path.isfile(exe):
            return {'ok': False, 'enabled': False, 'message': f'gesture_https missing: {exe}'}
        env = os.environ.copy()
        env['XW_GESTURE_MANAGED'] = '1'
        log_f = open('/tmp/xw_gesture_https.log', 'ab')
        try:
            self._gesture_proc = subprocess.Popen(
                [
                    exe,
                    '--web-dir', web,
                    '--port', str(GESTURE_PORT),
                    '--cert-dir', certs,
                    '--serve',
                    '--idle-exit', str(int(GESTURE_IDLE_EXIT_S)),
                ],
                env=env,
                stdout=log_f,
                stderr=log_f,
                start_new_session=True,
            )
        except OSError as exc:
            return {'ok': False, 'enabled': False, 'message': str(exc)}
        finally:
            log_f.close()
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if self._gesture_proc.poll() is not None:
                code = self._gesture_proc.returncode
                self._gesture_proc = None
                return {'ok': False, 'enabled': False, 'message': f'gesture_https exited ({code})'}
            if self._gesture_port_up():
                self._push_task('手势服务已启动')
                st = self.gesture_status()
                st['message'] = 'started'
                return st
            time.sleep(0.15)
        return {'ok': False, 'enabled': False, 'message': 'gesture_https start timeout'}

    def _stop_gesture_locked(self) -> Dict[str, Any]:
        p = self._gesture_proc
        self._gesture_proc = None
        if p is not None and p.poll() is None:
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:  # noqa: BLE001
                try:
                    p.kill()
                except Exception:  # noqa: BLE001
                    pass
        self._push_task('手势服务已停止')
        st = self.gesture_status()
        st['ok'] = True
        st['enabled'] = False
        st['message'] = 'stopped'
        return st


    def recharge_status(self) -> Dict[str, Any]:
        with self._lock:
            body = dict(self._recharge)
            mode = (self._state or {}).get('mode')
        ready = self._set_recharge.service_is_ready()
        body.update({
            'ok': True,
            'service_ready': ready,
            'topic': '/xw/recharge/enable',
            'mode': mode,
            'hint': '正交任务：需已进导航；近场走激光反光条锁 odom，不拆 Nav2',
        })
        return body

    def set_recharge(self, enabled: bool) -> Dict[str, Any]:
        if not self._set_recharge.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'set_recharge service unavailable (supervisor down?)'}
        req = SetBool.Request()
        req.data = bool(enabled)
        fut = self._set_recharge.call_async(req)
        for _ in range(60):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'set_recharge timeout'}
        res = fut.result()
        with self._lock:
            if res.success:
                self._recharge['enabled'] = bool(enabled)
                if not enabled and self._recharge.get('phase') not in ('fail', 'success'):
                    self._recharge['phase'] = 'idle'
                    self._recharge['label'] = '待命'
                    self._recharge['active'] = False
        self._push_task(_zh_message(res.message) or f'回充已{"开启" if enabled else "关闭"}')
        out = self.recharge_status()
        out['ok'] = bool(res.success)
        out['message'] = _zh_message(res.message) or res.message
        return out

    def explore_status(self) -> Dict[str, Any]:
        with self._lock:
            body = dict(self._explore)
            mode = (self._state or {}).get('mode')
        ready = self._set_explore.service_is_ready()
        body.update({
            'ok': True,
            'service_ready': ready,
            'topic': '/xw/explore/enable',
            'mode': mode,
            'hint': '正交任务：建图模式 + frontier；Nav2 无 AMCL，跟随 SLAM /map',
        })
        return body

    def set_explore(self, enabled: bool, map_name: str = '') -> Dict[str, Any]:
        name = (map_name or '').strip()
        if enabled and name:
            msg = String()
            msg.data = name
            self._explore_map_pub.publish(msg)
            with self._lock:
                self._explore['map_name'] = name
        if not self._set_explore.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'set_explore service unavailable (supervisor down?)'}
        req = SetBool.Request()
        req.data = bool(enabled)
        fut = self._set_explore.call_async(req)
        for _ in range(80):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'set_explore timeout'}
        res = fut.result()
        with self._lock:
            if res.success:
                self._explore['enabled'] = bool(enabled)
                if not enabled and self._explore.get('phase') not in ('fail', 'success'):
                    self._explore['phase'] = 'idle'
                    self._explore['message'] = '待命'
                    self._explore['active'] = False
        self._push_task(_zh_message(res.message) or f'自主建图已{"开启" if enabled else "关闭"}')
        out = self.explore_status()
        out['ok'] = bool(res.success)
        out['message'] = _zh_message(res.message) or res.message
        return out

    def _push_task(self, line: str) -> None:
        text = _to_task_zh(line)
        if not text:
            return
        with self._lock:
            self._tasks.insert(0, text)
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
        self._push_task(_format_task_progress(msg))

    def _on_result(self, msg: TaskResult) -> None:
        self._push_task(_format_task_result(msg))

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
                'recharge': dict(self._recharge),
                'explore': dict(self._explore),
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
        tip = _zh_message(res.message)
        if not tip or tip == '收到，开始动':
            sign = '往后走' if float(distance_m) < 0 else '往前走'
            tip = f'{sign} {abs(float(distance_m)):.1f}'.rstrip('0').rstrip('.') + '米'
            if abs(float(angle_deg)) > 0.5:
                tip = ('左转' if float(angle_deg) > 0 else '右转') + f'{abs(float(angle_deg)):.0f}°，' + tip
        self._push_task(tip if res.success else f'没动起来 · {tip}')
        return {
            'ok': bool(res.success),
            'message': tip if res.success else res.message,
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
        self._push_task(f'去那边')
        return {
            'ok': True,
            'topic': '/xw/goal_pose',
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
            'frame_id': msg.header.frame_id,
        }

    def publish_initial_pose(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str = 'map',
    ) -> Dict[str, Any]:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id or 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        half = float(yaw) * 0.5
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        # Modest covariance for AMCL particle cloud
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self._initialpose_pub.publish(msg)
        self._push_task('位置定好了')
        return {
            'ok': True,
            'topic': '/initialpose',
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
            'frame_id': msg.header.frame_id,
        }

    def publish_patrol(
        self,
        map_name: str = '',
        loop: bool = False,
        waypoints: Optional[List[str]] = None,
        action: str = 'start',
        command_id: str = '',
    ) -> Dict[str, Any]:
        payload = {
            'action': action or 'start',
            'map_name': map_name or '',
            'loop': bool(loop),
            'command_id': command_id or f'patrol-{int(time.time() * 1000)}',
        }
        if waypoints:
            payload['waypoints'] = list(waypoints)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._patrol_pub.publish(msg)
        act = str(action or 'start')
        if act == 'stop':
            self._push_task('巡航已停止')
        else:
            self._push_task(f'巡航已启动{"（循环）" if loop else ""}')
        return {'ok': True, 'topic': '/xw/nav/patrol_cmd', **payload}

    def cancel_nav(self) -> Dict[str, Any]:
        self._nav_cancel_pub.publish(Bool(data=True))
        self._push_task('导航已取消')
        return {'ok': True, 'topic': '/xw/nav/cancel'}

    def sensor_hub_status(self) -> Dict[str, Any]:
        """Topic presence for nav sensor panel (live + placeholders)."""
        try:
            pairs = self.get_topic_names_and_types()
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'message': str(exc), 'sensors': {}, 'layout': _SENSOR_LAYOUT}
        names = {str(n) for n, _ in pairs}

        def has(*topics: str) -> bool:
            return any(t in names for t in topics)

        def has_publisher(*topics: str) -> bool:
            """True only if someone is publishing (topic name alone can be subscriber-only)."""
            for t in topics:
                try:
                    if self.get_publishers_info_by_topic(t):
                        return True
                except Exception:  # noqa: BLE001
                    continue
            return False

        lidar_live = has_publisher('/scan', 'scan')
        sensors = {
            'lidar': {
                'id': 'lidar',
                'label': '激光雷达',
                'status': 'live' if lidar_live else 'missing',
                'topics': ['/scan'],
                'present': lidar_live,
                'hint': '' if lidar_live else '无 /scan 发布（检查雷达供电+信号线 TX/RX/GND）',
            },
            'depth_camera': {
                'id': 'depth_camera',
                'label': '前上深度',
                'status': 'live'
                if has(
                    '/camera/front_up/color/image_raw/compressed',
                    '/camera/front_up/depth/image_raw',
                )
                else 'missing',
                'topics': [
                    '/camera/front_up/color/image_raw/compressed',
                    '/camera/front_up/depth/image_raw',
                ],
                'present': has(
                    '/camera/front_up/color/image_raw/compressed',
                    '/camera/front_up/depth/image_raw',
                ),
                'pointcloud_enabled': bool(self._pointcloud_enabled),
                'preview': '/camera/front_up/color/image_raw/compressed',
            },
            'depth_camera_2': {
                'id': 'depth_camera_2',
                'label': '前下深度',
                'status': 'live'
                if has(
                    '/camera/front_down/color/image_raw/compressed',
                    '/camera/front_down/depth/image_raw',
                )
                else 'missing',
                'topics': [
                    '/camera/front_down/color/image_raw/compressed',
                    '/camera/front_down/depth/image_raw',
                ],
                'present': has(
                    '/camera/front_down/color/image_raw/compressed',
                    '/camera/front_down/depth/image_raw',
                ),
                'preview': '/camera/front_down/color/image_raw/compressed',
                'hint': '默认关；USB3(speed=5000)后 USE_DEPTH_CAM_2=true',
            },
            'ultrasonic': {
                'id': 'ultrasonic',
                'label': '超声波',
                'status': 'placeholder',
                'topics': ['/ultrasonic_array'],
                # Not fitted: never treat stub/graph presence as online
                'present': False,
                'hint': '未装配，占位',
            },
            'imu': {
                'id': 'imu',
                'label': 'IMU',
                'status': 'live' if has('/imu/data', 'imu/data') else 'missing',
                'topics': ['/imu/data'],
                'present': has('/imu/data', 'imu/data'),
                'hint': 'WT901C485 Modbus → /dev/imu → /imu/data',
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
        mode_zh = _MODE_ZH.get(str(res.message), _zh_message(res.message) or str(res.message))
        self._push_task(f'已切换到{mode_zh}' if res.success else f'模式切换失败 · {mode_zh}')
        return {
            'ok': bool(res.success),
            'message': mode_zh,
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
        self._push_task(f'已切换为{label}形态')
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
        self._push_task(f'地图：{_to_task_zh(res.message) or ("好了" if res.success else "没成功")}')
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
        self._push_task(f'航点：{_to_task_zh(res.message) or ("好了" if res.success else "没成功")}')
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
            finite = [float(r) for r in ranges if math.isfinite(float(r)) and 0.0 < float(r) < 1e6]
            step = max(1, len(ranges) // 16)

            def _sample(r: Any) -> Optional[float]:
                try:
                    v = float(r)
                except (TypeError, ValueError):
                    return None
                return round(v, 3) if math.isfinite(v) else None

            summary.update(
                {
                    'frame_id': getattr(getattr(msg, 'header', None), 'frame_id', ''),
                    'angle_min': _json_safe_number(float(getattr(msg, 'angle_min', 0.0))),
                    'angle_max': _json_safe_number(float(getattr(msg, 'angle_max', 0.0))),
                    'range_min': _json_safe_number(float(getattr(msg, 'range_min', 0.0))),
                    'range_max': _json_safe_number(float(getattr(msg, 'range_max', 0.0))),
                    'n_ranges': len(ranges),
                    'min_valid': min(finite) if finite else None,
                    'max_valid': max(finite) if finite else None,
                    'sample_ranges': [_sample(r) for r in ranges[::step][:16]],
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

    SPA_SHELL_PATHS = frozenset({
        '/',
        '/index.html',
        '/pages/topics.html',
        '/pages/viz.html',
        '/pages/teleop.html',
        '/pages/mapping.html',
        '/pages/navigation.html',
        '/pages/maps.html',
        '/pages/map_beautify.html',
        '/pages/settings.html',
        '/pages/dashboard.html',
    })

    def _serve_spa_shell(self) -> None:
        index = Path(self.directory) / 'index.html'
        if not index.is_file():
            self.send_error(404, 'SPA shell missing')
            return
        try:
            data = index.read_bytes()
        except OSError:
            self.send_error(500, 'read index.html failed')
            return
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

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
        # allow_nan=False: never emit Infinity/NaN (invalid JSON; breaks browser r.json()).
        body = json.dumps(obj, ensure_ascii=False, default=str, allow_nan=False).encode('utf-8')
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
        if path == '/api/follow':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.follow_status())
        if path == '/api/gesture':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.gesture_status())
        if path == '/api/recharge':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.recharge_status())
        if path == '/api/explore':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.explore_status())
        if path == '/api/sensors':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.sensor_hub_status())
        if path in self.SPA_SHELL_PATHS:
            return self._serve_spa_shell()
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
        if path == '/api/initialpose':
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
                self.bridge.publish_initial_pose(
                    x,
                    y,
                    yaw_f,
                    str(data.get('frame_id') or 'map'),
                ),
            )
        if path == '/api/nav/patrol':
            wps = data.get('waypoints')
            if wps is not None and not isinstance(wps, list):
                return self._json(400, {'ok': False, 'message': 'waypoints must be list'})
            return self._json(
                200,
                self.bridge.publish_patrol(
                    str(data.get('map_name') or ''),
                    bool(data.get('loop', False)),
                    wps,
                    str(data.get('action') or 'start'),
                    str(data.get('command_id') or ''),
                ),
            )
        if path == '/api/nav/cancel':
            return self._json(200, self.bridge.cancel_nav())
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
        if path == '/api/follow':
            enabled = data.get('enabled')
            if enabled is None:
                enabled = data.get('enable')
            if enabled is None:
                return self._json(400, {'ok': False, 'message': 'missing enabled'})
            return self._json(200, self.bridge.set_follow(bool(enabled)))
        if path == '/api/gesture':
            enabled = data.get('enabled')
            if enabled is None:
                enabled = data.get('enable')
            if enabled is None:
                return self._json(400, {'ok': False, 'message': 'missing enabled'})
            return self._json(200, self.bridge.set_gesture(bool(enabled)))
        if path == '/api/recharge':
            enabled = data.get('enabled')
            if enabled is None:
                enabled = data.get('enable')
            if enabled is None:
                return self._json(400, {'ok': False, 'message': 'missing enabled'})
            return self._json(200, self.bridge.set_recharge(bool(enabled)))
        if path == '/api/explore':
            enabled = data.get('enabled')
            if enabled is None:
                enabled = data.get('enable')
            if enabled is None:
                return self._json(400, {'ok': False, 'message': 'missing enabled'})
            map_name = str(data.get('map_name') or data.get('map') or '')
            return self._json(
                200,
                self.bridge.set_explore(bool(enabled), map_name=map_name),
            )
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
        Path(os.environ.get('XW_WS', '/ros2_ws')) / 'src' / 'xw_web' / 'public',
        Path(__file__).resolve().parents[2] / 'public',
        Path('/ros2_ws/install/xw_web/share/xw_web/public'),
    ]
    for c in candidates:
        if c and c.is_dir():
            return c.resolve()
    return Path(__file__).resolve().parents[2] / 'public'


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
        try:
            bridge.set_gesture(False)
        except Exception:  # noqa: BLE001
            pass
        httpd.shutdown()
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
