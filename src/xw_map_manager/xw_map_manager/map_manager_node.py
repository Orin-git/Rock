#!/usr/bin/env python3
"""Map / waypoint file CRUD under XW_MAPS — gen1-compatible save + pointList."""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Pose2D
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from xw_interfaces.srv import MapManage, WaypointManage

_FORBIDDEN_NAME = re.compile(r'[/\\]')


def _normalize_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


def _charger_nav_yaw(tf_yaw: float) -> float:
    return _normalize_yaw(tf_yaw + math.pi)


def _is_charger_name(name: str) -> bool:
    n = (name or '').strip()
    return n in ('charger', '充电桩')


def sanitize_map_name(raw: str) -> str:
    name = (raw or '').strip()
    if not name or '..' in name or _FORBIDDEN_NAME.search(name):
        raise ValueError('invalid map name')
    return name


def point_list_name(map_name: str) -> str:
    name = map_name.strip()
    if name.endswith('_pointList'):
        return name
    return f'{name}_pointList'


class MapManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_map_manager')
        default_maps = os.environ.get('XW_MAPS', '/ros2_ws/maps')
        self.declare_parameter('maps_dir', default_maps)
        self._root = Path(str(self.get_parameter('maps_dir').value))
        self._root.mkdir(parents=True, exist_ok=True)
        (self._root / 'waypoints').mkdir(exist_ok=True)

        self._cached_start: Optional[Tuple[float, float, float]] = None
        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Pose2D, '/xw/slam/start_pose', self._on_start_pose, latch)
        self._saved_pub = self.create_publisher(Bool, '/xw/slam/map_saved', 10)

        self.create_service(MapManage, '/xw/map/manage', self._on_map)
        self.create_service(WaypointManage, '/xw/map/waypoint', self._on_wp)
        self.get_logger().info(f'map manager root={self._root}')

    def _on_start_pose(self, msg: Pose2D) -> None:
        if math.isnan(msg.x):
            self._cached_start = None
            return
        self._cached_start = (float(msg.x), float(msg.y), float(msg.theta))

    def _wp_path(self, list_name: str) -> Path:
        return self._root / 'waypoints' / f'{list_name}.yaml'

    def _skip_map_stem(self, stem: str) -> bool:
        if stem.startswith('.'):
            return True
        if '_nav_active' in stem:
            return True
        if stem.endswith('_keepout') or '_keepout' in stem:
            return True
        return False

    def _list_maps(self) -> List[str]:
        out: List[str] = []
        for p in sorted(self._root.glob('*.yaml')):
            if self._skip_map_stem(p.stem):
                continue
            out.append(p.stem)
        return out

    def _load_waypoints(self, list_name: str) -> Dict[str, Any]:
        path = self._wp_path(list_name)
        if not path.exists():
            return {'name': list_name, 'waypoints': []}
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        except (OSError, yaml.YAMLError):
            return {'name': list_name, 'waypoints': []}
        if not isinstance(data, dict):
            return {'name': list_name, 'waypoints': []}
        wps = data.get('waypoints') or []
        if not isinstance(wps, list):
            wps = []
        return {'name': data.get('name') or list_name, 'waypoints': wps}

    def _save_waypoints(self, list_name: str, waypoints: List[Dict[str, Any]]) -> None:
        last_charger_idx = -1
        for i, wp in enumerate(waypoints):
            if isinstance(wp, dict) and _is_charger_name(str(wp.get('name', ''))):
                last_charger_idx = i
        cleaned: List[Dict[str, Any]] = []
        for i, wp in enumerate(waypoints):
            if not isinstance(wp, dict):
                continue
            name = str(wp.get('name', '')).strip()
            if _is_charger_name(name):
                if i != last_charger_idx:
                    continue
                name = 'charger'
            cleaned.append({
                'name': name,
                'x': float(wp.get('x', 0.0)),
                'y': float(wp.get('y', 0.0)),
                'yaw': float(wp.get('yaw', 0.0)),
            })
        payload = {'name': list_name, 'waypoints': cleaned}
        path = self._wp_path(list_name)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            yaml.safe_dump(payload, allow_unicode=True, sort_keys=False),
            encoding='utf-8',
        )

    def _upsert_charger(self, map_name: str, x: float, y: float, yaw: float) -> str:
        list_name = point_list_name(map_name)
        data = self._load_waypoints(list_name)
        wps: List[Dict[str, Any]] = list(data.get('waypoints') or [])
        nav_yaw = _charger_nav_yaw(yaw)
        found = False
        for wp in wps:
            if _is_charger_name(str(wp.get('name', ''))):
                wp['name'] = 'charger'
                wp['x'] = x
                wp['y'] = y
                wp['yaw'] = nav_yaw
                found = True
                break
        if not found:
            wps.append({'name': 'charger', 'x': x, 'y': y, 'yaw': nav_yaw})
        self._save_waypoints(list_name, wps)
        return list_name

    def _parse_charger_from_json(self, data_json: str) -> Optional[Tuple[float, float, float]]:
        if not data_json:
            return None
        try:
            data = json.loads(data_json)
        except json.JSONDecodeError:
            return None
        if not isinstance(data, dict):
            return None
        charger = data.get('charger') or data.get('start_pose')
        if not isinstance(charger, dict):
            if all(k in data for k in ('x', 'y', 'yaw')):
                charger = data
            else:
                return None
        try:
            return float(charger['x']), float(charger['y']), float(charger['yaw'])
        except (KeyError, TypeError, ValueError):
            return None

    def _run_map_saver(self, map_name: str) -> Tuple[bool, str]:
        map_path = str(self._root / map_name)
        cmds = [
            [
                'timeout', '20',
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path, '-t', '/map',
                '--ros-args', '-p', 'map_subscribe_transient_local:=false',
            ],
            [
                'timeout', '20',
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path, '-t', '/map',
            ],
        ]
        last_out = ''
        for cmd in cmds:
            try:
                proc = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=25,
                    check=False,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                last_out = str(exc)
                continue
            last_out = (proc.stdout or '') + (proc.stderr or '')
            yaml_f = self._root / f'{map_name}.yaml'
            pgm_f = self._root / f'{map_name}.pgm'
            if proc.returncode == 0 and yaml_f.is_file() and pgm_f.is_file():
                return True, 'ok'
        return False, last_out.strip() or 'map_saver failed'

    def _cascade_rename_waypoints(self, old_map: str, new_map: str) -> None:
        old_list = point_list_name(old_map)
        new_list = point_list_name(new_map)
        src = self._wp_path(old_list)
        if src.exists():
            data = self._load_waypoints(old_list)
            self._save_waypoints(new_list, list(data.get('waypoints') or []))
            if src.resolve() != self._wp_path(new_list).resolve():
                src.unlink(missing_ok=True)

    def _cascade_delete_waypoints(self, map_name: str) -> None:
        self._wp_path(point_list_name(map_name)).unlink(missing_ok=True)

    def _on_map(self, req: MapManage.Request, res: MapManage.Response):
        op = int(req.operation)
        try:
            if op == 2:
                maps = self._list_maps()
                res.success = True
                res.message = 'ok'
                res.map_list = maps
                res.data_json = json.dumps(maps)
                return res

            if op == 1:
                name = sanitize_map_name(req.map_name or '')
                ok, detail = self._run_map_saver(name)
                if not ok:
                    res.success = False
                    res.message = f'保存失败: {detail}'
                    return res

                pose = self._parse_charger_from_json(req.data_json)
                if pose is None:
                    pose = self._cached_start
                if pose is not None:
                    list_name = self._upsert_charger(name, pose[0], pose[1], pose[2])
                    charger_note = f'；charger → {list_name}'
                else:
                    charger_note = '；未写入 charger（无起点位姿）'

                msg = Bool()
                msg.data = True
                self._saved_pub.publish(msg)

                res.success = True
                res.message = f'地图保存成功: {name}{charger_note}'
                res.map_list = [name]
                res.data_json = json.dumps({'map_name': name})
                return res

            if op == 3:
                src_name = sanitize_map_name(req.map_name)
                dst_name = sanitize_map_name(req.new_name)
                src_yaml = self._root / f'{src_name}.yaml'
                if not src_yaml.exists():
                    res.success = False
                    res.message = 'not found'
                    return res
                dst_yaml = self._root / f'{dst_name}.yaml'
                if dst_yaml.exists():
                    res.success = False
                    res.message = 'target exists'
                    return res
                src_yaml.rename(dst_yaml)
                try:
                    meta = yaml.safe_load(dst_yaml.read_text(encoding='utf-8')) or {}
                    if isinstance(meta, dict) and 'image' in meta:
                        meta['image'] = f'{dst_name}.pgm'
                        dst_yaml.write_text(
                            yaml.safe_dump(meta, allow_unicode=True, sort_keys=False),
                            encoding='utf-8',
                        )
                except (OSError, yaml.YAMLError):
                    pass
                src_pgm = self._root / f'{src_name}.pgm'
                if src_pgm.exists():
                    src_pgm.rename(self._root / f'{dst_name}.pgm')
                for ko in list(self._root.glob(f'{src_name}.keepout*')) + list(
                    self._root.glob(f'{src_name}_keepout*')
                ):
                    suffix = ko.name[len(src_name):]
                    ko.rename(self._root / f'{dst_name}{suffix}')
                self._cascade_rename_waypoints(src_name, dst_name)
                res.success = True
                res.message = 'renamed'
                res.map_list = [dst_name]
                return res

            if op == 4:
                name = sanitize_map_name(req.map_name)
                (self._root / f'{name}.yaml').unlink(missing_ok=True)
                (self._root / f'{name}.pgm').unlink(missing_ok=True)
                for ko in list(self._root.glob(f'{name}.keepout*')) + list(
                    self._root.glob(f'{name}_keepout*')
                ):
                    ko.unlink(missing_ok=True)
                self._cascade_delete_waypoints(name)
                res.success = True
                res.message = 'deleted'
                return res

            if op in (5, 6, 7, 8):
                name = sanitize_map_name(req.map_name)
                p = self._root / f'{name}.yaml'
                if op == 5:
                    if not p.exists():
                        res.success = False
                        res.message = 'not found'
                        return res
                    res.success = True
                    res.message = 'ok'
                    res.data_json = p.read_text(encoding='utf-8')
                    return res
                if op == 6:
                    p.write_text(req.data_json or '', encoding='utf-8')
                    res.success = True
                    res.message = 'updated'
                    return res
                ko = self._root / f'{name}.keepout.json'
                if op == 7:
                    res.success = True
                    res.message = 'ok'
                    res.data_json = ko.read_text(encoding='utf-8') if ko.exists() else '{}'
                    return res
                ko.write_text(req.data_json or '{}', encoding='utf-8')
                res.success = True
                res.message = 'keepout updated'
                return res

            res.success = False
            res.message = f'unknown op {op}'
            return res
        except ValueError as exc:
            res.success = False
            res.message = str(exc)
            return res
        except OSError as exc:
            res.success = False
            res.message = str(exc)
            return res

    def _list_point_lists(self) -> List[str]:
        return sorted(p.stem for p in (self._root / 'waypoints').glob('*.yaml'))

    def _on_wp(self, req: WaypointManage.Request, res: WaypointManage.Response):
        op = int(req.operation)
        try:
            if op == 5:
                names = self._list_point_lists()
                res.success = True
                res.message = 'ok'
                res.names = names
                res.data_json = json.dumps(names)
                return res

            raw = (req.map_name or req.waypoint_name or '').strip()
            if not raw:
                res.success = False
                res.message = 'missing list name'
                return res
            list_name = point_list_name(raw) if not raw.endswith('_pointList') else raw

            if op == 2:
                data = self._load_waypoints(list_name)
                res.success = True
                res.message = 'ok'
                res.names = [list_name]
                res.data_json = json.dumps(data, ensure_ascii=False)
                return res

            if op == 1:
                waypoints: List[Dict[str, Any]] = []
                if req.data_json:
                    try:
                        payload = json.loads(req.data_json)
                    except json.JSONDecodeError:
                        res.success = False
                        res.message = 'invalid data_json'
                        return res
                    if isinstance(payload, dict):
                        waypoints = list(payload.get('waypoints') or [])
                    elif isinstance(payload, list):
                        waypoints = payload
                self._save_waypoints(list_name, waypoints)
                res.success = True
                res.message = 'saved'
                res.names = [list_name]
                res.data_json = json.dumps(self._load_waypoints(list_name), ensure_ascii=False)
                return res

            if op == 3:
                self._wp_path(list_name).unlink(missing_ok=True)
                res.success = True
                res.message = 'deleted'
                res.names = self._list_point_lists()
                return res

            if op == 4:
                new_raw = (req.new_name or '').strip()
                if not new_raw:
                    res.success = False
                    res.message = 'missing new_name'
                    return res
                new_list = (
                    point_list_name(new_raw)
                    if not new_raw.endswith('_pointList')
                    else new_raw
                )
                src = self._wp_path(list_name)
                if not src.exists():
                    res.success = False
                    res.message = 'not found'
                    return res
                data = self._load_waypoints(list_name)
                self._save_waypoints(new_list, list(data.get('waypoints') or []))
                if src.resolve() != self._wp_path(new_list).resolve():
                    src.unlink(missing_ok=True)
                res.success = True
                res.message = 'renamed'
                res.names = [new_list]
                return res

            res.success = False
            res.message = f'unknown op {op}'
            return res
        except OSError as exc:
            res.success = False
            res.message = str(exc)
            return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
