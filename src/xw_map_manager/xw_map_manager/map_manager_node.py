#!/usr/bin/env python3
"""Map / waypoint file CRUD under XW_MAPS."""

import json
import os
from pathlib import Path

import rclpy
from rclpy.node import Node

from xw_interfaces.srv import MapManage, WaypointManage


class MapManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_map_manager')
        default_maps = os.environ.get('XW_MAPS', '/ros2_ws/maps')
        self.declare_parameter('maps_dir', default_maps)
        self._root = Path(str(self.get_parameter('maps_dir').value))
        self._root.mkdir(parents=True, exist_ok=True)
        (self._root / 'waypoints').mkdir(exist_ok=True)

        self.create_service(MapManage, '/xw/map/manage', self._on_map)
        self.create_service(WaypointManage, '/xw/map/waypoint', self._on_wp)
        self.get_logger().info(f'map manager root={self._root}')

    def _map_path(self, name: str) -> Path:
        safe = ''.join(c for c in name if c.isalnum() or c in ('-', '_'))
        return self._root / f'{safe}.yaml'

    def _on_map(self, req: MapManage.Request, res: MapManage.Response):
        op = int(req.operation)
        try:
            if op == 2:  # list
                maps = sorted(p.stem for p in self._root.glob('*.yaml'))
                res.success = True
                res.message = 'ok'
                res.map_list = maps
                res.data_json = json.dumps(maps)
                return res
            if op == 1:  # save placeholder
                path = self._map_path(req.map_name or 'untitled')
                meta = {
                    'image': f'{path.stem}.pgm',
                    'resolution': 0.05,
                    'origin': [0.0, 0.0, 0.0],
                    'occupied_thresh': 0.65,
                    'free_thresh': 0.25,
                    'negate': 0,
                    'note': 'placeholder gen2 skeleton',
                }
                if req.data_json:
                    try:
                        meta.update(json.loads(req.data_json))
                    except json.JSONDecodeError:
                        pass
                path.write_text(json.dumps(meta, indent=2) + '\n')
                # minimal pgm companion
                (self._root / f'{path.stem}.pgm').write_bytes(
                    b'P5\n# xw placeholder\n1 1\n255\n\xff'
                )
                res.success = True
                res.message = f'saved {path.name}'
                res.map_list = [path.stem]
                res.data_json = json.dumps(meta)
                return res
            if op == 3:  # rename
                src = self._map_path(req.map_name)
                dst = self._map_path(req.new_name)
                if not src.exists():
                    res.success = False
                    res.message = 'not found'
                    return res
                src.rename(dst)
                pgm = self._root / f'{Path(req.map_name).stem}.pgm'
                if pgm.exists():
                    pgm.rename(self._root / f'{dst.stem}.pgm')
                res.success = True
                res.message = 'renamed'
                return res
            if op == 4:  # delete
                p = self._map_path(req.map_name)
                if p.exists():
                    p.unlink()
                pgm = self._root / f'{p.stem}.pgm'
                if pgm.exists():
                    pgm.unlink()
                res.success = True
                res.message = 'deleted'
                return res
            if op in (5, 6, 7, 8):
                p = self._map_path(req.map_name)
                if op == 5:
                    if not p.exists():
                        res.success = False
                        res.message = 'not found'
                        return res
                    res.success = True
                    res.message = 'ok'
                    res.data_json = p.read_text()
                    return res
                if op == 6:
                    p.write_text(req.data_json or '{}')
                    res.success = True
                    res.message = 'updated'
                    return res
                ko = self._root / f'{p.stem}.keepout.json'
                if op == 7:
                    res.success = True
                    res.message = 'ok'
                    res.data_json = ko.read_text() if ko.exists() else '{}'
                    return res
                if op == 8:
                    ko.write_text(req.data_json or '{}')
                    res.success = True
                    res.message = 'keepout updated'
                    return res
            res.success = False
            res.message = f'unknown op {op}'
            return res
        except OSError as exc:
            res.success = False
            res.message = str(exc)
            return res

    def _wp_file(self, map_name: str) -> Path:
        safe = ''.join(c for c in map_name if c.isalnum() or c in ('-', '_')) or 'default'
        return self._root / 'waypoints' / f'{safe}.json'

    def _on_wp(self, req: WaypointManage.Request, res: WaypointManage.Response):
        op = int(req.operation)
        path = self._wp_file(req.map_name or 'default')
        data = {}
        if path.exists():
            try:
                data = json.loads(path.read_text())
            except json.JSONDecodeError:
                data = {}
        if op == 5:  # list
            names = sorted(data.keys())
            res.success = True
            res.message = 'ok'
            res.names = names
            res.data_json = json.dumps(data)
            return res
        if op == 1:  # save
            name = req.waypoint_name or 'wp'
            try:
                pose = json.loads(req.data_json) if req.data_json else {}
            except json.JSONDecodeError:
                pose = {'raw': req.data_json}
            data[name] = pose
            path.write_text(json.dumps(data, indent=2))
            res.success = True
            res.message = 'saved'
            res.names = sorted(data.keys())
            return res
        if op == 2:  # load one
            if req.waypoint_name not in data:
                res.success = False
                res.message = 'not found'
                return res
            res.success = True
            res.message = 'ok'
            res.names = [req.waypoint_name]
            res.data_json = json.dumps(data[req.waypoint_name])
            return res
        if op == 3:
            data.pop(req.waypoint_name, None)
            path.write_text(json.dumps(data, indent=2))
            res.success = True
            res.message = 'deleted'
            res.names = sorted(data.keys())
            return res
        if op == 4:
            if req.waypoint_name in data:
                data[req.new_name] = data.pop(req.waypoint_name)
                path.write_text(json.dumps(data, indent=2))
            res.success = True
            res.message = 'renamed'
            res.names = sorted(data.keys())
            return res
        res.success = False
        res.message = f'unknown op {op}'
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
