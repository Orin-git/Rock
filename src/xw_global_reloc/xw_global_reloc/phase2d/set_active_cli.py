#!/usr/bin/env python3
"""Set Active Visual DB version (atomic pointer) + optional Reloc reload."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from xw_global_reloc.phase2d.version_store import (
    atomic_set_pointer,
    read_pointer_version,
    validate_version_for_activate,
    visual_root,
)


def _call_reload(timeout: float) -> dict:
    """Call /xw/visual_db/reload without breaking an already-running rclpy context.

    When invoked from inside xw_visual_db_build (or any live node), Context is
    already initialized — never call rclpy.init()/shutdown() in that case.
    """
    owned_context = False
    if not rclpy.ok():
        rclpy.init()
        owned_context = True
    node = Node('visual_db_set_active_cli')
    try:
        cli = node.create_client(Trigger, '/xw/visual_db/reload')
        if not cli.wait_for_service(timeout_sec=timeout):
            return {'status': 'RELOAD_SERVICE_UNAVAILABLE'}
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if fut.result() is None:
            return {'status': 'RELOAD_CALL_FAILED'}
        try:
            return json.loads(fut.result().message or '{}')
        except json.JSONDecodeError:
            return {'status': 'RELOAD_BAD_RESPONSE', 'raw': fut.result().message}
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if owned_context and rclpy.ok():
            rclpy.shutdown()


def set_active_version(
    *,
    maps_dir: Path,
    map_name: str,
    version: str,
    reload: bool = True,
    timeout: float = 30.0,
) -> dict:
    vroot = visual_root(maps_dir, map_name)
    old = read_pointer_version(vroot)
    ok, reason = validate_version_for_activate(
        vroot, version, maps_dir=maps_dir, map_name=map_name, require_map_hash_match=True
    )
    if not ok:
        return {
            'status': reason,
            'old_version': old,
            'requested_version': version,
            'active_version_after': old,
        }

    atomic_set_pointer(vroot, version)
    after = read_pointer_version(vroot)
    out = {
        'status': 'POINTER_OK',
        'old_version': old,
        'requested_version': version,
        'active_version_after': after,
    }
    if not reload:
        return out

    try:
        reload_res = _call_reload(timeout)
    except Exception as exc:  # noqa: BLE001
        # Pointer already switched — restore previous Active on any reload path error.
        reload_res = {'status': 'RELOAD_EXCEPTION', 'error': str(exc)}
        if old:
            try:
                atomic_set_pointer(vroot, old)
                out['pointer_restored'] = old
            except Exception as restore_exc:  # noqa: BLE001
                out['pointer_restore_error'] = str(restore_exc)
        out['reload'] = reload_res
        out['status'] = 'RELOAD_EXCEPTION'
        out['active_version_after'] = read_pointer_version(vroot)
        return out

    out['reload'] = reload_res
    st = str(reload_res.get('status') or '')
    if st != 'RELOAD_OK':
        if old:
            try:
                atomic_set_pointer(vroot, old)
                out['pointer_restored'] = old
                out['restore_reload'] = _call_reload(timeout)
            except Exception as exc:  # noqa: BLE001
                out['pointer_restore_error'] = str(exc)
        out['status'] = st or 'RELOAD_FAILED'
        out['active_version_after'] = read_pointer_version(vroot)
        return out
    out['status'] = 'OK'
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='Set current_active_version')
    p.add_argument('version')
    p.add_argument('--maps-dir', default='/ros2_ws/maps')
    p.add_argument('--map-name', default='vp')
    p.add_argument('--no-reload', action='store_true')
    p.add_argument('--timeout', type=float, default=30.0)
    args = p.parse_args(argv)
    out = set_active_version(
        maps_dir=Path(args.maps_dir),
        map_name=args.map_name,
        version=args.version,
        reload=not args.no_reload,
        timeout=args.timeout,
    )
    print(json.dumps(out, indent=2))
    return 0 if out.get('status') in ('OK', 'POINTER_OK') else 1


if __name__ == '__main__':
    raise SystemExit(main())
