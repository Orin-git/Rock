#!/usr/bin/env python3
"""One-shot CLI: request Candidate capture (or dry-run) from xw_visual_db_capture.

Usage:
  ros2 run xw_global_reloc visual_db_capture_candidate
  ros2 run xw_global_reloc visual_db_capture_candidate --dry-run
  # Or standalone one-shot node (no long-running capture node required):
  ros2 run xw_global_reloc visual_db_capture_candidate --oneshot [--dry-run]
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _via_service(dry_run: bool, timeout: float) -> int:
    rclpy.init()
    node = Node('visual_db_capture_cli')
    # Temporarily set remote dry_run via publishing then calling service is awkward;
    # prefer topic JSON command + wait for result.
    pub = node.create_publisher(String, '/xw/visual_db/capture_candidate', 10)
    result_holder = {'raw': None}

    def _on_res(msg: String) -> None:
        result_holder['raw'] = msg.data

    node.create_subscription(String, '/xw/visual_db/capture_result', _on_res, 10)
    # Wait for discovery
    time.sleep(0.4)
    pub.publish(String(data=json.dumps({'dry_run': dry_run, 'source': 'manual_test'})))
    t0 = time.time()
    while result_holder['raw'] is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if result_holder['raw'] is None:
        print('ERROR: no capture_result (is xw_visual_db_capture running?)', file=sys.stderr)
        return 2
    print(result_holder['raw'])
    try:
        data = json.loads(result_holder['raw'])
        return 0 if data.get('status') == 'ACCEPTED' else 1
    except json.JSONDecodeError:
        return 1


def _oneshot(dry_run: bool) -> int:
    from xw_global_reloc.phase2d.capture_candidate_node import VisualDbCaptureNode

    rclpy.init()
    node = VisualDbCaptureNode()
    # Allow latched state to arrive
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = node.capture_once(dry_run=dry_run, source='manual_test')
    print(json.dumps(result.to_dict(), indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result.status == 'ACCEPTED' else 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='Phase2D-A2 Candidate capture')
    p.add_argument('--dry-run', action='store_true', help='Run all gates; do not write disk')
    p.add_argument('--oneshot', action='store_true', help='Spin ephemeral capture node')
    p.add_argument('--timeout', type=float, default=15.0)
    p.add_argument('--trigger-svc', action='store_true', help='Call Trigger service instead of topic')
    args = p.parse_args(argv)

    if args.oneshot:
        return _oneshot(args.dry_run)

    if args.trigger_svc:
        rclpy.init()
        node = Node('visual_db_capture_cli_svc')
        if args.dry_run:
            # Service uses node dry_run_default; topic path is preferred for dry_run.
            print('NOTE: use topic/--oneshot for dry_run; calling service with dry_run_default', file=sys.stderr)
        cli = node.create_client(Trigger, '/xw/visual_db/capture_candidate_svc')
        if not cli.wait_for_service(timeout_sec=args.timeout):
            print('ERROR: service unavailable', file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 2
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
        node.destroy_node()
        rclpy.shutdown()
        if fut.result() is None:
            print('ERROR: service call failed', file=sys.stderr)
            return 2
        print(fut.result().message)
        return 0 if fut.result().success else 1

    return _via_service(args.dry_run, args.timeout)


if __name__ == '__main__':
    raise SystemExit(main())
