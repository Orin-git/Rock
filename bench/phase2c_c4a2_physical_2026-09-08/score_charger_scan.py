#!/usr/bin/env python3
"""Score live /scan at charger waypoint and last correct pose. No boot, no seed."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from xw_phase2c.laser_prior_verify import verify_pose_with_laser
from xw_global_reloc.laser_verify import DistanceField

L = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
CHARGER = (1.8663955491712294, -0.05958837147746455, -3.1286646850836126)
LAST_OK = (1.7993268507431137, 0.01590625307226233, -2.8414570739649156)

def main():
    rclpy.init()
    n = Node('score_charger_scan')
    got = {}
    n.create_subscription(OccupancyGrid, '/map', lambda m: got.__setitem__('map', m), L)
    n.create_subscription(LaserScan, '/scan', lambda m: got.__setitem__('scan', m), 10)
    n.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: got.__setitem__('amcl', m), L)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0 and not (got.get('map') and got.get('scan')):
        rclpy.spin_once(n, timeout_sec=0.1)
    scan = got.get('scan')
    mp = got.get('map')
    print('load', open('/proc/loadavg').read().split()[0])
    print('have', bool(mp), bool(scan))
    if scan:
        finite = sum(1 for r in scan.ranges if r == r and scan.range_min <= r <= scan.range_max)
        print('scan', scan.header.frame_id, 'n', len(scan.ranges), 'valid', finite, 'stamp', scan.header.stamp.sec)
    if not (scan and mp):
        n.destroy_node(); rclpy.shutdown(); return
    field = DistanceField(mp)
    for name, pose in (('charger_wp', CHARGER), ('last_correct', LAST_OK)):
        r = verify_pose_with_laser(pose, scan, mp, field=field, min_score=0.38)
        print(name, json.dumps({k: r.get(k) for k in ('ok','reason','laser_score','matched_ratio','valid_beams','mean_dist','runtime_sec')}))
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
