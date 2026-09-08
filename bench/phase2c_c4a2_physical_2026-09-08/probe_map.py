#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

L = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

def main():
    rclpy.init()
    n = Node('probe_map')
    got = {'map': None}
    n.create_subscription(OccupancyGrid, '/map', lambda m: got.__setitem__('map', m), L)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 12.0 and got['map'] is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    pubs = n.count_publishers('/map')
    m = got['map']
    print('elapsed', round(time.monotonic()-t0, 2), 'pubs', pubs, 'got', m is not None)
    if m:
        print('wh', m.info.width, m.info.height, 'frame', m.header.frame_id)
    print('load', open('/proc/loadavg').read().split()[0])
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
