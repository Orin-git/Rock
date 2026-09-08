#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from xw_interfaces.msg import PowerState

def main():
    rclpy.init()
    n = Node('probe_power')
    got = []
    n.create_subscription(PowerState, '/xw/power', lambda m: got.append(m), 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    print('n', len(got), 'load', open('/proc/loadavg').read().split()[0])
    if got:
        m = got[-1]
        print('charging', m.charging, 'docked', m.docked, 'battery', m.battery_percent)
        print('fields', [a for a in dir(m) if not a.startswith('_')][:40])
    else:
        print('no_power')
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
