#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from xw_interfaces.msg import UltrasonicArray

rclpy.init()
n = Node('us_probe')


def cb(m):
    print(tuple(round(float(x), 2) for x in m.ranges), flush=True)


n.create_subscription(UltrasonicArray, '/ultrasonic_array', cb, 10)
t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.5)
rclpy.shutdown()
