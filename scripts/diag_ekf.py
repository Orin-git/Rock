"""Read-only: is the EKF's input alive? (chassis wheel odom)"""
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool

rclpy.init()
n = Node('diag_ekf')
topics = ['/odom/wheel', '/odom', '/chassis/status', '/xw/chassis/status', '/diagnostics']
stat = {}
def mk(t, typ):
    st = stat[t] = [0, None, None]
    def cb(m):
        st[0] += 1; st[1] = time.monotonic()
        st[2] = m
    n.create_subscription(typ, t, cb, 10)
mk('/odom/wheel', Odometry)
mk('/odom', Odometry)
for t in ('/chassis/status', '/xw/chassis/status'):
    try: mk(t, String)
    except Exception: pass

t0 = time.monotonic()
while time.monotonic() - t0 < 15.0:
    rclpy.spin_once(n, timeout_sec=0.2)
rclpy.shutdown()

for t, st in stat.items():
    age = 'never' if st[1] is None else f'{time.monotonic()-st[1]:.2f}s ago'
    print(f'{t:20s}: {st[0]:4d} msgs  last={age}')
    m = st[2]
    if m is not None and t == '/odom/wheel':
        tw = m.twist.twist
        print(f'    frame={m.header.frame_id}->{m.child_frame_id} '
              f'vx={tw.linear.x:.3f} wz={tw.angular.z:.3f} '
              f'header_age={(n.get_clock().now().nanoseconds/1e9 - m.header.stamp.sec):.1f}s')
