import rclpy, time, struct
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

rclpy.init()
n = Node('cm_dump')
got = {}
def cb(m):
    got['m'] = m
n.create_subscription(OccupancyGrid, '/local_costmap/costmap', cb, 10)
t0 = time.time()
while time.time() - t0 < 8 and 'm' not in got:
    rclpy.spin_once(n, timeout_sec=0.5)
if 'm' in got:
    m = got['m']
    w, h = m.info.width, m.info.height
    res = m.info.resolution
    # scan center row (robot row) for free width ahead
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    print(f"map {w}x{h} res={res} origin=({ox:.2f},{oy:.2f})")
    # find occupied cells in 3m ahead (robot at center 2x2 => row index proportional)
    cx = w // 2
    row = cy = h // 2
    occ = []
    for y in range(h):
        for x in range(w):
            v = m.data[y * w + x]
            if v > 50:
                occ.append((x, y, v))
    print(f"occupied cells total: {len(occ)}")
    # gaps: for columns x in [cx-40, cx+40], check column cells within rows cy-20..cy+20
    near = [c for c in occ if abs(c[0] - cx) < 40 and abs(c[1] - cy) < 25]
    print(f"near obstacles: {len(near)}")
    # print obstacle columns distribution along x (map coords)
    from collections import Counter
    col = Counter(c[0] for c in near)
    print("cols:", sorted(col.items())[:30])
else:
    print("no costmap received")
rclpy.shutdown()
