#!/usr/bin/env bash
# Static Depth Legacy vs Direct Remap AB collector (run inside container).
set -eo pipefail
set +u
source /ros2_ws/scripts/ros_env.sh
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
OUT="${1:?out dir}"
LABEL="${2:-depth}"
mkdir -p "$OUT"
DUR="${3:-20}"

{
  echo "=== $LABEL ==="
  date -Iseconds
  echo -n "relay_depth_up="; ros2 param get /xw_depth_topic_bridge relay_depth 2>/dev/null || echo NA
  echo -n "relay_depth_down="; ros2 param get /xw_depth_topic_bridge_front_down relay_depth 2>/dev/null || echo NA
  echo "--- topic info depth image ---"
  ros2 topic info -v /camera/front_up/depth/image_raw 2>&1 | head -55
  echo "--- topic info depth info ---"
  ros2 topic info -v /camera/front_up/depth/camera_info 2>&1 | head -40
  echo "--- encoding / one frame ---"
  timeout 5 ros2 topic echo /camera/front_up/depth/image_raw --once 2>&1 | head -25
  echo "--- camera_info K/D/P ---"
  timeout 5 ros2 topic echo /camera/front_up/depth/camera_info --once 2>&1 | head -55
  echo "--- hz depth ---"
  timeout "$DUR" ros2 topic hz /camera/front_up/depth/image_raw 2>&1 | grep -E "average rate|WARNING" | tail -5 || true
  echo "--- points_nav ---"
  timeout 12 ros2 topic hz /camera/front_up/depth/points_nav 2>&1 | grep -E "average rate|does not|WARNING" | tail -5 || true
  ros2 topic info /camera/front_up/depth/points_nav 2>&1 | head -20 || true
  echo "--- publisher process lines ---"
  ps -eo pid,pcpu,comm,args 2>/dev/null | grep -E "ascamera|depth_topic_bridge" | grep -v grep | head -20 || true
  echo -n "npu="; cat /sys/class/devfreq/fdab0000.npu/load 2>/dev/null || echo NA
  echo -n "loadavg="; cat /proc/loadavg
} | tee "$OUT/${LABEL}.txt"

# Interval stats via Python (stamp-based)
python3 - <<PY | tee -a "$OUT/${LABEL}.txt"
import time, statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

class C(Node):
    def __init__(self):
        super().__init__('depth_interval_probe')
        self.stamps=[]
        qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, '/camera/front_up/depth/image_raw', self.cb, qos)
    def cb(self, msg):
        t=msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.stamps.append(t)

rclpy.init()
n=C(); t0=time.time()
while time.time()-t0 < float("$DUR"):
    rclpy.spin_once(n, timeout_sec=0.2)
st=n.stamps
if len(st)>=3:
    iv=[(st[i]-st[i-1])*1000.0 for i in range(1,len(st)) if st[i]>=st[i-1]]
    iv=sorted(iv)
    def pct(p):
        if not iv: return float('nan')
        k=(len(iv)-1)*p/100.0; f=int(k); c=min(f+1,len(iv)-1)
        return iv[f]*(c-k)+iv[c]*(k-f) if f!=c else iv[f]
    print(f"depth_interval_ms n={len(iv)} p50={pct(50):.1f} p95={pct(95):.1f} p99={pct(99):.1f} max={max(iv):.1f}")
else:
    print(f"depth_interval_ms insufficient stamps n={len(st)}")
n.destroy_node(); rclpy.shutdown()
PY
