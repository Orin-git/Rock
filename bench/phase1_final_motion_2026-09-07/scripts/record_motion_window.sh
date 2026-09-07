#!/usr/bin/env bash
# Motion validation recorder — SAFE: does NOT command motion.
# Usage (inside container after ros_env):
#   bash record_motion_window.sh <out_dir> <duration_sec> [label]
set -eo pipefail
set +u
source /ros2_ws/scripts/ros_env.sh
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
OUT="${1:?out dir}"
DUR="${2:-60}"
LABEL="${3:-window}"
mkdir -p "$OUT"
TS="$(date -Iseconds)"
echo "RECORD label=$LABEL dur=$DUR ts=$TS" | tee "$OUT/meta.txt"

cpu_sample() {
  python3 /ros2_ws/bench/phase1_hotfix_2026-09-07/scripts/cpu_sample.py "$1" "$2"
}

# Background CPU over full duration
cpu_sample "$OUT/cpu.txt" "$DUR" &
CPU_PID=$!

# Parallel light collectors
{
  echo "===== /scan ====="; timeout "$DUR" ros2 topic hz /scan 2>&1 | grep average | tail -3
  echo "===== /odom ====="; timeout 20 ros2 topic hz /odom 2>&1 | grep average | tail -3
  echo "===== /camera/front_up/depth/image_raw ====="; timeout 20 ros2 topic hz /camera/front_up/depth/image_raw 2>&1 | grep average | tail -3
  echo "===== /camera/front_down/depth/image_raw ====="; timeout 20 ros2 topic hz /camera/front_down/depth/image_raw 2>&1 | grep average | tail -3
} > "$OUT/hz.txt" 2>&1 &
HZ_PID=$!

# EKF / controller hz (best-effort topic names)
{
  echo "===== odom (EKF out) ====="; timeout 25 ros2 topic hz /odom 2>&1 | grep -E "average|WARNING" | tail -5
  echo "===== controller cmd_vel_nav ====="; timeout 25 ros2 topic hz /cmd_vel_nav 2>&1 | grep -E "average|WARNING|does not" | tail -5
  echo "===== controller_server diag if any ====="; timeout 8 ros2 topic list 2>/dev/null | grep -i controller | head -20
} > "$OUT/ekf_ctrl_hz.txt" 2>&1 &
EKF_PID=$!

# Depth interval
python3 - <<'PY' > "$OUT/depth_interval.txt" 2>&1 &
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

class C(Node):
    def __init__(self):
        super().__init__("depth_iv")
        self.st = []
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, "/camera/front_up/depth/image_raw", self.cb, qos)

    def cb(self, msg):
        self.st.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

rclpy.init()
n = C()
t0 = time.time()
dur = float(__import__("os").environ.get("DEPTH_IV_SEC", "45"))
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.2)
st = n.st
iv = sorted([(st[i] - st[i - 1]) * 1000 for i in range(1, len(st)) if st[i] >= st[i - 1]])

def pct(p):
    if not iv:
        return float("nan")
    k = (len(iv) - 1) * p / 100.0
    f = int(k)
    c = min(f + 1, len(iv) - 1)
    return iv[f] * (c - k) + iv[c] * (k - f) if f != c else iv[f]

mx = max(iv) if iv else float("nan")
print("n=%d p50=%.1f p95=%.1f p99=%.1f max=%.1f" % (len(iv), pct(50), pct(95), pct(99), mx))
n.destroy_node()
rclpy.shutdown()
PY
IV_PID=$!

# Pose / loc / cov samples every ~5s for duration
python3 - <<PY > "$OUT/pose_loc_series.txt" 2>&1 &
import time, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int8
from tf2_ros import Buffer, TransformListener, TransformException

class S(Node):
    def __init__(self):
        super().__init__("motion_series")
        self.pose = None
        self.loc = None
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, 10)
        self.create_subscription(Int8, "/xw/localization_status", self.on_loc, 10)

    def on_pose(self, msg):
        self.pose = msg

    def on_loc(self, msg):
        self.loc = int(msg.data)

    def cov_xy_yaw(self):
        if self.pose is None:
            return None
        c = self.pose.pose.covariance
        # indices: xx=0, yy=7, yaw=35
        return float(c[0]), float(c[7]), float(c[35])

    def yaw(self):
        q = self.pose.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

rclpy.init()
n = S()
t0 = time.time()
dur = float("$DUR")
print("t_rel,loc,x,y,yaw,cov_xx,cov_yy,cov_yaw,map_odom_ok,odom_base_ok")
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.1)
    if int((time.time() - t0) * 10) % 50 == 0:  # ~5s
        pass
    # sample every 5s wall
    now = time.time()
    if not hasattr(n, "_next") or now >= n._next:
        n._next = now + 5.0
        mo = ob = "fail"
        try:
            n.tf.lookup_transform("map", "odom", rclpy.time.Time())
            mo = "ok"
        except TransformException:
            mo = "fail"
        try:
            n.tf.lookup_transform("odom", "base_link", rclpy.time.Time())
            ob = "ok"
        except TransformException:
            ob = "fail"
        if n.pose is not None:
            p = n.pose.pose.pose.position
            cov = n.cov_xy_yaw()
            print("%.1f,%s,%.4f,%.4f,%.4f,%.6g,%.6g,%.6g,%s,%s" % (
                now - t0, str(n.loc), p.x, p.y, n.yaw(), cov[0], cov[1], cov[2], mo, ob))
        else:
            print("%.1f,%s,,,,,,,%s,%s" % (now - t0, str(n.loc), mo, ob))
n.destroy_node()
rclpy.shutdown()
PY
SER_PID=$!

# top snapshot mid-window
sleep 5
ps -eo pid,pcpu,comm,args --sort=-pcpu 2>/dev/null | grep -E "ascamera|depth_topic|person_percept|topic_health|ekf_node|amcl|controller|bt_navigator|localization_he|web_server|recharge|slam" | grep -v grep | head -25 > "$OUT/top_key.txt" || true
echo -n "npu=" > "$OUT/sys.txt"
cat /sys/class/devfreq/fdab0000.npu/load 2>/dev/null >> "$OUT/sys.txt" || echo NA >> "$OUT/sys.txt"
echo "loadavg=$(cat /proc/loadavg)" >> "$OUT/sys.txt"

wait $CPU_PID || true
wait $HZ_PID || true
wait $EKF_PID || true
wait $IV_PID || true
wait $SER_PID || true

# one-shot loc/state at end
timeout 3 ros2 topic echo /xw/localization_status --once > "$OUT/loc_end.txt" 2>&1 || true
timeout 3 ros2 topic echo /xw/robot_state --once > "$OUT/state_end.txt" 2>&1 || true
timeout 3 ros2 topic echo /amcl_pose --once > "$OUT/amcl_end.txt" 2>&1 || true

echo DONE | tee -a "$OUT/meta.txt"
