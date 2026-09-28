#!/usr/bin/env bash
# P3A L1 收尾 —— 显式 PID 停 rtabmap + 采日志 + 解析 DB
# kill 逻辑逐字沿用已跑通的 /home/radxa/p3a_harvest_push.sh:16-40
set -u
source /ros2_ws/scripts/ros_env.sh
export LC_ALL=C

DB=$(cat /tmp/p3a_l1_current_db 2>/dev/null || echo "")
LOG=$(cat /tmp/p3a_l1_current_log 2>/dev/null || echo "")
echo "=== $(date -u +%FT%TZ)  L1 收尾 ==="
echo "  DB=$DB"
echo "  LOG=$LOG"
echo

echo "--- 停 rtabmap：先【列举】候选，再用显式 PID ---"
ps -eo pid,ppid,etimes,args --no-headers 2>/dev/null | grep -a rtabmap | grep -av grep | sed 's/^/  /'

RT_PID=$(pgrep -f 'rtabmap_slam/rtabmap' | head -1)
LP_PID=$(pgrep -f 'rtabmap_l1.launch.py' | head -1)
echo "  RT_PID=${RT_PID:-none}   LAUNCH_PID=${LP_PID:-none}"

for P in ${RT_PID:-} ${LP_PID:-}; do
  [ -n "$P" ] || continue
  if kill -TERM "$P" 2>/dev/null; then echo "  TERM -> $P 发出"; else echo "  TERM -> $P 失败"; fi
done
sleep 6
for P in ${RT_PID:-} ${LP_PID:-}; do
  [ -n "$P" ] || continue
  if kill -0 "$P" 2>/dev/null; then
    echo "  $P 仍在，KILL"; kill -KILL "$P" 2>/dev/null
  else
    echo "  $P 已退出"
  fi
done
sleep 3
echo "--- 残留复查（应为空）---"
ps -eo pid,args --no-headers 2>/dev/null | grep -a rtabmap | grep -av grep | sed 's/^/  /' || true
echo "  ^ 空 = 无残留"

echo
echo "=== 日志关键行 ==="
echo "  DB 大小 = $(stat -c%s "$DB" 2>/dev/null || echo '?') bytes"
echo "  日志行数 = $(wc -l < "$LOG" 2>/dev/null || echo 0)"
echo "  'Did not receive data' = $(grep -ac 'Did not receive data' "$LOG" 2>/dev/null || echo 0)"
echo
echo "--- 最后 5 行 ---"
tail -5 "$LOG" 2>/dev/null | sed 's/^/  /'
echo
echo "--- 迭代首末 ---"
grep -a 'rtabmap (' "$LOG" 2>/dev/null | head -1 | sed 's/^/  HEAD /'
grep -a 'rtabmap (' "$LOG" 2>/dev/null | tail -1 | sed 's/^/  TAIL /'
echo
echo "--- ★ 回环 / 拒绝（本次的核心判据）---"
echo "  'Rejected loop closure' 总数 = $(grep -ac 'Rejected loop closure' "$LOG" 2>/dev/null || echo 0)"
echo "  --- 按原因分类 ---"
grep -a 'Rejected loop closure' "$LOG" 2>/dev/null | sed 's/.*Rejected loop closure //' | sed 's/ between .*//' | sed 's/[0-9]\+/N/g' | sort | uniq -c | sed 's/^/    /'
echo "  --- matches 分布（前 40 条）---"
grep -a 'Rejected loop closure' "$LOG" 2>/dev/null | grep -ao 'matches=[0-9]*' | sort | uniq -c | sort -rn | head -20 | sed 's/^/    /'
echo "  --- 任何 'Loop closure detected' / 'Highest hypothesis' ---"
grep -aiE 'loop closure detected|highest hypothesis|Total loop closures' "$LOG" 2>/dev/null | tail -10 | sed 's/^/    /'
echo "  --- WARN/ERROR 末 15 条 ---"
grep -aiE 'WARN|ERROR' "$LOG" 2>/dev/null | tail -15 | sed 's/^/    /'

echo
echo "=== ★ 新 DB 的 Info.parameters 实际生效值（最终裁定）==="
python3 - "$DB" <<'PYEOF'
import sqlite3, re, sys
db = sys.argv[1]
try:
    con = sqlite3.connect(f"file:{db}?immutable=1", uri=True)
    b = con.execute("SELECT parameters FROM Info LIMIT 1").fetchone()[0]
except Exception as e:
    print("  读不到 Info.parameters:", e); raise SystemExit
t = b.decode("utf-8","replace") if isinstance(b,bytes) else str(b)
print("  parameters 长度 =", len(t))
for k in ["Vis/DepthAsMask","Mem/DepthAsMask","Vis/FeatureType","Kp/DetectorStrategy",
          "Vis/MaxFeatures","Kp/MaxFeatures","Vis/EstimationType","Vis/MinInliers",
          "Rtabmap/LoopThr","Mem/StereoFromMotion","OdomF2M/ValidDepthRatio"]:
    m = re.search(re.escape(k) + r"[;=:]\s*([^\n;]*)", t)
    print(f"    {k:32s} = {m.group(1).strip() if m else '★ 未找到'}")
n = con.execute("SELECT COUNT(*) FROM Node").fetchone()[0]
f = con.execute("SELECT COUNT(*) FROM Feature").fetchone()[0]
print(f"  Node = {n}   Feature = {f}   Feature/节点 = {f/max(n,1):.1f}")
for ty, in con.execute("SELECT DISTINCT type FROM Link ORDER BY type"):
    c = con.execute("SELECT COUNT(*) FROM Link WHERE type=?", (ty,)).fetchone()[0]
    print(f"    Link type={ty} : {c}")
print("  --- type=1 回环边明细（前 20）---")
for r in con.execute("SELECT from_id,to_id FROM Link WHERE type=1 ORDER BY from_id,to_id LIMIT 20"):
    print(f"    {r[0]} -> {r[1]}")
n1 = con.execute("SELECT COUNT(*) FROM Link WHERE type=1 AND ((from_id=1 AND to_id=200) OR (from_id=200 AND to_id=1))").fetchone()[0]
print(f"  ★★★ 真回环 (1<->200) 命中 = {n1}")
PYEOF
echo
echo "=== 收尾完成 $(date -u +%FT%TZ) ==="
echo "  按纪律 7：DB 留在 /tmp，若要取出请立刻 scp（/tmp 会随容器重启清空）"
