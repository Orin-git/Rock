#!/bin/bash
# 容器内：正式生产 promote（会话过滤 build_20260924_023803_72cf0b -> v1.7）
set -u
source /ros2_ws/scripts/ros_env.sh
export PYTHONUNBUFFERED=1

PROD=/ros2_ws/maps/vp/visual
MARK=/ros2_ws/mon_20260928/LOAD_MARKER.log
CLI=/ros2_ws/install/xw_global_reloc/lib/xw_global_reloc/visual_db_c1_validate_promote
SID=build_20260924_023803_72cf0b
OUT=/tmp/prod_promote.log

echo "########## A. 前置核对 ##########"
echo "--- 指针 ---"; readlink "$PROD/current_active_version"
echo "--- 闸门（实时） ---"
for T in /xw/localization_status /xw/nav/goals_blocked /xw/localization/phase2c_loc_state; do
  printf "  %-42s " "$T"; timeout 10 ros2 topic echo "$T" --once 2>/dev/null | head -1
done
echo "--- 磁盘 ---"; df -h /ros2_ws | tail -1
echo "--- 备份仍在 ---"; ls -la /ros2_ws/backups/vp_visual_20260928_0134_pre_promote.tar.gz

echo
echo "########## B. 固化 14 条候选 meta.yaml 指纹 ##########"
python3 - <<'PY' > /tmp/cand_before.txt
import hashlib
from pathlib import Path
K = Path('/ros2_ws/maps/vp/visual/candidate/keyframes')
ids = """cand_20260924_023841_521593 cand_20260924_023847_527421 cand_20260924_023857_537832
cand_20260924_023903_543635 cand_20260924_023936_576629 cand_20260924_023941_581893
cand_20260924_024009_609172 cand_20260924_024034_634632 cand_20260924_024240_760619
cand_20260924_024339_819075 cand_20260924_024431_871469 cand_20260924_024437_877435
cand_20260924_024440_880273 cand_20260924_024527_927443 cand_20260922_081633_993685""".split()
for i in sorted(ids):
    p = K / i / 'meta.yaml'
    h = hashlib.sha256(p.read_bytes()).hexdigest()[:16] if p.exists() else '(缺)'
    print(f'{h}  {i}')
PY
cat /tmp/cand_before.txt

echo
echo "########## C. 开始生产 promote（生产默认路径：reload + 回滚演练） ##########"
echo "$(date -u +%FT%TZ) LOAD_WINDOW_START 生产 promote（no nice，含 reload+rollback test）" >> "$MARK"
setsid nohup bash -c '
  timeout 5400 "'"$CLI"'" --maps-dir /ros2_ws/maps --map-name vp --build-session-id "'"$SID"'" > "'"$OUT"'" 2>&1
  echo "EXIT=$? at $(date -u +%FT%TZ)" >> "'"$OUT"'"
  echo "$(date -u +%FT%TZ) LOAD_WINDOW_END 生产 promote 结束" >> "'"$MARK"'"
' >/dev/null 2>&1 &
echo "已后台启动 PID=$!"
sleep 8
echo "--- 8s 后状态 ---"
ps -eo pid,etime,stat,cmd | grep -a "[c]1_validate_promote" || echo "(未见进程)"
head -5 "$OUT" 2>/dev/null
