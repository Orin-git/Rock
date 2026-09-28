#!/usr/bin/env bash
# P3A L1 中途体检：rtabmap 到底有没有在收数据
set -u
L=$(cat /tmp/p3a_l1_current_log 2>/dev/null || echo "")
D=$(cat /tmp/p3a_l1_current_db 2>/dev/null || echo "")
echo "  LOG = $L"
echo "  DB  = $D"
echo
if [ -z "$L" ] || [ ! -f "$L" ]; then echo "  ★ 日志文件不存在"; exit 1; fi

echo "  日志行数 = $(wc -l < "$L")"
echo "  DB 大小  = $(stat -c%s "$D" 2>/dev/null || echo '尚未创建') bytes"
echo
echo "  --- 数据接收 ---"
echo "    'Did not receive data' 次数 = $(grep -ac 'Did not receive data' "$L")"
echo "    'rtabmap (' 迭代行数        = $(grep -ac 'rtabmap (' "$L")"
echo "  --- 迭代行 末 3 条 ---"
grep -a 'rtabmap (' "$L" | tail -3 | cut -c1-220 | sed 's/^/    /'
echo
echo "  --- 回环/拒绝 ---"
echo "    'Rejected loop closure' = $(grep -ac 'Rejected loop closure' "$L")"
echo "    'Loop closure detected' = $(grep -ai -c 'loop closure detected' "$L")"
echo "  --- WARN/ERROR 末 8 条 ---"
grep -aiE 'WARN|ERROR' "$L" | tail -8 | cut -c1-220 | sed 's/^/    /'
echo
echo "  --- 末 6 行 ---"
tail -6 "$L" | cut -c1-220 | sed 's/^/    /'
echo
echo "  --- 进程还活着吗 ---"
echo "    rtabmap PID = $(pgrep -f 'rtabmap_slam/rtabmap' | head -1)"
echo
echo "  --- 话题存在性（只 list，不 hz 不 echo —— 红线 4）---"
timeout 15 ros2 topic list 2>/dev/null | grep -a 'ascamera_hp60c/' | sed 's/^/    /'
