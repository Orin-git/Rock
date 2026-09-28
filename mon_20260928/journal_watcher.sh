#!/bin/bash
# 189 宿主 journal 观察器（2026-09-28 09:20-16:30 CST）
# 只读 journalctl；用游标保证窗口【无重叠、无空隙】；每个窗口都打印总行数（分母）
OUT=/home/radxa/ros2_ws/mon_20260928
EV=$OUT/journal_events.log
SINCE=$OUT/.journal_since
HB=$OUT/watcher.hb
END=$(date -d '2026-09-28 16:30:00 +0800' +%s)

mkdir -p "$OUT"

PAT='Took [0-9.]+ seconds|Failed to meet update rate|No valid BMS telemetry|stopStreaming|Stream updated to flag|wlP2p33s0|link becomes ready|link is not ready|Link is Down|retcode -1|InvalidHandle|detected collision ahead|No valid |link up|link down'

echo "$(date -u +%s)" > "$SINCE"

while [ "$(date -u +%s)" -lt "$END" ]; do
  S=$(cat "$SINCE")
  N=$(date -u +%s)
  TMP=$(mktemp)
  journalctl --since "@$S" --until "@$N" --no-pager -o short-iso 2>/dev/null > "$TMP"
  {
    echo "===== window @$S -> @$N ($(date -u -d "@$S" +%FT%TZ) -> $(date -u -d "@$N" +%FT%TZ)) ====="
    echo "# window_total_lines=$(wc -l < "$TMP")   # 分母：0 = journalctl 出问题，不是"没事件""
    grep -aE "$PAT" "$TMP"
  } >> "$EV"
  rm -f "$TMP"
  echo "$N" > "$SINCE"
  echo "$(date -u +%FT%TZ) cursor=$N" > "$HB"
  sleep 300
done

echo "===== WATCHER_STOP $(date -u +%FT%TZ) =====" >> "$EV"
echo "$(date -u +%FT%TZ) STOPPED" > "$HB"
