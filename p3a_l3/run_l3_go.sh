#!/usr/bin/env bash
# P3A L3【正式轮 · PROTOCOL v2】—— 与 v1 的 launch/实参【逐字相同】，只改【起跑协议】
#
# ★ v1（2026-09-28）实测暴露的问题：起跑到推之间有 33 s 站桩，造出 41 个位姿完全相同的
#   起点节点（0.00 m）。这直接导致昨天定案的「回环=0」真机制：
#     (a) 站桩 ⇒ SFM 无运动可依 ⇒ 起点/终点每帧只有 48~71 个 3D（中段 296~378）⇒ 回环两端匹配不上
#     (b) 同一位置连造 41 个同姿态节点 ⇒ 一条回到该簇的回环让 GTSAM 秩亏
#         ⇒ 「Graph optimization failed! Rejecting last loop closures added.」
#   ⇒ 结论：不要在起点/终点站桩。见记忆 robot189-loop-closure-gtsam-rollback。
#
# ★ v2 的三处改动：
#   1) 推车前等待 33 s → 目标 <10 s：安全闸改用【argv 级判据】。
#      昨天已实测证明它是决定性的：remap 一旦落到节点 argv（`odom:=/rtabmap/rgbd_odom`），
#      中间件层就不可能让 rgbd_odometry 再往 /odom 写。话题级核对（发布者计数、/tf）
#      是【确认】不是【判据】，挪到推起来之后做。
#   2) 采样器从坏的 shell 版换成 rclpy 版（odom_sampler.py）：shell 版两个 bug ——
#      grep 缩进写错 + `ros2 topic echo --once` 在 daemon 冷启动时 >12 s 被 timeout 杀掉。
#   3) 打印【回程指令】：到起点不要停，继续推过去 5~10 m。
#
# ★★ 握手协议（v1 起就有，保留）：起跑必须逐字给出用户的放行原话，否则拒绝执行。
#      bash run_l3_go.sh --go '准备好了可以推'
#
# ★ 收尾自检（闸G「几何可比闸门」，2026-10-08 加）：bash run_l3_go.sh --check [--db <路径>]
#   在 stop_l3.sh 收尾后跑 ⇒ 对本轮自己的 DB 只读自检（不起任何节点）：G1 废节点==0、
#   G2 min self-dist<1.0（硬闸，不过=该轮作废）；G3 opt_ids 覆盖度、G4 真回环数（走行口径）为报告项。
set -u
source /ros2_ws/scripts/ros_env.sh
export LC_ALL=C

GO=""
while [ $# -gt 0 ]; do
  case "$1" in
    --go) GO="${2:-}"; shift 2;;
    --no-sampler) SAMPLER=0; shift;;
    --check) CHECK=1; shift;;          # 收尾自检（闸G，只读 DB、不起跑）
    --db) CHECKDB="${2:-}"; shift 2;;  # --check 用的 DB 路径（缺省读 /tmp/p3a_l3_current_db）
    *) echo "未知参数: $1"; shift;;
  esac
done
SAMPLER="${SAMPLER:-1}"
CHECK="${CHECK:-}"; CHECKDB="${CHECKDB:-}"

# ═══════════ ★★★ 闸G：几何可比闸门（收尾自检；2026-10-08 加）═══════════
#   用法（【收尾】时跑：stop_l3.sh 完成之后，对本轮 DB 做几何可比自检）：
#     bash run_l3_go.sh --check                # 用 /tmp/p3a_l3_current_db 指向的本轮 DB
#     bash run_l3_go.sh --check --db <db路径>  # 或任意归档 DB（含 scp 到 165 的）
#   ★ 只读 DB（sqlite mode=ro）、不起任何节点/话题、不写任何文件
#     ⇒ 不需要 --go 放行（那条握手管的是「起跑」，本模式不起跑）。
#   判据出处（全部是项目里已用过的量；要改判据先改
#   /home/cjy/189_p3a_offline/tooling_20261008/gateG_core.py 并留痕，再同步本 heredoc）：
#     G1 同姿态废节点(>=3 连)==0             [硬闸] PHASE3_A_FINDING_2026-09-29.md §4 预登记有效性门
#     G2 min self-dist(|i−j|>=20) < 1.0 m    [硬闸] 同上 §4（口径 = traj_shape.py）
#     G3 Admin.opt_ids 覆盖度/孤立节点数      [报告] FINDING_map_correction_admin_opt_poses_2026-09-30.md §2/§6
#     G4 真回环数【两端走行路径口径】          [报告] FINDING_db_reaudit_words_loopclosure_2026-10-08.md §3.1/§3.5/§6
#        ★ 禁用 Link type=1 裸计数 与 |Δid|>50 代替：都会被同姿态退化边骗
#          （l1.db 有 44 行 |Δid|>50，走行全 0.0 m）。
#   硬闸红 ⇒ 按 §4 预登记：该轮【作废、不解读】。
if [ "$CHECK" = "1" ]; then
  DB="${CHECKDB:-$(cat /tmp/p3a_l3_current_db 2>/dev/null || echo "")}"
  echo "═══════════ 闸G 几何可比自检（收尾） $(date -u +%FT%TZ) ═══════════"
  echo "  DB=${DB:-（/tmp/p3a_l3_current_db 取不到）}"
  if [ -z "$DB" ] || [ ! -f "$DB" ]; then
    echo "  ★★★ 闸G：DB 不存在 —— 先跑一轮（run_l3_go.sh + stop_l3.sh），或用 --db <路径> 指定。"
    exit 3
  fi
  python3 - "$DB" <<'PYG'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ══════════════════════════════════════════════════════════════════════════
# 闸G「几何可比闸门」的核心判据 —— 只读 DB（mode=ro），不起任何节点/话题。
#
# ★ 本文件是【开发/测试副本】；同一段代码逐字嵌入 run_l3_go.sh 的 --check 模式
#   （作为 heredoc 载荷，以 bash run_l3_go.sh --check [--db <路径>] 调用）。
#
# 判据口径与出处（全部取项目里已用过的量，未自创）：
#   G1 同姿态废节点(>=3 连)==0         —— PHASE3_A_FINDING_2026-09-29.md §4 预登记有效性门
#                                        口径实现 = analyze_mf2000.py（3D 距<0.01 m 且 |Δyaw|<0.1°，按 id 连段）
#   G2 min self-dist(|i−j|>=20)<1.0 m —— 同上 §4；口径 = traj_shape.py（按列表序号间隔>=20 取 2D 距）
#   G3 Admin.opt_ids 覆盖度/孤立节点数  —— FINDING_map_correction_admin_opt_poses_2026-09-30.md §2/§6
#                                        （opt_ids = 参与图的节点；zlib 裸流 int32；
#                                          对照 opt_ids 之外: lap2=10（全孤立）/ cov_norm=7（=2 孤立{127,128}+5 站桩簇{2..6}））
#   G4 真回环数【两端走行路径】口径      —— FINDING_db_reaudit_words_loopclosure_2026-10-08.md §3.1/§3.5/§6
#                                        ★ 禁用 Link type=1 裸计数 与 |Δid|>50（都会被同姿态退化边骗：
#                                          l1.db 有 44 行 |Δid|>50，走行全 0.0 m）
#                                        报告两条线：>10 m（§3.5 建议判据）、>3 m（§3.2 表内口径）
#
# G1/G2 是硬闸（红 ⇒ 按 §4 预登记：该轮作废、不解读）；G3/G4 只报告（项目未定阈值，附实测对照值）。
# 退出码：0 = G1、G2 全过；1 = 有硬闸红；3 = DB 打不开/不是 rtabmap 库
# ══════════════════════════════════════════════════════════════════════════
import sqlite3, struct, math, sys, zlib

SEP_G2      = 20     # traj_shape.py 同一常量
WALK_STRICT = 10.0   # FINDING_db_reaudit §3.5「建议改用 走行 >10 m」
WALK_LOOSE  = 3.0    # FINDING_db_reaudit §3.2 表内「走行 >3 m 记为长程」


def wrap180(d):
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def main():
    if len(sys.argv) < 2:
        print("  用法: gateG_core.py <db>")
        return 3
    db = sys.argv[1]
    try:
        con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
        rows = con.execute(
            "SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id").fetchall()
        n_all = con.execute("SELECT COUNT(*) FROM Node").fetchone()[0]
    except Exception as e:
        print("  ★★★ 闸G：DB 打不开 / 不是 rtabmap 库（%s）：%r" % (db, e))
        return 3
    if not rows:
        print("  ★★★ 闸G：Node 表为空（%s）" % db)
        return 3
    ids = [r[0] for r in rows]
    t3, yaw = {}, {}
    for nid, b in rows:
        v = struct.unpack("<12f", b)
        t3[nid] = (v[3], v[7], v[11])              # 3x4 [R|t] 行主序：平移在 [3],[7],[11]
        yaw[nid] = math.degrees(math.atan2(v[4], v[0]))
    extra = ""
    if n_all != len(ids):
        extra = "  ⚠ Node 共 %d 行，其中 pose 为 NULL 的 %d 行已跳过" % (n_all, n_all - len(ids))
    print("  节点 N=%d  (id %d..%d)%s" % (len(ids), ids[0], ids[-1], extra))

    # ── G1 同姿态废节点（口径 = analyze_mf2000.py）────────────────────────
    stall, run = [], 1
    for a, b in zip(ids, ids[1:]):
        if math.dist(t3[a], t3[b]) < 0.01 and abs(wrap180(yaw[a] - yaw[b])) < 0.1:
            run += 1
        else:
            if run >= 3:
                stall.append((a - run + 1, a, run))
            run = 1
    if run >= 3:
        stall.append((ids[-1] - run + 1, ids[-1], run))
    n_waste = sum(x[2] for x in stall)
    g1 = (n_waste == 0)
    print("  闸G1 同姿态废节点(>=3 连) == 0        : %s  %s" % (
        "✓ 0 个" if g1 else "✗ %d 个" % n_waste,
        "（无同姿态簇）" if not stall else "簇: %s" % (stall,)))
    print("        出处: PHASE3_A_FINDING §4 预登记门  对照: mf2000=0(过) / nosfm=98(不过)")

    # ── G2 min self-dist（口径 = traj_shape.py）──────────────────────────
    xy = [(t3[i][0], t3[i][1]) for i in ids]
    best = []
    for i in range(len(xy)):
        for j in range(i + SEP_G2, len(xy)):
            best.append((math.dist(xy[i], xy[j]), ids[i], ids[j]))
    if best:
        best.sort()
        dmin, mi, mj = best[0]
        n_lt1 = sum(1 for d, _, _ in best if d < 1.0)
    else:
        dmin, mi, mj, n_lt1 = float("inf"), -1, -1, 0
    g2 = (dmin < 1.0)
    print("  闸G2 min self-dist(|i−j|>=%d) < 1.0 m : %s 最小 %.3f m (%d <-> %d)  <1.0 m 对数 %d/%d" % (
        SEP_G2, "✓" if g2 else "✗", dmin, mi, mj, n_lt1, len(best)))
    print("        出处: PHASE3_A_FINDING §4 预登记门  对照: mf2000=2.802(不过) / nosfm=0.000(过；站桩簇占据 1↔21)")

    # ── G3 Admin.opt_ids 覆盖度/孤立节点数（出处 = FINDING_map_correction §2/§6）──
    try:
        r = con.execute("SELECT opt_ids FROM Admin").fetchone()
        oids = []
        if r and r[0]:
            ib = zlib.decompress(r[0])
            oids = list(struct.unpack("<%di" % (len(ib) // 4), ib))
        oset = set(oids)
        outside = [i for i in ids if i not in oset]
        if oids:
            print("  闸G3 Admin.opt_ids 覆盖度（参与图节点）: %d/%d = %.1f%%   opt_ids 之外 %d 个%s" % (
                len([i for i in ids if i in oset]), len(ids),
                100.0 * len([i for i in ids if i in oset]) / len(ids),
                len(outside),
                (": %s" % (outside[:12],)) if outside else " （全部参与图）"))
        else:
            print("  闸G3 Admin.opt_ids 覆盖度（参与图节点）: Admin.opt_ids 为空/NULL（该库无优化块）")
        print("        出处: FINDING_map_correction §2/§6（opt_ids=参与图节点；孤立节点=惰性无害）")
        print("              对照: lap2 孤立=10 / cov_norm 孤立=7[{2..6}站桩簇+{127,128}]")
    except Exception as e:
        print("  闸G3 Admin.opt_ids 覆盖度（参与图节点）: 读不到（%r）" % e)

    # ── G4 真回环数【两端走行路径】口径（出处 = FINDING_db_reaudit §3.1/§3.5/§6）──
    cum, c, prev = {}, 0.0, None
    for nid in ids:
        if prev is not None:
            c += math.dist(t3[nid], prev)
        cum[nid] = c
        prev = t3[nid]
    try:
        t1 = con.execute("SELECT from_id, to_id FROM Link WHERE type=1").fetchall()
    except Exception as e:
        t1 = []
        print("  闸G4 真回环（两端走行路径）              : Link 读不到（%r）" % e)
    travs = []
    for a, b in t1:
        if a in cum and b in cum:
            travs.append((abs(cum[a] - cum[b]), a, b))
    n_gt10 = sum(1 for t, _, _ in travs if t > WALK_STRICT)
    n_gt3 = sum(1 for t, _, _ in travs if t > WALK_LOOSE)
    tmax = max((t for t, _, _ in travs), default=0.0)
    tmax_pair = max(travs, default=(0.0, -1, -1))
    print("  闸G4 真回环数【两端走行路径口径】         : type=1 共 %d 行 | 走行>%.0f m(§3.5 建议) = %d 行 | 走行>%.0f m(§3.2 表内) = %d 行" % (
        len(t1), WALK_STRICT, n_gt10, WALK_LOOSE, n_gt3))
    print("       最远走行 %.1f m (%d <-> %d)   ★ 不许用 type=1 裸计数或 |Δid|>50 代替；双向各一行" % (
        tmax, tmax_pair[1], tmax_pair[2]))
    print("        出处: FINDING_db_reaudit §3.1/§3.5/§6  对照: cov_norm 20 行/最远 35.8 m；退化轮 0 行/0.0 m")
    if n_gt10 == 0:
        print("        ⚠ 本轮无「走行>10 m」的真回环 —— 依赖『有真回访』的结论不得用本轮当对照臂（§4 的 mf2000 教训）")
    else:
        show = sorted([t for t in travs if t[0] > WALK_STRICT], reverse=True)[:8]
        for t, a, b in show:
            print("         真回环 %3d <-> %-3d  走行 %.1f m" % (a, b, t))

    con.close()
    hard_red = (not g1) or (not g2)
    print("  ── 闸G 汇总: G1 %s | G2 %s | (G3/G4 为报告项) ──" % (
        "过" if g1 else "红", "过" if g2 else "红"))
    return 1 if hard_red else 0


if __name__ == "__main__":
    sys.exit(main())
PYG
  RC=$?
  case "$RC" in
    0) echo "  ⇒ 闸G 全绿（硬闸 G1/G2 过）—— 该轮几何可比性自检通过（G3/G4 为报告项，见上）。" ;;
    1) echo "  ★★★ 闸G 红了 —— 按 PHASE3_A_FINDING §4 预登记：本轮【作废，不解读】。" ;;
    *) echo "  ★★★ 闸G 没跑成（exit=$RC）—— 自检未完成；不许当「未判定」放过，修好再跑。" ;;
  esac
  exit "$RC"
fi

if [ -z "$GO" ]; then
  cat <<'EOF'
★★ 拒绝起跑：缺少 --go '用户放行原话'
      bash run_l3_go.sh --go '准备好了可以推'
      收尾自检: bash run_l3_go.sh --check   （只对已产出的 DB 做几何可比自检，不起跑）
  授权 ≠ 信号。2026-09-28 我把「实在不行 就再推一轮」（授权）当成执行信号，
  11 秒后自动起跑，比用户放行早了 51 秒，白推一轮。
EOF
  exit 2
fi

TS=$(date -u +%Y%m%d_%H%M%S)
DB=/tmp/p3a_l3go_${TS}.db
LOG=/tmp/p3a_l3go_${TS}.log
SAMP=/tmp/p3a_l3go_${TS}_odom.txt
GATE=/tmp/p3a_l3go_${TS}_gate.txt
echo "$DB"  > /tmp/p3a_l3_current_db
echo "$LOG" > /tmp/p3a_l3_current_log
echo "$SAMP"> /tmp/p3a_l3_current_samp

echo "=== $(date -u +%FT%TZ)  P3A L3 正式轮 PROTOCOL=v2 起跑 ==="
echo "  用户放行原话（逐字）: 「$GO」"
echo "  DB=$DB"
echo "  LOG=$LOG"
ps -eo pid,args --no-headers | grep -a rtabmap | grep -av grep | sed 's/^/  [残留] /'
echo

# ── 预热 ROS daemon：让后面的【确认】阶段的话题查询从 ~20 s 降到 ~2 s ──
#    （v1 的 33 s 里绝大部分是 daemon 冷启动等掉的）
( timeout 60 ros2 topic list >/dev/null 2>&1 ) &
WARM=$!

cd /tmp
nohup ros2 launch /ros2_ws/p3a_l3/rtabmap_l3.launch.py \
  rgb_topic:=/ascamera_hp60c/camera_publisher/rgb0/image \
  depth_topic:=/ascamera_hp60c/camera_publisher/depth0/image_raw \
  camera_info_topic:=/ascamera_hp60c/camera_publisher/rgb0/camera_info \
  depth:=true stereo:=false compressed:=false rgbd_sync:=false \
  visual_odometry:=true icp_odometry:=false \
  publish_tf_odom:=false \
  odom_topic:=rgbd_odom \
  frame_id:=base_link publish_tf_map:=false \
  rtabmap_viz:=false rviz:=false \
  database_path:=$DB \
  approx_sync:=true sync_queue_size:=10 qos:=2 \
  localization:=false wait_for_transform:=0.2 output:=screen >"$LOG" 2>&1 &
LPID=$!
T0=$(date +%s)
echo "  launch PID=$LPID   $(date -u +%FT%TZ)"

for i in $(seq 1 45); do
  grep -aq 'rtabmap (' "$LOG" 2>/dev/null && break
  sleep 1
done
if ! grep -aq 'rtabmap (' "$LOG" 2>/dev/null; then
  echo "  ★★ 等了 $(( $(date +%s) - T0 ))s rtabmap 仍未迭代 —— 日志末 30 行："
  tail -30 "$LOG" | cut -c1-180 | sed 's/^/    /'
  exit 1
fi
echo "  ★ rtabmap 已开始迭代（等了 $(( $(date +%s) - T0 ))s）"
echo

# ═══════════ ★★★ 瞬时安全闸（argv 级，决定性，~0 秒）═══════════
RP=$(pgrep -f 'rtabmap_slam/rtabmap' | head -1)
OP=$(pgrep -f 'rtabmap_odom/rgbd_odometry' | head -1)
if [ -z "${RP:-}" ] || [ -z "${OP:-}" ]; then
  echo "  ★★★ 节点没起全（rtabmap=${RP:-none} rgbd_odometry=${OP:-none}）—— 终止"
  exit 1
fi
RARGV=$(tr '\0' '\n' < "/proc/$RP/cmdline")
OARGV=$(tr '\0' '\n' < "/proc/$OP/cmdline")

# ★ 判据分两类，别混（这是 09-28 差点犯的错）：
#   remap（`odom:=...`，launch 的 remappings=[...]）⇒ 一定在 argv ⇒ 【决定性判据】
#   参数（`publish_tf` / `subscribe_odom_info`，launch 的 parameters=[{...}]）⇒ 走 --params-file，
#        argv 里【根本不会有】⇒ 只看 argv 会误报红、把好的一轮挡掉。必须双查。
PFILES=$(printf '%s\n%s\n' "$OARGV" "$RARGV" | grep -aE 'launch_params|\.ya?ml$' | sed 's/^--params-file=//' | sort -u)
param_all() {   # $1=参数名 → 打印参数文件里该参数出现过的所有取值（去重去引号）
  local n="$1" f
  for f in $PFILES; do [ -f "$f" ] && grep -a -h "^ *$n:" "$f"; done 2>/dev/null \
    | sed "s/^ *$n: *//; s/['\"]//g; s/[[:space:]]*$//" | sort -u | tr '\n' ' '
}

FAIL=0
printf "  闸A rgbd_odometry 输出必须落 /rtabmap/rgbd_odom : "
if echo "$OARGV" | grep -qx 'odom:=/rtabmap/rgbd_odom'; then echo "✓ [remap]"; else echo "✗ 实际: $(echo "$OARGV" | grep -a '^odom:=' || echo 无)"; FAIL=1; fi
printf "  闸B rgbd_odometry 绝不许写 /odom              : "
if echo "$OARGV" | grep -qx 'odom:=/odom'; then echo "✗✗ 撞车！"; FAIL=1; else echo "✓ [remap] 无 odom:=/odom"; fi
printf "  闸C rgbd_odometry 不许发 TF                   : "
if echo "$OARGV" | grep -qx 'publish_tf:=False\|publish_tf:=false'; then echo "✓ [argv]"
else
  C=$(param_all publish_tf)
  case "$C" in
    "false "|"False ") echo "✓ [params-file] $C";;
    "")  echo "… 未确认（argv 与 params-file 都没找到）—— 不阻断，由推起后的 /tf 兜底";;
    *)   echo "✗ 实际 = $C"; FAIL=1;;
  esac
fi
EXP_ODOM_IN="${P3A_ODOM_IN_TOPIC:-/odom}"
# ★ 期望值由环境变量决定（2026-09-30 用户点名授权；不设 = 改前行为 /odom）。
#   仍是硬断言一个确定值，不是放宽。臂别可从本行日志追溯。
printf "  闸D rtabmap 输入必须读 %s（期望，由 P3A_ODOM_IN_TOPIC 决定）           : " "$EXP_ODOM_IN"
if echo "$RARGV" | grep -qx "odom:=$EXP_ODOM_IN"; then echo "✓ [remap]"; else echo "✗ 实际: $(echo "$RARGV" | grep -a '^odom:=' || echo 无)"; FAIL=1; fi
printf "  闸E rtabmap 仍订阅 OdomInfo（SFM 原料）       : "
E=$(param_all subscribe_odom_info)
if echo "$RARGV" | grep -qx 'subscribe_odom_info:=True\|subscribe_odom_info:=true'; then echo "✓ [argv] $E"
elif [ -n "$E" ]; then echo "✓ [params-file] $E"
else echo "… 未确认 —— 由推起后的 OdomInfo.local_map_size 兜底"; fi

# ── 闸F：协方差中继必须活着 + 脚本指纹对（零 DDS 开销 ⇒ 不拖慢"可以推了"）──
#    ★ 发布者计数【故意】不在这里：ros2 topic info 受 daemon 冷启动影响可达 ~20 s，
#      放进闸区会把"可以推了"推迟到 T 之后 ⇒ 挪到推起后的确认块。
printf "  闸F relay 必须活着 + 脚本指纹对             : "
if [ "$EXP_ODOM_IN" != "/odom_cov_norm" ]; then
  echo "… 臂未开（EXP_ODOM_IN=$EXP_ODOM_IN）⇒ 不适用"
else
  FP=$(pgrep -f 'p3a_l3/odom_cov_norm\.py' | head -1)
  FM=$(md5sum /ros2_ws/p3a_l3/odom_cov_norm.py 2>/dev/null | cut -d' ' -f1)
  FL=$(grep -ac 'odom_cov_norm 起' "$LOG" 2>/dev/null)
  if [ -z "${FP:-}" ]; then
    echo "✗ relay 进程不在（launch 里那条 ExecuteProcess 没起来？）"; FAIL=1
  elif [ "$FM" != "7c976740633a2a5d099772688cc0666d" ]; then
    echo "✗ relay 脚本 md5 = ${FM:-拿不到}（应 7c976740633a2a5d099772688cc0666d）"; FAIL=1
  elif [ "${FL:-0}" = "0" ]; then
    echo "✓ [proc] PID=$FP md5✓  ⚠ 日志无起报行（非阻断，由推后发布者计数兜底）"
  else
    echo "✓ [proc] PID=$FP md5✓ 日志起报✓"
  fi
fi
if [ "$FAIL" != "0" ]; then
  echo
  echo "  ★★★ 有闸红了 —— 立刻收尾，不许推。"
  bash /ros2_ws/p3a_l3/stop_l3.sh >/dev/null 2>&1
  exit 1
fi
echo "  ⇒ argv 级全绿（决定性）。起跑到现在 $(( $(date +%s) - T0 ))s"
echo

# ── 采样器（rclpy 版）──
if [ "$SAMPLER" = "1" ]; then
  nohup python3 /ros2_ws/p3a_l3/odom_sampler.py "$SAMP" 2 >/dev/null 2>&1 &
  echo "  采样器已起（rclpy，每 2s）→ $SAMP"
fi

echo "════════════════════════════════════════════════════════"
echo "  ★★★ 可以推了（起跑到现在 $(( $(date +%s) - T0 ))s，v1 是 33s）"
echo "════════════════════════════════════════════════════════"
echo "  ① 立刻开始推，不要等后面那些确认（它们在后台跑，不挡你）"
echo "  ② ★★★ 走满一整圈"
echo "  ③ ★★★ 过了起点【不要停】：一边继续往前走，一边发消息告诉我（「过了」即可）。"
echo "     我收到就【立刻】杀 rtabmap。★★★ 在我杀掉之前【千万别停】。"
echo "     杀掉之后随你停 —— rtabmap 已死，站着不动也不会再产节点。"
echo "     原因：站桩会让 SFM 拿不到 3D，且在同一位置堆出大量同姿态节点。"
echo "     实证：v4 收轮时，人机往返延迟造出 node 75~87 共 13 个位姿【完全相同】的废节点，"
echo "     它们提出的候选（76<->8 … 87<->8）全军覆没。"
echo "     ★ 这条延迟现在反而是好事：你走得越远，收尾的运动节点越多。"
echo

# ═══════════ 推起来之后才做的【确认】（不是判据，红了只是警告）═══════════
{
  echo "=== PROTOCOL v2 确认报告 $(date -u +%FT%TZ) ==="
  echo "--- /odom 发布者（应为 1；变 2 = 生产被污染）---"
  timeout 25 ros2 topic info /odom 2>/dev/null
  if [ "$EXP_ODOM_IN" = "/odom_cov_norm" ]; then
    echo "--- /odom_cov_norm 发布者（闸F 臂开时 应=1）---"
    timeout 25 ros2 topic info /odom_cov_norm 2>/dev/null
  fi
  echo "--- /tf child frame 分布（不许出现 rgbd_odom）---"
  timeout 8 ros2 topic echo /tf 2>/dev/null | grep -a 'child_frame_id' | sort | uniq -c | sort -rn | head -8
  echo "--- 新话题 ---"
  timeout 25 ros2 topic list 2>/dev/null | grep -aE 'rgbd_odom|odom_info'
  echo "--- OdomInfo（local_map_size>0 = SFM 有原料）---"
  OI=$(timeout 25 ros2 topic echo --once /rtabmap/odom_info 2>/dev/null | head -40)
  for F in lost matches inliers features local_map_size; do
    V=$(echo "$OI" | grep -a -m1 "^$F:" | sed "s/^$F: *//")
    printf "    %-18s = %s\n" "$F" "${V:-（拿不到）}"
  done
} >"$GATE" 2>&1
echo "  --- 确认报告（后台跑完，写 $GATE）---"
cat "$GATE" | sed 's/^/    /'
PUB=$(awk '/^--- \/odom 发布者/{f=1;next} /^--- \/tf/{f=0} f' "$GATE" | grep -a -m1 'Publisher count:' | grep -ao '[0-9]*$')
if [ "${PUB:-}" = "1" ]; then
  echo "    ✓ /odom 仍只有 1 个发布者（解耦成功，生产未被污染）"
elif [ -n "${PUB:-}" ]; then
  echo "    ★★★ 警告：/odom 发布者 = $PUB（应=1）！立刻 bash /ros2_ws/p3a_l3/stop_l3.sh"
else
  echo "    … /odom 发布者数没拿到 —— 报告在 $GATE"
fi

if [ "$EXP_ODOM_IN" = "/odom_cov_norm" ]; then
  PUBF=$(awk '/^--- \/odom_cov_norm 发布者/{f=1;next} /^--- \/tf/{f=0} f' "$GATE" | grep -a -m1 'Publisher count:' | grep -ao '[0-9]*$')
  if [ "${PUBF:-}" = "1" ]; then
    echo "    ✓ /odom_cov_norm 发布者=1（relay 真在转发）"
  elif [ -n "${PUBF:-}" ]; then
    echo "    ★★★ 警告：/odom_cov_norm 发布者 = ${PUBF:-}（应=1）！relay 没在转发 ⇒ 本轮里程计输入存疑"
  else
    echo "    … /odom_cov_norm 发布者数没拿到 —— 报告在 $GATE"
  fi
fi
if awk '/^--- \/tf child frame/{f=1;next} /^--- 新话题/{f=0} f' "$GATE" | grep -aq 'rgbd_odom'; then
  echo "    ★★★ 警告：/tf 里出现了 rgbd_odom 帧！立刻 bash /ros2_ws/p3a_l3/stop_l3.sh"
fi
echo
echo "=== $(date -u +%FT%TZ) 起跑完成，共 $(( $(date +%s) - T0 ))s。收尾用 stop_l3.sh ==="
