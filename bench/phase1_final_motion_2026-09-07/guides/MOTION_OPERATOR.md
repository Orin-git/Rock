# Phase1 Final Motion Validation — Operator Runbook

**Cursor must NOT start motion without your explicit safety OK.**

Repro: `phase1-hotfix-20260907` @ `cd725fd`  
Artifacts: `/home/radxa/ros2_ws/bench/phase1_final_motion_2026-09-07/`

## Safety gate (required before Test 1/2/3)

Reply in chat with ALL of:

1. 测试区安全确认
2. 急停可用确认
3. 允许开始 Test N（1 / 2 / 3）

Until then: **no NavigateToPose / set_follow / cmd_vel**.

## Preconditions (every test)

1. Map `vp` loaded (NAVIGATING).
2. Place robot at known start.
3. Publish accurate `/initialpose` (Web/Foxglove).
4. Wait until `localization_status=0` (and stays healthy).
5. AMCL params must remain: `laser_model_type=likelihood_field`, `do_beamskip=false`.

Check:

```bash
docker exec -it ros2_humble_dev bash -lc 'source /ros2_ws/scripts/ros_env.sh
ros2 topic echo /xw/localization_status --once
ros2 param get /amcl laser_model_type
ros2 param get /amcl do_beamskip
'
```

## Test 1 — Normal Navigation Moving (≥5 min)

Operator: send multi-waypoint NavigateToPose (Web or nav session).  
Include straight + turns (+ narrow/static obstacles if safe).

Recorder (non-commanding) while robot moves:

```bash
docker exec -it ros2_humble_dev bash -lc '
source /ros2_ws/scripts/ros_env.sh
bash /ros2_ws/bench/phase1_final_motion_2026-09-07/scripts/record_motion_window.sh \
  /ros2_ws/bench/phase1_final_motion_2026-09-07/test1_nav 320 T1_nav
'
```

## Test 2 — Legacy Follow reference (≥30m)

```bash
ros2 param set /xw_follow_session follow_localization_mode legacy_freeze
# start follow via Web / set_follow true; walk straight+turn+brief occlusion
# stop follow, THEN immediately:
python3 /ros2_ws/bench/phase1_final_motion_2026-09-07/scripts/exit_pose_jump.py \
  /ros2_ws/bench/phase1_final_motion_2026-09-07/test2_legacy_follow/exit_jump.txt
# then NavigateToPose to known waypoint; record success/fail
```

During follow, optionally run recorder ~90–120s:

```bash
bash .../record_motion_window.sh .../test2_legacy_follow 120 T2_legacy
```

## Test 3 — Continuous Follow (≥3 successful rounds)

Only change:

```bash
ros2 param set /xw_follow_session follow_localization_mode continuous
```

Repeat Test 2 path for rounds r1/r2/r3 under `test3_continuous/`.  
Target: each ≥30m; cumulative Follow ≥100m preferred.  
Observe Exit Pose Jump: pos&lt;0.25m, yaw&lt;5° (engineering line).

**Do not enable beamskip** on failure — only log cov/scan/CPU/pose/person/loc.

## After PASS

Recommend production default `follow_localization_mode=continuous`, keep `legacy_freeze` rollback; tag candidate.
