# Test 1 — Normal Navigation Moving (summary)

- Safety OK from operator; initialpose done; started despite latch status quirk.
- Motion: `/xw/nav/patrol_cmd` loop over `vp` waypoints, ≥320s record window.
- Mid-run: `localization_status` became 0 / `loc=ok`; `cmd_vel` non-zero; pose traversed charger→wp corridor.
- Stopped patrol after record DONE; `cmd_vel` returned to zero.

## Metrics (320s window)

| Metric | Value | Observe target |
|--------|--------|----------------|
| CPU avg/P50/P95/max | **58.3 / 50.6 / 99.5 / 100.0** % | avg&lt;55–60; P95&lt;70 |
| loadavg (cpu sample end) | 10.25 10.85 6.23 | — |
| EKF `/odom` Hz | ≈19.7 | stable |
| EKF miss (&gt;150ms)/min | ≈0.75 (3 gaps / 240s, max gap 0.27s) | low |
| `/scan` Hz | ≈10.0 | ≈10 |
| Depth interval | p50=206 p95=720 **p99=1308 max=1440 ms** | no sustained ≥1s stall |
| map→odom / odom→base | ok throughout series | — |
| AMCL cov (moving) | xx up to ~0.07, yaw cov ~0.01–0.016 | not diverged |
| localization_status | oscillates 0/1 in series; operator notes latch may be wrong | health via AMCL+nav |

## Notes / risks for PASS judgment

1. **CPU P95/max hit ~100%** — above engineering observe line (~70%). Avg still ~58%. Likely load spike (sys loadavg was 22 early). Not auto-FAIL alone; watch Follow tests.
2. **Depth interval max ~1.44s** — meets “秒级 stall” concern in this window; not continuous every frame (p50=206ms).
3. `hz.txt` incomplete for depth topics (parallel timeout contention); EKF/scan covered separately.

Artifacts: `bench/phase1_final_motion_2026-09-07/test1_nav/`
