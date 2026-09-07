# Test 2 — Legacy Follow reference (summary)

- Mode: `follow_localization_mode=legacy_freeze`
- AMCL: `likelihood_field`, `do_beamskip=false`
- Follow enabled ~06:41:37 → off ~06:45:15 (~3.5 min)
- Observed `cmd_vel` non-zero mid-run; follow latch confirmed on

## Distance

| Metric | Value |
|--------|--------|
| Start AMCL xy | (0.144, -0.478) |
| End-of-follow AMCL xy | (−6.601, −0.848) |
| **Map-frame displacement** | **6.755 m** |

**Warning:** crow-flies displacement ≪ 30 m. Path length may be longer if route folded, but evidence does **not** prove ≥30 m. Prefer re-run or operator confirmation of walked distance.

## Runtime metrics (150 s record)

| Metric | Value |
|--------|--------|
| CPU avg/P50/P95/max | **64.2 / 53.5 / 99.5 / 100** % |
| `/scan` | ≈9.99 Hz |
| EKF `/odom` | ≈19.94 Hz |
| Depth interval | p50=235 p95=895 **p99=1488 max=2400 ms** |
| AMCL cov at follow end | **xx≈0.55, yy≈0.15** (elevated vs pre ~0.01) |
| Post-nav stability jump (delayed, QoS-fixed) | pos=0.00 m yaw=0.00° but cov still **xx≈0.25 yy≈0.24** |

Ideal Exit Pose Jump at follow-stop t=0 was **missed** (first script QoS VOLATILE vs AMCL TRANSIENT_LOCAL). Delayed sample is post Follow→Nav, not true exit jump.

## Follow → Nav

- Goal: `charger`
- `nav_motion_seen=False` (cmd samples near-zero threshold) but pose moved from (−6.6,−0.85) → **(1.35, 0.49)** near charger
- `nav_success=False` under d&lt;0.5 m gate (final ≈0.75 m from charger)
- **Partial:** navigation progressed toward waypoint without re-`initialpose`; not clean arrival

## Artifacts

`bench/phase1_final_motion_2026-09-07/test2_legacy_follow/`
