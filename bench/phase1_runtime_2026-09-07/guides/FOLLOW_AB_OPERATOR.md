# Follow Localization AB — Operator Only (e-stop required)

Cursor must NOT run this unsupervised.

## Preconditions
- Area clear, e-stop ready
- Loc OK (status 0), map `vp`
- DO NOT enable beamskip this round

## TEST A — legacy_freeze
```bash
docker exec -it ros2_humble_dev bash -lc 'source /ros2_ws/scripts/ros_env.sh
ros2 param set /xw_follow_session follow_localization_mode legacy_freeze
ros2 param get /xw_follow_session follow_localization_mode
'
# Start follow via Web UI or set_follow true; walk ≥30m (straight+turn+occlusion)
# Stop follow; at t=0/1/2/5/10s record amcl_pose + localization_status
# Navigate to known waypoint; record success and Exit Pose Jump
```

## TEST B — continuous (only change this)
```bash
ros2 param set /xw_follow_session follow_localization_mode continuous
# Repeat identical route. Engineering threshold: pos jump <0.25m, yaw <5deg
# Repeat ≥3 (prefer 5) before considering default flip
```
