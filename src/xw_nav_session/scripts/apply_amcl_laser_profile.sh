#!/usr/bin/env bash
# Apply AMCL laser AB profile without restarting Nav2 (best-effort SetParameters).
# Usage: apply_amcl_laser_profile.sh likelihood_field|likelihood_field_prob_beamskip
set -eo pipefail
PROFILE="${1:-likelihood_field}"
case "$PROFILE" in
  likelihood_field|A|test_a|testA)
    ros2 param set /amcl laser_model_type likelihood_field
    ros2 param set /amcl do_beamskip false
    echo "Applied Test A profile: likelihood_field do_beamskip=false"
    ;;
  likelihood_field_prob_beamskip|B|test_b|testB)
    ros2 param set /amcl laser_model_type likelihood_field_prob
    ros2 param set /amcl do_beamskip true
    ros2 param set /amcl beam_skip_distance 0.5
    ros2 param set /amcl beam_skip_threshold 0.3
    ros2 param set /amcl beam_skip_error_threshold 0.9
    echo "Applied Test B profile: likelihood_field_prob do_beamskip=true"
    ;;
  *)
    echo "Unknown profile: $PROFILE" >&2
    echo "Use: likelihood_field | likelihood_field_prob_beamskip" >&2
    exit 2
    ;;
esac
ros2 param get /amcl laser_model_type || true
ros2 param get /amcl do_beamskip || true
