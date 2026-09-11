#!/bin/bash
# Extract the D4 evidence in a form that can be analysed offline.
# Runs inside the container. Avoids nested quoting by living in a file.
set -u
L=/ros2_ws/log/nav2_session.launch.log

echo "##### AMCL_LOOKUP_FAILURES #####"
grep -E "Failed to transform initial pose" "$L" \
  | grep -oE "178909[0-9]{4}\.[0-9]+" | while read -r t; do
      printf '%s %s\n' "$(date -d "@${t%.*}" +%H:%M:%S)" "$t"
    done

echo "##### AMCL_NO_POSE #####"
grep -E "AMCL cannot publish a pose" "$L" \
  | grep -oE "178909[0-9]{4}\.[0-9]+" | while read -r t; do
      printf '%s %s\n' "$(date -d "@${t%.*}" +%H:%M:%S)" "$t"
    done

echo "##### AMCL_INITIAL_POSE_RECEIVED #####"
grep -E "initialPoseReceived" "$L" \
  | grep -oE "178909[0-9]{4}\.[0-9]+" | while read -r t; do
      printf '%s %s\n' "$(date -d "@${t%.*}" +%H:%M:%S)" "$t"
    done

echo "##### CTRL_MISSED_RATE_PER_MINUTE #####"
grep -E "Control loop missed its desired rate" "$L" \
  | grep -oE "178909[0-9]{4}" | while read -r t; do
      date -d "@$t" +%H:%M
    done | sort | uniq -c | awk '{print $2, $1}'

echo "##### LOG_WINDOW #####"
head -1 "$L" | grep -oE "178909[0-9]{4}\.[0-9]+" | head -1
tail -1 "$L" | grep -oE "178909[0-9]{4}\.[0-9]+" | head -1
