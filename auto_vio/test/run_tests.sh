#!/bin/sh
# Build and run every host test. Run from this directory.
set -e
out=$(mktemp -d)
for t in nav_test parse_test turn_test route_test headland_test circle_fit_test steering_test line_test turn_follow_test radius_calibration_test protocol_v2_test safety_test pwm_health_test fault_buffer_test mission_protocol_test direction_test steering_map_test speed_control_test; do
  g++ -std=c++17 -I. -o "$out/$t" "$t.cpp"
  "$out/$t"
done
./replay_test.sh "$out"
echo "--- all host tests passed ---"
