#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"

mode="${1:-round1}"
if [[ "$mode" == "round1" || "$mode" == "round2" || "$mode" == "demo" ]]; then shift || true; fi
case "$mode" in
  round1) mode_args=(catcher_strategy:=predictive runner_strategy:=standardized); title="ROUND 1 — CATCHER QUALIFICATION" ;;
  round2) mode_args=(catcher_strategy:=predictive runner_strategy:=strategic); title="ROUND 2 — AUTONOMOUS 1v1" ;;
  demo) mode_args=(catcher_strategy:=predictive runner_strategy:=stationary match_duration:=30.0); title="CAPTURE DEMO" ;;
  *) echo "Use: ./run_gui.sh [round1|round2|demo]"; exit 2 ;;
esac

echo "$title"
echo "RED = CATCHER    BLUE = RUNNER    Blue disk = 0.5 m capture zone"
echo "Match length: 3 minutes. Hold inside the blue zone for 1 second."
echo "Press Ctrl+C here to stop."

exec ros2 launch turtle_pursuit pursuit.launch.py \
  headless:=false \
  rviz:=false \
  match_duration:=180.0 \
  "${mode_args[@]}" \
  "$@"
