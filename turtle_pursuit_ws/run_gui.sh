#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$workspace_dir/scripts/preflight.sh" launch
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"

mode="${1:-round1}"
if [[ "$mode" == "round1" || "$mode" == "round2" || "$mode" == "demo" || "$mode" == "catcher-gauntlet" || "$mode" == "runner-gauntlet" || "$mode" == "hardest" ]]; then shift || true; fi
case "$mode" in
  round1) mode_args=(catcher_strategy:=predictive runner_strategy:=standardized); title="ROUND 1 — CATCHER QUALIFICATION" ;;
  round2) mode_args=(catcher_strategy:=predictive runner_strategy:=competitive); title="OUR CATCHER vs OUR RUNNER — BEST vs BEST" ;;
  demo) mode_args=(catcher_strategy:=predictive runner_strategy:=stationary match_duration:=30.0); title="CAPTURE DEMO" ;;
  catcher-gauntlet) mode_args=(catcher_strategy:=predictive runner_strategy:=competitive); title="CATCHER GAUNTLET — PREDICTIVE vs CANONICAL COMPETITIVE RUNNER" ;;
  runner-gauntlet) mode_args=(catcher_strategy:=aggressive runner_strategy:=competitive); title="OUR COMPETITIVE RUNNER vs HARDEST PRESSURE CATCHER" ;;
  hardest) mode_args=(catcher_strategy:=aggressive runner_strategy:=competitive); title="HARDEST CATCHER vs OUR COMPETITIVE RUNNER" ;;
  *) echo "Use: ./run_gui.sh [round1|round2|demo|catcher-gauntlet|runner-gauntlet|hardest]"; exit 2 ;;
esac

world_args=()
explicit_world=false
for argument in "$@"; do [[ "$argument" == world:=* ]] && explicit_world=true; done
if [[ "$explicit_world" == false ]]; then
  arena_seed="${TURTLE_PURSUIT_ARENA_SEED:-$(date +%s)}"
  random_world_base="/tmp/turtle_pursuit_random_arena_${arena_seed}"
  python3 "$workspace_dir/scripts/generate_random_world.py" \
    --template "$workspace_dir/src/turtle_pursuit/worlds/pursuit_arena.sdf" \
    --output "${random_world_base}.sdf" --seed "$arena_seed" --world-name pursuit_random
  world_args=("world:=$random_world_base" world_name:=pursuit_random "seed:=$arena_seed")
fi

echo "$title"
echo "RED = CATCHER    BLUE = RUNNER    Blue disk = 0.5 m capture zone"
echo "Sensors: FULL (RPLidar + RGB-D + bumper/hazard + pose); override with sensors:=lidar or sensors:=stable"
echo "Arena: randomized every run; set TURTLE_PURSUIT_ARENA_SEED to replay one layout"
echo "Match length: 3 minutes. Hold inside the blue zone for 1 second."
echo "Press Ctrl+C here to stop."

exec ros2 launch turtle_pursuit pursuit.launch.py \
  headless:=false \
  rviz:=false \
  dashboard:=true \
  "scenario:=$title" \
  match_duration:=180.0 \
  "${world_args[@]}" \
  "${mode_args[@]}" \
  "$@"
