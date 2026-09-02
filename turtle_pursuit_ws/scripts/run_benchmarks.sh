#!/usr/bin/env bash
set -eo pipefail
ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ws_dir/install/setup.bash"
set -u
ros2 run turtle_pursuit benchmark --seeds "${1:-10}" --duration "${2:-180}" --output "$ws_dir/benchmark_results.csv" --require-all-captures
