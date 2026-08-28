#!/usr/bin/env bash
set -euo pipefail
ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ws_dir/install/setup.bash"
ros2 run turtle_pursuit benchmark --seeds "${1:-10}" --duration "${2:-180}" --output "$ws_dir/benchmark_results.csv"
