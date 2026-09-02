#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
purpose="${1:-all}"
failures=0

pass() { echo "[OK] $*"; }
fail() { echo "[FAIL] $*" >&2; failures=$((failures+1)); }

if [[ -r /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
  pass "ROS 2 Jazzy setup found"
else
  fail "Missing /opt/ros/jazzy/setup.bash (install ROS 2 Jazzy)"
fi

for command_name in ros2 colcon python3 gz; do
  if command -v "$command_name" >/dev/null 2>&1; then
    pass "$command_name is available"
  else
    fail "$command_name is not installed or not on PATH"
  fi
done

required_packages=(ros_gz_sim ros_gz_bridge turtlebot4_gz_bringup turtlebot4_description irobot_create_description irobot_create_control irobot_create_gz_bringup)
for package_name in "${required_packages[@]}"; do
  if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    pass "ROS package $package_name found"
  else
    fail "Required ROS package $package_name is missing"
  fi
done

if [[ -r "$workspace_dir/install/setup.bash" ]]; then
  source "$workspace_dir/install/setup.bash"
  if ros2 pkg prefix turtle_pursuit >/dev/null 2>&1; then
    pass "turtle_pursuit workspace overlay is built"
  else
    fail "Overlay exists but turtle_pursuit is not discoverable; rebuild the workspace"
  fi
else
  fail "Workspace is not built; run: cd $workspace_dir && colcon build --symlink-install"
fi

if find "$workspace_dir/src" -type f \
  -not -path '*/__pycache__/*' \
  -not -path '*/.pytest_cache/*' \
  -not -path '*.egg-info/*' \
  -newer "$workspace_dir/install/setup.bash" -print -quit 2>/dev/null | grep -q .; then
  fail "Source files are newer than the build; run colcon build --symlink-install"
else
  pass "Build is current with source files"
fi

if [[ "$purpose" == "launch" ]] && pgrep -f '[g]z sim' >/dev/null 2>&1; then
  fail "Another Gazebo Sim process is running; stop it before starting a match"
fi

if (( failures > 0 )); then
  echo "Preflight failed with $failures problem(s). Nothing was launched." >&2
  exit 1
fi

echo "Preflight passed."
