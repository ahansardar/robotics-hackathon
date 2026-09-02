#!/usr/bin/env bash
set -eo pipefail

profile="${1:-full}"
case "$profile" in
  stable|lidar|full) ;;
  *) echo "Use: $0 [stable|lidar|full]" >&2; exit 2 ;;
esac

source /opt/ros/jazzy/setup.bash
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/install/setup.bash"

failures=0
deadline=$((SECONDS+${TURTLE_PURSUIT_SENSOR_WAIT:-50}))
check_topic() {
  while (( SECONDS < deadline )); do
    if timeout 3s ros2 topic echo --once "$1" >/dev/null 2>&1; then
      echo "[OK] $1"
      return
    fi
  done
  echo "[FAIL] no message on $1" >&2
  failures=$((failures+1))
}

check_topic /match/catcher_mode
check_topic /match/runner_mode
if [[ "$profile" != "stable" ]]; then
  check_topic /catcher/scan
  check_topic /runner/scan
fi
if [[ "$profile" == "full" ]]; then
  check_topic /catcher/camera/color/image_raw
  check_topic /runner/camera/color/image_raw
  check_topic /catcher/camera/depth/image_raw
  check_topic /runner/camera/depth/image_raw
fi

if (( failures > 0 )); then
  echo "Live sensor verification failed with $failures missing stream(s)." >&2
  exit 1
fi
echo "Live sensor verification passed for profile: $profile"
