#!/usr/bin/env bash
# Observation only: no SCAN controller and no velocity output.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
scan_ws="$root/SCAN-Planner-ros2-community"
if [ ! -f "$scan_ws/install/setup.bash" ]; then
  echo "SCAN-Planner has not been built: $scan_ws/install/setup.bash missing" >&2
  exit 1
fi
set +u
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
source "$scan_ws/install/setup.bash"
set -u
exec ros2 launch "$root/scripts/scan_compare/compare.launch.py" "$@"
