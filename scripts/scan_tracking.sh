#!/usr/bin/env bash
# Experimental SCAN-only local controller, using the existing Nav2 safety chain.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
scan_setup="$root/SCAN-Planner-ros2-community/install/setup.bash"
if [ ! -f "$scan_setup" ]; then
  echo 'SCAN workspace is not built.' >&2
  exit 1
fi
# Do not allow another navigation stack or an old SCAN comparison to compete.
if pgrep -u "$(id -u)" -f '[s]uper_lio_bringup_launch.py|[n]av2_controller/controller_server|[s]can_compare/compare.launch.py|[s]can_planner_node|[n]av2_collision_monitor/collision_monitor|[t]wist_mux/twist_mux' >/dev/null; then
  echo 'Stop the existing navigation and SCAN comparison processes first.' >&2
  exit 1
fi
if [ "${SLIO_NO_DRIVER:-0}" != "1" ] && pgrep -u "$(id -u)" -f '[l]ivox_lidar_node|[l]ivox_ros_driver2_node' >/dev/null; then
  echo 'A Livox driver is already running. Stop it before starting this mode to avoid port conflicts.' >&2
  exit 1
fi
config="$(mktemp /tmp/scan_nav_params.XXXXXX.yaml)"
scan_log_dir="$(mktemp -d /tmp/scan_tracking.XXXXXX)"
echo "SCAN controller and planner log: $scan_log_dir/scan.log"
scan_pid=''
nav_pid=''
cleanup() {
  # Both launches run in their own sessions. Signal their entire process groups,
  # not just the intermediary shell (which otherwise leaves respawned nodes).
  [ -z "$scan_pid" ] || kill -TERM -- "-$scan_pid" 2>/dev/null || true
  [ -z "$nav_pid" ] || kill -TERM -- "-$nav_pid" 2>/dev/null || true
  [ -z "$scan_pid" ] || wait "$scan_pid" 2>/dev/null || true
  [ -z "$nav_pid" ] || wait "$nav_pid" 2>/dev/null || true
  rm -f -- "$config"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
python3 - "$root/src/me_nav2_bringup/config/nav2_params.yaml" "$config" <<'PY'
import sys, yaml
from pathlib import Path
with open(sys.argv[1], encoding='utf-8') as f:
    params = yaml.safe_load(f)
controller = params['controller_server']['ros__parameters']
controller['FollowPath'] = {
    'plugin': 'scan_nav2_controller/ScanController',
    'max_linear': 0.4,
    'max_angular': 0.35,
    'command_timeout': 0.3,
}
params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(
    Path(sys.argv[1]).resolve().parents[1] / 'behavior_trees' / 'navigate_scan.xml')
# Reduce the inflation bands for the SCAN experiment while retaining the
# existing footprint and the independent collision monitor.
params['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['inflation_radius'] = 0.20
params['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['inflation_radius'] = 0.35
with open(sys.argv[2], 'w', encoding='utf-8') as f:
    yaml.safe_dump(params, f, allow_unicode=True, sort_keys=False)
PY
export SCAN_NAV_MODE=1 SCAN_NAV_PARAMS="$config" SLIO_NO_CLEAN=1
setsid "$root/scripts/scan_compare.sh" input_path:=/scan_control/plan > >(tee "$scan_log_dir/scan.log") 2>&1 &
scan_pid=$!
setsid "$root/scripts/super_lio_nav.sh" tracking "${1:-true}" &
nav_pid=$!
wait -n "$scan_pid" "$nav_pid"
