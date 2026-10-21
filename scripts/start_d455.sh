#!/usr/bin/env bash
# Standalone D455 driver.
set -eo pipefail
d455_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${CONAVBOT_D455_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin CONAVBOT_D455_CLEAN_ENV=1 \
    bash --noprofile --norc "$d455_root/scripts/start_d455.sh" "$@"
fi
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-$d455_root/runtime_logs/camera/ros}"
mkdir -p "$ROS_LOG_DIR"
d455_mode="${1:---check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$d455_mode" in
  --check) exec ros2 launch "$d455_root/scripts/d455.launch.py" --show-args "$@" ;;
  --start) exec ros2 launch "$d455_root/scripts/d455.launch.py" "$@" ;;
  *) echo 'Usage: start_d455.sh [--check|--start] [launch arguments]' >&2; exit 2 ;;
esac
