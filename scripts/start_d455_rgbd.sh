#!/usr/bin/env bash
# D455 driver + own RGB-D adapter only; no robot or navigation launch.
set -eo pipefail
rgbd_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${WHEELTEC_RGBD_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin WHEELTEC_RGBD_CLEAN_ENV=1 \
    bash --noprofile --norc "$rgbd_root/scripts/start_d455_rgbd.sh" "$@"
fi
source /opt/ros/humble/setup.bash
source "$rgbd_root/install/local_setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$rgbd_root/runtime_logs/camera/ros}"
mkdir -p "$ROS_LOG_DIR"
rgbd_mode="${1:---check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$rgbd_mode" in
  --check) exec ros2 launch wheeltec_rgbd d455_rgbd.launch.py --show-args "$@" ;;
  --start) exec ros2 launch wheeltec_rgbd d455_rgbd.launch.py "$@" ;;
  *) echo 'Usage: start_d455_rgbd.sh [--check|--start] [launch arguments]' >&2; exit 2 ;;
esac
