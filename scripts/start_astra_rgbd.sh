#!/usr/bin/env bash
# Source only ROS, Astra's two device packages and our workspace.
set -eo pipefail
rgbd_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${WHEELTEC_RGBD_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin WHEELTEC_RGBD_CLEAN_ENV=1 \
    bash --noprofile --norc "$rgbd_root/scripts/start_astra_rgbd.sh" "$@"
fi
source /opt/ros/humble/setup.bash
rgbd_vendor="${WHEELTEC_ASTRA_INSTALL:-/home/wheeltec/wheeltec_ros2/install}"
for rgbd_pkg in astra_camera_msgs astra_camera; do
  rgbd_setup="$rgbd_vendor/$rgbd_pkg/share/$rgbd_pkg/package.bash"
  if [[ ! -f "$rgbd_setup" ]]; then
    echo "Missing device package: $rgbd_setup; set WHEELTEC_ASTRA_INSTALL" >&2
    exit 1
  fi
  source "$rgbd_setup"
done
source "$rgbd_root/install/local_setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$rgbd_root/runtime_logs/camera/ros}"
mkdir -p "$ROS_LOG_DIR"
rgbd_mode="${1:---check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$rgbd_mode" in
  --check) exec ros2 launch wheeltec_rgbd astra_rgbd.launch.py --show-args "$@" ;;
  --start) exec ros2 launch wheeltec_rgbd astra_rgbd.launch.py "$@" ;;
  *) echo 'Usage: start_astra_rgbd.sh [--check|--start] [launch arguments]' >&2; exit 2 ;;
esac
