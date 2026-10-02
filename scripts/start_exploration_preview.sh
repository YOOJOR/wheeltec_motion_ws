#!/usr/bin/env bash
# Shadow-only entry. Does not start robot/camera hardware or any motion client.
set -eo pipefail
exploration_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${WHEELTEC_EXPLORATION_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin WHEELTEC_EXPLORATION_CLEAN_ENV=1 \
    bash --noprofile --norc "$exploration_root/scripts/start_exploration_preview.sh" "$@"
fi
source /opt/ros/humble/setup.bash
source "$exploration_root/install/local_setup.bash"
exploration_conav="${WHEELTEC_CONAV_ROOT:-/home/wheeltec/workspace/conavgpt2_ws}"
source "$exploration_conav/scripts/env.sh"
export ROS_LOG_DIR="$exploration_root/runtime_logs/exploration/ros"
mkdir -p "$ROS_LOG_DIR"
exploration_mode="${1:---check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$exploration_mode" in
  --check)
    python -c 'from wheeltec_exploration.configuration import load_configuration; from wheeltec_exploration.upstream import load_fmm; import sys; c=load_configuration(sys.argv[1]); load_fmm(c["upstream_root"]); print("CHECK PASSED: SHADOW ONLY, default camera="+c["camera"])' \
      "$exploration_root/src/wheeltec_exploration/config/astra_preview.yaml" ;;
  --start)
    exec 9>"$exploration_root/runtime_logs/exploration/preview.lock"
    flock -n 9 || { echo 'Shadow preview already running.' >&2; exit 1; }
    exec python -m wheeltec_exploration.runtime \
      --config "$exploration_root/src/wheeltec_exploration/config/astra_preview.yaml" \
      --output "$exploration_root/runtime_logs/exploration/latest" "$@" ;;
  --demo)
    exec python -m wheeltec_exploration.offline \
      --upstream-root "$exploration_conav/upstream/Co-NavGPT2" \
      --output "$exploration_root/runtime_logs/exploration/demo" "$@" ;;
  *) echo 'Usage: start_exploration_preview.sh [--check|--start|--demo] [options]' >&2; exit 2 ;;
esac
