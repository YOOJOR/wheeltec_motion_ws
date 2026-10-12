#!/usr/bin/env bash
# Main source-Agent entry. Hardware startup is separate; motion is explicit.
set -eo pipefail
conav_motion_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${WHEELTEC_CONAV_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin WHEELTEC_CONAV_CLEAN_ENV=1 \
    bash --noprofile --norc "$conav_motion_root/scripts/start_conav_exploration.sh" "$@"
fi
source /opt/ros/humble/setup.bash
source "$conav_motion_root/install/local_setup.bash"
conav_deployment="${WHEELTEC_CONAV_ROOT:-/home/wheeltec/workspace/conavgpt2_ws}"
source "$conav_deployment/scripts/env.sh"
# Bound native CPU pools in this process tree; leave sensor/ROS processes alone.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
conav_dds_profile="$(python - "$conav_motion_root/src/wheeltec_rgbd/config/fastdds_camera.xml" <<'PY'
import sys
from wheeltec_rgbd.transport import camera_transport_environment
print(camera_transport_environment(sys.argv[1]).get('FASTRTPS_DEFAULT_PROFILES_FILE',''))
PY
)"
if [[ -n "$conav_dds_profile" ]]; then export FASTRTPS_DEFAULT_PROFILES_FILE="$conav_dds_profile"; fi
cd "$conav_deployment/development/Co-NavGPT2"
export ROS_LOG_DIR="$conav_motion_root/runtime_logs/conav/ros"
mkdir -p "$ROS_LOG_DIR"
conav_mode="${1:---check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$conav_mode" in
  --check)
    exec python -m wheeltec.check_agent --output "$conav_deployment/runtime_logs/source-agent/check" "$@" ;;
  --start)
    exec 9>"$conav_motion_root/runtime_logs/conav/agent.lock"
    flock -n 9 || { echo 'Source Agent already running.' >&2; exit 1; }
    exec python -m wheeltec.runtime \
      --config "$conav_deployment/development/Co-NavGPT2/wheeltec/config.yaml" \
      --output "$conav_motion_root/runtime_logs/conav/latest" "$@" ;;
  *) echo 'Usage: start_conav_exploration.sh [--check|--start] [options]' >&2; exit 2 ;;
esac
