#!/usr/bin/env bash
# Visualization only: no hardware launch, navigation enable, or motion request.
set -eo pipefail
rviz_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${CONAVBOT_RVIZ_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin CONAVBOT_RVIZ_CLEAN_ENV=1 \
    bash --noprofile --norc "$rviz_root/scripts/start_debug_rviz.sh" "$@"
fi
rviz_mode=--start
rviz_minimal=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --check|--start) rviz_mode="$1"; shift ;;
    --minimal) rviz_minimal=true; shift ;;
    --help|-h)
      echo 'Usage: start_debug_rviz.sh [--start|--check] [--minimal]'
      echo 'Default: D455, visualization only. Ctrl+C or close RViz to exit.'
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
source /opt/ros/humble/setup.bash
source "$rviz_root/install/local_setup.bash"
export PYTHONPATH="$rviz_root/../conavgpt2_ws/development/Co-NavGPT2${PYTHONPATH:+:$PYTHONPATH}"
rviz_geometry_config="$rviz_root/../conavgpt2_ws/development/Co-NavGPT2/conavbot/config.yaml"
python3 -m conavbot.frame_tree --config "$rviz_geometry_config" --check
[[ -x /opt/ros/humble/lib/rviz2/rviz2 ]] || { echo 'RViz2 is not installed.' >&2; exit 1; }
rviz_config="$rviz_root/config/rviz/exploration_debug.rviz"
if [[ "$rviz_minimal" == true ]]; then
  rviz_config="$rviz_root/runtime_logs/visualization/minimal_debug.rviz"
fi
python3 - "$rviz_root/config/rviz/exploration_debug.rviz" "$rviz_config" "$rviz_mode" "$rviz_minimal" <<'PY'
import sys
from pathlib import Path
import yaml
source, target, mode, minimal = sys.argv[1:]
config = yaml.safe_load(Path(source).read_text())
manager = config['Visualization Manager']
if minimal == 'true':
    for display in manager['Displays']:
        if display['Class'].endswith(('/Image','/PointCloud2')):
            display['Enabled'] = False
if mode == '--start' and target != source:
    destination = Path(target)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(yaml.safe_dump(config, sort_keys=False))
print('RViz: D455, fixed_frame=camera_init')
print('Source-Agent map/markers: /conavbot')
print('Minimal map view: ' + minimal + ' (raw image/cloud displays disabled when true)')
print('No hardware or exploration process is started; no movement is requested.')
PY
if [[ "$rviz_mode" == --check ]]; then
  echo 'CHECK PASSED: RViz installed and layout readable; no GUI or device started.'
  exit 0
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" && -z "${QT_QPA_PLATFORM:-}" ]]; then
  echo 'No graphical session. Run this entry in the Orin desktop terminal.' >&2
  exit 1
fi
export ROS_LOG_DIR="$rviz_root/runtime_logs/visualization/ros"
mkdir -p "$ROS_LOG_DIR"
echo 'Paper map: white=observed non-obstacle, black=obstacle, gray=unknown (may participate in planning).'
echo 'Cyan=frontier edges, yellow=candidates, green=goal, orange=short-term goal; text shows scan progress.'
echo 'Odometry shows the FAST-LIO IMU/body reference point, not base_link.'
echo 'Use /conavbot/decision for the original Agent.'
python3 -m conavbot.frame_tree --config "$rviz_geometry_config" &
rviz_tf_pid=$!
trap 'kill -TERM "$rviz_tf_pid" 2>/dev/null || true; wait "$rviz_tf_pid" 2>/dev/null || true' EXIT
ros2 run rviz2 rviz2 -d "$rviz_config"
