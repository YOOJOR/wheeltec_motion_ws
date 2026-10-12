#!/usr/bin/env bash
# Visualization only: no hardware launch, navigation enable, or motion request.
set -eo pipefail
rviz_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${WHEELTEC_RVIZ_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin WHEELTEC_RVIZ_CLEAN_ENV=1 \
    bash --noprofile --norc "$rviz_root/scripts/start_debug_rviz.sh" "$@"
fi
rviz_mode=--start
rviz_camera=astra_s
while [[ $# -gt 0 ]]; do
  case "$1" in
    --check|--start) rviz_mode="$1"; shift ;;
    --camera)
      [[ $# -ge 2 ]] || { echo 'Missing --camera value.' >&2; exit 2; }
      rviz_camera="$2"; shift 2 ;;
    --help|-h)
      echo 'Usage: start_debug_rviz.sh [--start|--check] [--camera astra_s|d455]'
      echo 'Default: Astra, visualization only. Ctrl+C or close RViz to exit.'
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
case "$rviz_camera" in astra_s|d455) ;; *) echo 'Camera must be astra_s or d455.' >&2; exit 2 ;; esac
source /opt/ros/humble/setup.bash
[[ -x /opt/ros/humble/lib/rviz2/rviz2 ]] || { echo 'RViz2 is not installed.' >&2; exit 1; }
rviz_config="$rviz_root/config/rviz/exploration_debug.rviz"
# D455 changes image inputs only; never substitutes unverified camera TF.
if [[ "$rviz_camera" == d455 ]]; then
  rviz_config="$rviz_root/runtime_logs/visualization/d455_debug.rviz"
fi
python3 - "$rviz_root/config/rviz/exploration_debug.rviz" "$rviz_config" "$rviz_camera" "$rviz_mode" <<'PY'
import sys
from pathlib import Path
import yaml
source, target, camera, mode = sys.argv[1:]
config = yaml.safe_load(Path(source).read_text())
manager = config['Visualization Manager']
assert manager['Global Options']['Fixed Frame'] == 'camera_init'
assert not any(t['Class'].endswith(('SetGoal', 'SetInitialPose', 'PublishPoint'))
               for t in manager['Tools'])
if camera == 'd455':
    topics = {
        'Camera RGB (raw)': '/wheeltec_camera/d455/raw/wheeltec_d455/color/image_raw',
        'Camera depth (raw)': '/wheeltec_camera/d455/raw/wheeltec_d455/aligned_depth_to_color/image_raw',
    }
    for display in manager['Displays']:
        if display['Name'] in topics:
            display['Topic']['Value'] = topics[display['Name']]
        if display['Name'] == 'Camera depth (raw)':
            display['Max Value'] = 6000
    if mode == '--start':
        destination = Path(target)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(yaml.safe_dump(config, sort_keys=False))
print('RViz visualization only: camera=' + camera + ', fixed_frame=camera_init')
print('Map: /wheeltec_exploration/map; goals: /goal and /short_term_goal under /wheeltec_exploration')
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
echo 'Odometry shows the FAST-LIO IMU/body reference point, not wheeltec_base.'
echo 'Goal arrows are position markers, not commanded headings; check /wheeltec_exploration/decision for validity.'
exec ros2 run rviz2 rviz2 -d "$rviz_config"
