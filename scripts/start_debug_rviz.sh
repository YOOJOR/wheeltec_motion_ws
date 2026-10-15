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
rviz_backend=source_agent
rviz_minimal=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --check|--start) rviz_mode="$1"; shift ;;
    --camera)
      [[ $# -ge 2 ]] || { echo 'Missing --camera value.' >&2; exit 2; }
      rviz_camera="$2"; shift 2 ;;
    --backend)
      [[ $# -ge 2 ]] || { echo 'Missing --backend value.' >&2; exit 2; }
      rviz_backend="$2"; shift 2 ;;
    --minimal) rviz_minimal=true; shift ;;
    --help|-h)
      echo 'Usage: start_debug_rviz.sh [--start|--check] [--camera astra_s|d455] [--backend source_agent|test_preview] [--minimal]'
      echo 'Default: Astra, visualization only. Ctrl+C or close RViz to exit.'
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
case "$rviz_camera" in astra_s|d455) ;; *) echo 'Camera must be astra_s or d455.' >&2; exit 2 ;; esac
case "$rviz_backend" in source_agent|test_preview) ;; *) echo 'Backend must be source_agent or test_preview.' >&2; exit 2 ;; esac
source /opt/ros/humble/setup.bash
source "$rviz_root/install/local_setup.bash"
rviz_geometry_config="$rviz_root/../conavgpt2_ws/development/Co-NavGPT2/wheeltec/config.yaml"
python3 -m wheeltec_rgbd.frame_tree --config "$rviz_geometry_config" --camera "$rviz_camera" --check
[[ -x /opt/ros/humble/lib/rviz2/rviz2 ]] || { echo 'RViz2 is not installed.' >&2; exit 1; }
rviz_config="$rviz_root/config/rviz/exploration_debug.rviz"
# D455 changes image inputs only; never substitutes unverified camera TF.
if [[ "$rviz_camera" == d455 || "$rviz_backend" == source_agent || "$rviz_minimal" == true ]]; then
  rviz_config="$rviz_root/runtime_logs/visualization/${rviz_backend}_${rviz_camera}_${rviz_minimal}_debug.rviz"
fi
python3 - "$rviz_root/config/rviz/exploration_debug.rviz" "$rviz_config" "$rviz_camera" "$rviz_mode" "$rviz_backend" "$rviz_minimal" <<'PY'
import sys
from pathlib import Path
import yaml
source, target, camera, mode, backend, minimal = sys.argv[1:]
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
if backend == 'source_agent':
    for display in manager['Displays']:
        topic = display.get('Topic',{}).get('Value','')
        if topic.startswith('/wheeltec_exploration/'):
            display['Topic']['Value'] = topic.replace('/wheeltec_exploration/','/wheeltec_conav/',1)
if minimal == 'true':
    for display in manager['Displays']:
        if display['Class'].endswith(('/Image','/PointCloud2')):
            display['Enabled'] = False
if mode == '--start' and target != source:
    destination = Path(target)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(yaml.safe_dump(config, sort_keys=False))
print('RViz visualization only: camera=' + camera + ', backend=' + backend + ', fixed_frame=camera_init')
print('Source-Agent map/markers: /wheeltec_conav; test preview: /wheeltec_exploration')
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
echo 'Use /wheeltec_conav/decision for the original Agent, /wheeltec_exploration/decision for the test preview.'
python3 -m wheeltec_rgbd.frame_tree --config "$rviz_geometry_config" --camera "$rviz_camera" &
rviz_tf_pid=$!
trap 'kill -TERM "$rviz_tf_pid" 2>/dev/null || true; wait "$rviz_tf_pid" 2>/dev/null || true' EXIT
ros2 run rviz2 rviz2 -d "$rviz_config"
