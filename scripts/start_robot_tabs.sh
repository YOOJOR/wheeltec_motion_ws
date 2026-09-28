#!/usr/bin/env bash
# Alternative GNOME Terminal UI; reuse the existing stack environment/config.
set -eo pipefail
tabs_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${WHEELTEC_STACK_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin WHEELTEC_STACK_CLEAN_ENV=1 \
    bash --noprofile --norc "$tabs_script_dir/start_robot_tabs.sh" "$@"
fi
for tabs_arg in "$@"; do
  case "$tabs_arg" in
    --help|-h|--stop|--status)
      exec python3 "$tabs_script_dir/robot_stack/tabs.py" "$@" ;;
  esac
done
mapfile -d '' -t tabs_setups < <(python3 "$tabs_script_dir/robot_stack/tabs.py" --print-setups "$@")
[[ ${#tabs_setups[@]} -ge 2 ]] || { echo '无法读取 ROS 环境路径；检查配置。' >&2; exit 1; }
source "${tabs_setups[0]}"
for tabs_setup in "${tabs_setups[@]:1}"; do source "$tabs_setup"; done
exec python3 "$tabs_script_dir/robot_stack/tabs.py" "$@"
