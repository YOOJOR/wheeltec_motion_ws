#!/usr/bin/env bash
# GNOME Terminal stack entry; preserve network preferences and clear overlays.
set -eo pipefail
stack_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${CONAVBOT_STACK_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin CONAVBOT_STACK_CLEAN_ENV=1 \
    bash --noprofile --norc "$stack_script_dir/start_robot.sh" "$@"
fi
for stack_arg in "$@"; do
  case "$stack_arg" in
    --help|-h|--stop|--status)
      exec python3 "$stack_script_dir/robot_stack/tabs.py" "$@" ;;
  esac
done
mapfile -d '' -t stack_setups < <(python3 "$stack_script_dir/robot_stack/tabs.py" --print-setups "$@")
[[ ${#stack_setups[@]} -ge 2 ]] || { echo '无法读取 ROS 环境路径；检查配置。' >&2; exit 1; }
source "${stack_setups[0]}"
for stack_setup in "${stack_setups[@]:1}"; do source "$stack_setup"; done
exec python3 "$stack_script_dir/robot_stack/tabs.py" "$@"
