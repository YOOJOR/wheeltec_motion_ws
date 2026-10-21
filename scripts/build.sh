#!/usr/bin/env bash
# Build project packages without running robot nodes or tests.
set -eo pipefail
motion_build_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${CONAVBOT_BUILD_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin CONAVBOT_BUILD_CLEAN_ENV=1 \
    bash --noprofile --norc "$motion_build_dir/scripts/build.sh" "$@"
fi
cd "$motion_build_dir"
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF "$@"
