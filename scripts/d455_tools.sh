#!/usr/bin/env bash
# Human-facing librealsense tools, independent of all robot/ROS camera launches.
set -eo pipefail
source /opt/ros/humble/setup.bash
case "${1:---check}" in
  --check)
    for d455_tool in realsense-viewer rs-enumerate-devices rs-depth-quality rs-record rs-convert; do
      test -x "/opt/ros/humble/bin/$d455_tool" || { echo "Missing $d455_tool" >&2; exit 1; }
      printf '%s\n' "/opt/ros/humble/bin/$d455_tool"
    done
    test -f /opt/ros/humble/include/librealsense2/rs.hpp
    dpkg-query -W ros-humble-librealsense2 ros-humble-realsense2-camera ;;
  --viewer) exec /opt/ros/humble/bin/realsense-viewer ;;
  --enumerate) exec /opt/ros/humble/bin/rs-enumerate-devices -s ;;
  --quality) exec /opt/ros/humble/bin/rs-depth-quality ;;
  *) echo 'Usage: d455_tools.sh [--check|--viewer|--enumerate|--quality]' >&2; exit 2 ;;
esac
