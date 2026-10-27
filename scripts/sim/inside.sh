#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
if [[ -f /sim/runtime/install/setup.bash ]]; then
  source /sim/runtime/install/setup.bash
fi
export PYTHONPATH="/sim/agent/development/Co-NavGPT2${PYTHONPATH:+:$PYTHONPATH}"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/sim/runtime/install/conavbot_sim/lib${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1 YOLO_AUTOINSTALL=false
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/sim/ws/scripts/sim/fastdds.xml
export ROS_LOG_DIR=/sim/runtime/ros
export MPLCONFIGDIR=/sim/runtime/matplotlib
export NUMBA_CACHE_DIR=/sim/runtime/numba
export YOLO_CONFIG_DIR=/sim/runtime/ultralytics
export HF_HOME=/sim/runtime/huggingface
export TORCH_HOME=/sim/runtime/torch
cd /sim/agent/development/Co-NavGPT2
exec "$@"
