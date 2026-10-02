#!/usr/bin/env bash
set -eo pipefail
exploration_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$exploration_root/install/local_setup.bash"
export ROS_DOMAIN_ID="${WHEELTEC_EXPLORATION_TEST_DOMAIN_ID:-182}"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$exploration_root/runtime_logs/exploration-test/ros"
mkdir -p "$ROS_LOG_DIR"
"${WHEELTEC_CONAV_ROOT:-/home/wheeltec/workspace/conavgpt2_ws}/.venv/bin/python" -m pytest \
  "$exploration_root/src/wheeltec_exploration/test" -q \
  --junitxml="$exploration_root/build/wheeltec_exploration/pytest.xml" "$@"
