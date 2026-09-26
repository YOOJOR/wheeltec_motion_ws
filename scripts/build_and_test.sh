#!/usr/bin/env bash
# Build our two packages against Humble; tests never use the real robot topics.
set -eo pipefail
motion_build_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${WHEELTEC_BUILD_CLEAN_ENV:-}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH \
    -u COLCON_CURRENT_PREFIX -u AMENT_CURRENT_PREFIX \
    PATH=/usr/local/bin:/usr/bin:/bin WHEELTEC_BUILD_CLEAN_ENV=1 \
    bash --noprofile --norc "$motion_build_dir/scripts/build_and_test.sh" "$@"
fi
cd "$motion_build_dir"
source /opt/ros/humble/setup.bash
export WHEELTEC_VALIDATION_DIR="${WHEELTEC_VALIDATION_DIR:-$motion_build_dir/runtime_logs/validation/$(date +%Y%m%d-%H%M%S)-$$}"
mkdir -p "$WHEELTEC_VALIDATION_DIR"
export ROS_DOMAIN_ID="${WHEELTEC_TEST_DOMAIN_ID:-173}"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$WHEELTEC_VALIDATION_DIR/ros"
python3 --version | tee "$WHEELTEC_VALIDATION_DIR/python-version.txt"
colcon build --base-paths src --symlink-install --event-handlers console_direct+ \
  2>&1 | tee "$WHEELTEC_VALIDATION_DIR/build.log"
source install/local_setup.bash
colcon test --packages-select wheeltec_motion_control --event-handlers console_direct+ \
  --return-code-on-test-failure 2>&1 | tee "$WHEELTEC_VALIDATION_DIR/test.log"
colcon test-result --verbose 2>&1 | tee "$WHEELTEC_VALIDATION_DIR/test-result.log"
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
files = list(Path('build/wheeltec_motion_control').rglob('*.xml'))
count = sum(int(s.attrib.get('tests', 0)) for f in files
            for s in ET.parse(f).getroot().iter('testsuite'))
if count == 0:
    raise SystemExit('ERROR: no tests were discovered')
print(f'Verified nonempty test results: {count} cases')
PY
python3 -m unittest discover -s scripts/robot_stack -p 'test_*.py' -v \
  2>&1 | tee "$WHEELTEC_VALIDATION_DIR/stack-tests.log"
python3 scripts/smoke_entrypoints.py 2>&1 | tee "$WHEELTEC_VALIDATION_DIR/smoke.log"
printf 'Validation completed. Logs: %s\n' "$WHEELTEC_VALIDATION_DIR"
