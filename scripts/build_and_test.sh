#!/usr/bin/env bash
# Run inside ROS 2 Humble, at the workspace root; does not build vendor packages.
set -eo pipefail
source /opt/ros/humble/setup.bash
mkdir -p logs
python3 --version | tee logs/python-version.log
colcon build --base-paths src --symlink-install --event-handlers console_direct+ \
  2>&1 | tee logs/colcon-build.log
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-173}"
export ROS_LOCALHOST_ONLY=1
colcon test --packages-select wheeltec_motion_control --event-handlers console_direct+ \
  --return-code-on-test-failure 2>&1 | tee logs/colcon-test.log
colcon test-result --verbose 2>&1 | tee logs/colcon-test-result.log
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
