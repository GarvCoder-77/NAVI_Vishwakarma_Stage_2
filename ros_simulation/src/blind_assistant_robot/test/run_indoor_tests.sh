#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

echo "================================================================="
echo "   BLIND ASSISTANT ROBOT — INDOOR SIMULATION VERIFICATION SUITE   "
echo "================================================================="

cd "${WS_DIR}"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || \
source /opt/ros/iron/setup.bash 2>/dev/null || \
source /opt/ros/jazzy/setup.bash 2>/dev/null || \
source /opt/ros/*/setup.bash 2>/dev/null

# Source workspace install if available
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

echo "[1/2] Building package (if inside colcon workspace)..."
if [ -d "${WS_DIR}/src" ]; then
    colcon build --packages-select blind_assistant_robot
fi

echo "[2/2] Running automated test suites..."
python3 -m pytest -v "${SCRIPT_DIR}/test_webcam_lidar_fusion.py"
python3 -m pytest -v "${SCRIPT_DIR}/test_indoor_simulation.py"

echo "================================================================="
echo "   ✓ ALL TESTS PASSED SUCCESSFULLY!                             "
echo "================================================================="
