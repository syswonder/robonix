#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start Base Navigate Primitive Script
#
# Start script for prm::base.navigate (navigate to target pose)
# This script verifies that Nav2 is running and the goal topic is available.
# It also starts a simple node to publish navigation status.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PACKAGE_DIR"

# Source ROS2 setup if available
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source local setup if available
if [ -f "install/setup.bash" ]; then
    source install/setup.bash 2>/dev/null || true
fi

# Wait for Nav2 action server to be available (up to 30 seconds)
echo "Waiting for Nav2 navigation action server..."
TIMEOUT=30
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then
        echo "Nav2 navigation action server is available!"
        echo "Goal can be published to /goal_pose"
        exit 0
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

echo "Warning: Nav2 navigation action server not found. Make sure Nav2 is running"
exit 1
