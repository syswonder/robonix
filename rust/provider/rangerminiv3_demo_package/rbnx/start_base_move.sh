#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start Base Movement Primitive Script
#
# Start script for prm::base.move
# Note: The base movement is provided by four_wheel_steering_controller when webots is running.
# This script verifies the topics are available or waits for them.

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

# Wait for the cmd_vel and odom topics to be available (up to 30 seconds)
echo "Waiting for base movement topics (/cmd_vel and /odom)..."
TIMEOUT=30
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel$" && \
       ros2 topic list 2>/dev/null | grep -q "^/odom$"; then
        echo "Base movement topics are available!"
        exit 0
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

echo "Warning: Base movement topics not found. Make sure webots is running with robot_launch.py"
exit 1

