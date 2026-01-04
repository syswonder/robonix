#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start Back Depth Camera Primitive Script
#
# Start script for prm::camera.depth (back depth camera)
# Note: The depth camera is provided by webots_ros2_driver when webots is running.
# This script verifies the topic is available or waits for it.

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

# Wait for the depth camera topic to be available (up to 30 seconds)
echo "Waiting for back depth camera topic /head_back_camera/depth_registered/image_raw..."
TIMEOUT=30
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if ros2 topic list 2>/dev/null | grep -q "/head_back_camera/depth_registered/image_raw"; then
        echo "Back depth camera topic is available!"
        exit 0
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

echo "Warning: Back depth camera topic not found. Make sure webots is running with robot_launch.py"
exit 1

