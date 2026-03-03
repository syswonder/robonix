#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start Base Pose AMCL Primitive Script
#
# Start script for prm::base.pose.cov (robot pose in map frame directly from AMCL)
# This script verifies that AMCL is running and the /amcl_pose topic is available.
# No converter needed - uses PoseWithCovarianceStamped directly.

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

# Wait for the AMCL pose topic to be available (up to 30 seconds)
echo "Waiting for AMCL pose topic /amcl_pose..."
TIMEOUT=30
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if ros2 topic list 2>/dev/null | grep -q "^/amcl_pose$"; then
        echo "AMCL pose topic is available!"
        exit 0
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

if [ $ELAPSED -ge $TIMEOUT ]; then
    echo "Warning: AMCL pose topic not found. Make sure AMCL is running (e.g., via nav2 launch)"
    exit 1
fi
