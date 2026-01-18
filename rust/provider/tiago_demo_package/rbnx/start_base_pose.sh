#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start Base Pose Primitive Script
#
# Start script for prm::base.pose (robot pose in map frame using AMCL)
# This script verifies that AMCL is running and the pose topic is available.

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
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

if [ $ELAPSED -ge $TIMEOUT ]; then
    echo "Warning: AMCL pose topic not found. Make sure AMCL is running (e.g., via nav2 launch)"
    exit 1
fi

# Start pose converter node to convert PoseWithCovarianceStamped to PoseStamped
echo "Starting pose converter node..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/pose_converter.py" &
CONVERTER_PID=$!
echo $CONVERTER_PID > /tmp/pose_converter.pid

# Wait for converted pose topic to be available
echo "Waiting for converted pose topic /robot_pose..."
ELAPSED=0
while [ $ELAPSED -lt 10 ]; do
    if ros2 topic list 2>/dev/null | grep -q "^/robot_pose$"; then
        echo "Pose converter is running and /robot_pose topic is available!"
        exit 0
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

echo "Warning: Converted pose topic /robot_pose not found"
exit 1
