#!/bin/bash
# Build script for realsense_ros2

set -e

echo "Building realsense_ros2 packages..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "realsense_ros2 build completed successfully!"
