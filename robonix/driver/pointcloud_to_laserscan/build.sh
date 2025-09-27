#!/bin/bash
# Build script for pointcloud_to_laserscan

set -e

echo "Building pointcloud_to_laserscan packages..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "pointcloud_to_laserscan build completed successfully!"
