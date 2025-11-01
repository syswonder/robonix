#!/bin/bash
# Build script for robonix_core ROS2 interface package
# This builds only the ROS2 service definitions, not the Rust code

set -e

echo "Building robonix_core ROS2 interface package..."

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the package using colcon (current directory only)
colcon build --symlink-install --paths .

echo "Build completed successfully!"
echo ""
echo "To use the service definitions, run:"
echo "  source install/setup.sh"
echo ""
echo "Then you can use ros2 service call with robonix_core/srv/*"

