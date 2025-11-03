#!/bin/bash
# Build script for robonix_core ROS2 interface package
# This builds only the ROS2 service definitions, not the Rust code

set -e

echo "Building robonix_core ROS2 interface package..."

# Fix dependencies compatibility issues with ROS2 Humble
# ROS2 Humble requires empy==3.3.4, but newer versions (4.x) are incompatible
echo "Checking and fixing dependencies..."
pip install --force-reinstall "empy==3.3.4" --quiet
# Install lark parser required by rosidl_parser
pip install lark --quiet

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

