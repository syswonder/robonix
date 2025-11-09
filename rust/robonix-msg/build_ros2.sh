#!/bin/bash
set -e

echo "Building robonix_core ROS2 interface package..."

# Install required Python dependencies
echo "Installing Python dependencies..."
pip3 install "empy==3.3.4" --upgrade --ignore-installed --quiet
pip3 install lark --quiet

# Source ROS2 environment
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Build the package
echo "Building ROS2 package..."
colcon build

echo "Build completed successfully!"