#!/bin/bash
# Build script for asr

set -e

echo "Building asr packages..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "asr build completed successfully!"
