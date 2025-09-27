#!/bin/bash
# Build script for tts_output

set -e

echo "Building tts_output packages..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "tts_output build completed successfully!"
