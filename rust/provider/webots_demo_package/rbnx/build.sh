#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build Webots Demo Package Script
#
# Build script for webots_demo_package
# This script is executed by 'rbnx deploy build' command
# It should compile, install dependencies, or perform any necessary build steps

set -e  # Exit on error

echo "Building webots_demo_package..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PACKAGE_DIR"

# Source ROS2 setup if available
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Fix conda environment issues
if [ -n "$CONDA_PREFIX" ]; then
    export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "$CONDA_PREFIX/lib" | tr '\n' ':' | sed 's/:$//')
    unset CONDA_DEFAULT_ENV
    unset CONDA_PREFIX
    unset CONDA_PROMPT_MODIFIER
    unset CONDA_PYTHON_EXE
    unset CONDA_SHLVL
    export PATH=$(echo $PATH | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
fi

# Use system Python explicitly
export PYTHON3_EXECUTABLE=/usr/bin/python3
export PYTHON_EXECUTABLE=/usr/bin/python3

# Install required ROS2 dependencies
echo "Installing ROS2 dependencies..."
DEPS=(
    "ros-humble-controller-interface"
    "ros-humble-hardware-interface"
    "ros-humble-realtime-tools"
    "ros-humble-rclcpp-lifecycle"
    "ros-humble-nav-msgs"
    "ros-humble-rcpputils"
    "ros-humble-tf2"
    "ros-humble-tf2-msgs"
    "ros-humble-pluginlib"
    "ros-humble-controller-manager"
    "ros-humble-joint-state-broadcaster"
    "ros-humble-robot-state-publisher"
    "ros-humble-tf2-ros"
    "ros-humble-diff-drive-controller"
    "ros-humble-webots-ros2-driver"
    "ros-humble-webots-ros2-control"
    "ros-humble-webots-ros2"
    "ros-humble-nav2-bringup"
    "ros-humble-slam-toolbox"
    "ros-humble-rviz2"
)

# Check and install missing dependencies
MISSING_DEPS=()
for dep in "${DEPS[@]}"; do
    if ! dpkg -l | grep -q "^ii.*${dep}"; then
        MISSING_DEPS+=("${dep}")
    fi
done

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo "Installing missing dependencies: ${MISSING_DEPS[*]}"
    apt-get update -qq
    apt-get install -y "${MISSING_DEPS[@]}" || {
        echo "Warning: Some dependencies could not be installed. Build may fail."
    }
    # Re-source ROS2 environment after installing new packages
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
else
    echo "All required dependencies are already installed."
fi

# Clean previous build if it exists (to ensure fresh build with new dependencies)
if [ -d "build" ] || [ -d "install" ]; then
    echo "Cleaning previous build artifacts..."
    rm -rf build install log
fi

# Build packages using colcon
echo "Building webots_demo_package with colcon..."
if command -v colcon > /dev/null 2>&1; then
    colcon build \
        --cmake-args \
        -DPYTHON3_EXECUTABLE=/usr/bin/python3 \
        -DCMAKE_PREFIX_PATH=/opt/ros/humble
    echo "Package built successfully!"
else
    echo "Error: colcon not found. Please install colcon-common-extensions."
    exit 1
fi

echo "Build completed successfully!"

