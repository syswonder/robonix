#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build Tiago Demo Package Script
#
# Build script for tiago_demo_package
# This script is executed by 'rbnx deploy build' command
# It should compile, install dependencies, or perform any necessary build steps

set -e  # Exit on error

echo "Building tiago_demo_package..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PACKAGE_DIR"

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
    "ros-humble-nav2-map-server"
    "ros-humble-nav2-amcl"
    "ros-humble-nav2-common"
    "ros-humble-nav2-controller"
    "ros-humble-nav2-core"
    "ros-humble-nav2-costmap-2d"
    "ros-humble-nav2-msgs"
    "ros-humble-nav2-navfn-planner"
    "ros-humble-nav2-planner"
    "ros-humble-nav2-behaviors"
    "ros-humble-nav2-util"
    "ros-humble-nav2-voxel-grid"
    "ros-humble-nav2-behavior-tree"
    "ros-humble-nav2-bt-navigator"
    "ros-humble-nav2-lifecycle-manager"
    "ros-humble-nav2-simple-commander"
    "ros-humble-rviz2"
    "ros-humble-image-transport"
    "ros-humble-image-transport-plugins"
    "ros-humble-sensor-msgs"
    "ros-humble-geometry-msgs"
    "ros-humble-std-msgs"
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
    # Update package lists and retry on failure
    apt-get update -qq || apt-get update
    # Try to install with retry logic
    MAX_RETRIES=3
    RETRY_COUNT=0
    while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
        if apt-get install -y "${MISSING_DEPS[@]}" 2>&1; then
            break
        fi
        RETRY_COUNT=$((RETRY_COUNT + 1))
        if [ $RETRY_COUNT -lt $MAX_RETRIES ]; then
            echo "Installation attempt $RETRY_COUNT failed, retrying..."
            sleep 2
            apt-get update -qq || apt-get update
        else
            echo "Warning: Some dependencies could not be installed after $MAX_RETRIES attempts."
            echo "Attempting to continue with build..."
        fi
    done
fi

# Verify critical dependencies are installed
CRITICAL_DEPS=("ros-humble-controller-interface" "ros-humble-hardware-interface" "ros-humble-controller-manager" "ros-humble-webots-ros2-driver")
MISSING_CRITICAL=()
for dep in "${CRITICAL_DEPS[@]}"; do
    if ! dpkg -l | grep -q "^ii.*${dep}"; then
        MISSING_CRITICAL+=("${dep}")
    fi
done

if [ ${#MISSING_CRITICAL[@]} -gt 0 ]; then
    echo "Error: Critical dependencies are missing: ${MISSING_CRITICAL[*]}"
    echo "Please install them manually: apt-get install -y ${MISSING_CRITICAL[*]}"
    exit 1
fi

# Source ROS2 environment (must be done after installing packages)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS2 Humble setup.bash not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# Clean previous build if it exists (to ensure fresh build with new dependencies)
if [ -d "build" ] || [ -d "install" ]; then
    echo "Cleaning previous build artifacts..."
    rm -rf build install log
fi

# Build packages using colcon
echo "Building tiago_demo_package with colcon..."
if ! command -v colcon > /dev/null 2>&1; then
    echo "Error: colcon not found. Please install colcon-common-extensions."
    exit 1
fi

# Ensure ROS2 environment is sourced before building
source /opt/ros/humble/setup.bash

# Build with proper environment
colcon build \
    --symlink-install \
    --cmake-args \
    -DPYTHON3_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_PREFIX_PATH=/opt/ros/humble

echo "Package built successfully!"
echo "Build completed successfully!"
