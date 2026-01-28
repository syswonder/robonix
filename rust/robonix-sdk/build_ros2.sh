#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build ROS2 Interface Package Script
#
# Build script for robonix_sdk ROS2 interface package

set -e

echo "Building robonix_sdk ROS2 interface package..."

# Disable conda environment completely
unset CONDA_DEFAULT_ENV
unset CONDA_PREFIX
unset CONDA_PROMPT_MODIFIER
unset CONDA_PYTHON_EXE
unset CONDA_SHLVL

# Remove conda from PATH
export PATH=$(echo $PATH | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')

# Use system Python explicitly (force /usr/bin/python3)
export PYTHON3_EXECUTABLE=/usr/bin/python3

# Install required Python dependencies using system Python
echo "Installing Python dependencies..."
$PYTHON3_EXECUTABLE -m pip install "empy==3.3.4" --upgrade --ignore-installed --quiet
$PYTHON3_EXECUTABLE -m pip install lark --quiet
$PYTHON3_EXECUTABLE -m pip install numpy --quiet

# Source ROS2 environment
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Force ROS2 to use system Python (override any conda settings)
export PYTHON3_EXECUTABLE=/usr/bin/python3
export PYTHON_EXECUTABLE=/usr/bin/python3

# Clean build/install/log to avoid "existing path cannot be removed: Is a directory"
# when using --symlink-install (leftover dir conflicts with symlink creation)
if [ -d "build" ] || [ -d "install" ]; then
    echo "Cleaning previous build artifacts..."
    rm -rf build install log
fi

# Build the package (do not pass PYTHON3_EXECUTABLE; CMake finds Python itself)
echo "Building ROS2 package..."
colcon build --cmake-args -DCMAKE_PREFIX_PATH=/opt/ros/humble

echo "Build completed successfully!"