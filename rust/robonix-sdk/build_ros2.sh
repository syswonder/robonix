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

# compile check robonixpy python package
echo "Compiling robonixpy python package..."
$PYTHON3_EXECUTABLE -m compileall robonixpy

# Build the package with explicit Python path and disable Python generator if needed
echo "Building ROS2 package..."
colcon build --cmake-args \
  -DPYTHON3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_PREFIX_PATH=/opt/ros/humble

# # Install robonixpy to system site-packages for easy import
# echo "Installing robonixpy to system site-packages..."
# # Use the same Python executable that was used for building
# INSTALL_DIR=$($PYTHON3_EXECUTABLE -c "import site; print(site.getsitepackages()[0])")
# ROBONIXPY_SOURCE="install/robonix_sdk/local/lib/python3.10/dist-packages/robonixpy"
# ROBONIXPY_TARGET="$INSTALL_DIR/robonixpy"

# if [ -d "$ROBONIXPY_SOURCE" ]; then
#     # Remove old installation if exists
#     if [ -d "$ROBONIXPY_TARGET" ] || [ -L "$ROBONIXPY_TARGET" ]; then
#         echo "Removing old robonixpy installation..."
#         rm -rf "$ROBONIXPY_TARGET"
#     fi
    
#     # Copy or symlink to system site-packages
#     echo "Installing robonixpy to $ROBONIXPY_TARGET..."
#     cp -r "$ROBONIXPY_SOURCE" "$ROBONIXPY_TARGET"
#     echo "robonixpy installed successfully to system site-packages"
# else
#     echo "Warning: robonixpy source directory not found at $ROBONIXPY_SOURCE"
#     echo "Skipping system site-packages installation"
# fi

echo "Build completed successfully!"