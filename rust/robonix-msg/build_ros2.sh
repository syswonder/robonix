#!/bin/bash
set -e

echo "Building robonix_core ROS2 interface package..."

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

# compile check robonix_core python package
echo "Compiling robonix_core python package..."
$PYTHON3_EXECUTABLE -m compileall robonix_core

# Build the package with explicit Python path and disable Python generator if needed
echo "Building ROS2 package..."
colcon build --cmake-args \
  -DPYTHON3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_PREFIX_PATH=/opt/ros/humble

# # Install robonix_core to system site-packages for easy import
# echo "Installing robonix_core to system site-packages..."
# # Use the same Python executable that was used for building
# INSTALL_DIR=$($PYTHON3_EXECUTABLE -c "import site; print(site.getsitepackages()[0])")
# ROBONIX_CORE_SOURCE="install/robonix_core/local/lib/python3.10/dist-packages/robonix_core"
# ROBONIX_CORE_TARGET="$INSTALL_DIR/robonix_core"

# if [ -d "$ROBONIX_CORE_SOURCE" ]; then
#     # Remove old installation if exists
#     if [ -d "$ROBONIX_CORE_TARGET" ] || [ -L "$ROBONIX_CORE_TARGET" ]; then
#         echo "Removing old robonix_core installation..."
#         rm -rf "$ROBONIX_CORE_TARGET"
#     fi
    
#     # Copy or symlink to system site-packages
#     echo "Installing robonix_core to $ROBONIX_CORE_TARGET..."
#     cp -r "$ROBONIX_CORE_SOURCE" "$ROBONIX_CORE_TARGET"
#     echo "robonix_core installed successfully to system site-packages"
# else
#     echo "Warning: robonix_core source directory not found at $ROBONIX_CORE_SOURCE"
#     echo "Skipping system site-packages installation"
# fi

echo "Build completed successfully!"