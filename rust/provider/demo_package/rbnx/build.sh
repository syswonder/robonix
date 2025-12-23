#!/bin/bash
# Build script for demo_rgb_provider package
# This script is executed by 'rbnx deploy build' command
# It should compile, install dependencies, or perform any necessary build steps

set -e  # Exit on error

echo "Building demo_rgb_provider package..."

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

# Find robonix-sdk directory
ROBONIX_SDK_DIR=""
if [ -n "$ROBONIX_SDK_PATH" ] && [ -d "$ROBONIX_SDK_PATH" ]; then
    ROBONIX_SDK_DIR="$ROBONIX_SDK_PATH"
else
    # Search upward from package directory for robonix-sdk
    SEARCH_DIR="$PACKAGE_DIR"
    while [ "$SEARCH_DIR" != "/" ]; do
        if [ -d "$SEARCH_DIR/robonix-sdk" ]; then
            ROBONIX_SDK_DIR="$SEARCH_DIR/robonix-sdk"
            break
        fi
        SEARCH_DIR="$(dirname "$SEARCH_DIR")"
    done
fi

# Build demo_rgb_provider package using colcon
echo "Building demo_rgb_provider package with colcon..."
if command -v colcon > /dev/null 2>&1; then
    colcon build --packages-select demo_rgb_provider \
        --cmake-args \
        -DPYTHON3_EXECUTABLE=/usr/bin/python3 \
        -DCMAKE_PREFIX_PATH=/opt/ros/humble
    echo "Package built successfully!"
else
    echo "Error: colcon not found. Please install colcon-common-extensions."
    exit 1
fi

echo "Build completed successfully!"
