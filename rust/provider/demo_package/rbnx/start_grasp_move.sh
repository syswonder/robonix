#!/bin/bash
# Start script for cap::grasp.move

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PACKAGE_DIR"

# Source ROS2 setup if available
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Fix library path issues with conda environment
if [ -n "$CONDA_PREFIX" ]; then
    export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "$CONDA_PREFIX/lib" | tr '\n' ':' | sed 's/:$//')
fi

# Check if package needs to be built
if [ -f "package.xml" ] && [ ! -d "install" ]; then
    echo "Package not built, building now..."
    if command -v colcon > /dev/null 2>&1; then
        colcon build --packages-select demo_rgb_provider 2>&1 | tail -20
        if [ $? -ne 0 ]; then
            echo "Build failed, continuing anyway..."
        fi
    else
        echo "Warning: colcon not found, cannot build package"
    fi
fi

# Setup Python path - add both source and install directories
SYSTEM_PYTHON="/usr/bin/python3"
if [ -f "$SYSTEM_PYTHON" ]; then
    # Add source directory to PYTHONPATH (for development)
    if [ -d "$PACKAGE_DIR/demo_rgb_provider" ]; then
        export PYTHONPATH="$PACKAGE_DIR:${PYTHONPATH}"
    fi
    # Add install directory to PYTHONPATH (for installed package)
    if [ -d "$PACKAGE_DIR/install/demo_rgb_provider/lib/python3.10/site-packages" ]; then
        export PYTHONPATH="$PACKAGE_DIR/install/demo_rgb_provider/lib/python3.10/site-packages:${PYTHONPATH}"
    fi
    PYTHON_CMD="$SYSTEM_PYTHON"
    LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
else
    PYTHON_CMD="python3"
fi

# Set COLCON_CURRENT_PREFIX to current package directory to fix setup script path issues
export COLCON_CURRENT_PREFIX="$PACKAGE_DIR"

# Source the local setup if available (but don't fail if it doesn't exist)
if [ -f "install/setup.bash" ]; then
    # Suppress the error about build time path
    source install/setup.bash 2>/dev/null || true
elif [ -f "install/setup.sh" ]; then
    source install/setup.sh 2>/dev/null || true
fi

# Start grasp move (capability: cap::grasp.move)
# Try ros2 run first, fallback to Python module if not available
if command -v ros2 > /dev/null 2>&1 && ros2 pkg list 2>/dev/null | grep -q "^demo_rgb_provider$"; then
    exec ros2 run demo_rgb_provider grasp_move
else
    # Use Python module directly
    exec $PYTHON_CMD -m demo_rgb_provider.grasp_move
fi

