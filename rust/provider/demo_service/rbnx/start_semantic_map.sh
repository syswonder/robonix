#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start Semantic Map Service Script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PACKAGE_DIR"

# Source ROS2 setup
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

# Find and source robonix-sdk setup
ROBONIX_SDK_DIR=""
if [ -n "$ROBONIX_SDK_PATH" ] && [ -f "$ROBONIX_SDK_PATH/install/setup.bash" ]; then
    ROBONIX_SDK_DIR="$ROBONIX_SDK_PATH"
else
    SEARCH_DIR="$PACKAGE_DIR"
    while [ "$SEARCH_DIR" != "/" ]; do
        if [ -d "$SEARCH_DIR/robonix-sdk" ] && [ -f "$SEARCH_DIR/robonix-sdk/install/setup.bash" ]; then
            ROBONIX_SDK_DIR="$SEARCH_DIR/robonix-sdk"
            break
        fi
        SEARCH_DIR="$(dirname "$SEARCH_DIR")"
    done
fi

# Source robonix-sdk setup if found
if [ -n "$ROBONIX_SDK_DIR" ]; then
    source "$ROBONIX_SDK_DIR/install/setup.bash" 2>/dev/null || true
fi

# Source local setup if available
export COLCON_CURRENT_PREFIX="$PACKAGE_DIR"
[ -f "install/setup.bash" ] && source install/setup.bash 2>/dev/null || true

# Add source directory to PYTHONPATH
export PYTHONPATH="$PACKAGE_DIR:${PYTHONPATH}"

# Start service - directly run Python module
exec python3 -m demo_service_provider.semantic_map_service
