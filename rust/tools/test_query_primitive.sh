#!/bin/bash
# Test script for querying primitive service

# https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS2 setup
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

# Find and source robonix-sdk setup
ROBONIX_SDK_DIR=""
if [ -n "$ROBONIX_SDK_PATH" ] && [ -f "$ROBONIX_SDK_PATH/install/setup.bash" ]; then
    ROBONIX_SDK_DIR="$ROBONIX_SDK_PATH"
else
    # Search upward from script directory
    SEARCH_DIR="$SCRIPT_DIR"
    while [ "$SEARCH_DIR" != "/" ]; do
        if [ -d "$SEARCH_DIR/robonix-sdk" ] && [ -f "$SEARCH_DIR/robonix-sdk/install/setup.bash" ]; then
            ROBONIX_SDK_DIR="$SEARCH_DIR/robonix-sdk"
            break
        fi
        SEARCH_DIR="$(dirname "$SEARCH_DIR")"
    done
fi

if [ -z "$ROBONIX_SDK_DIR" ]; then
    echo "ERROR: robonix-sdk not found"
    echo "Please set ROBONIX_SDK_PATH or ensure robonix-sdk is in parent directory"
    exit 1
fi

echo "Sourcing robonix-sdk from: $ROBONIX_SDK_DIR"
source "$ROBONIX_SDK_DIR/install/setup.bash"

# Set DDS implementation to FastDDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "Using FastDDS (rmw_fastrtps_cpp) for ROS2 CLI tools"

# Run test script
echo "Running test script..."
python3 "$SCRIPT_DIR/test_query_primitive.py"

