#!/bin/bash
# Test script for ping service using ros2 command line

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

# Service name
SERVICE_NAME="/rbnx/ping"
SERVICE_TYPE="robonix_sdk/srv/PingPong"

echo "=========================================="
echo "Testing Ping Service"
echo "=========================================="
echo "Using FastDDS (rmw_fastrtps_cpp) for ROS2 CLI tools"
echo "Service: $SERVICE_NAME"
echo "Type: $SERVICE_TYPE"
echo ""

# Check if service is available
echo "Checking if service is available..."
if ! ros2 service list | grep -q "^$SERVICE_NAME$"; then
    echo "ERROR: Service $SERVICE_NAME is not available"
    echo "Available services:"
    ros2 service list | grep "/rbnx" || echo "  (no /rbnx services found)"
    exit 1
fi

echo "Service is available!"
echo ""

# Get service type to verify
echo "Verifying service type..."
ACTUAL_TYPE=$(ros2 service type $SERVICE_NAME)
if [ "$ACTUAL_TYPE" != "$SERVICE_TYPE" ]; then
    echo "WARNING: Service type mismatch"
    echo "  Expected: $SERVICE_TYPE"
    echo "  Actual: $ACTUAL_TYPE"
else
    echo "Service type matches: $ACTUAL_TYPE"
fi
echo ""

# Test 1: Simple ping
echo "Test 1: Simple ping"
echo "Calling service with message='hello' and sequence=1..."
ros2 service call $SERVICE_NAME $SERVICE_TYPE "{message: 'hello', sequence: 1}"
echo ""

# Test 2: Ping with different message
echo "Test 2: Ping with different message"
echo "Calling service with message='test message' and sequence=2..."
ros2 service call $SERVICE_NAME $SERVICE_TYPE "{message: 'test message', sequence: 2}"
echo ""

# Test 3: Ping with sequence number
echo "Test 3: Ping with sequence number"
SEQUENCE=42
echo "Calling service with message='ping' and sequence=$SEQUENCE..."
ros2 service call $SERVICE_NAME $SERVICE_TYPE "{message: 'ping', sequence: $SEQUENCE}"
echo ""

echo "=========================================="
echo "All tests completed successfully!"
echo "=========================================="

