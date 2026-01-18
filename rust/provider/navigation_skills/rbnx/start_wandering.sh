#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start Wandering Skill Script
#
# Start script for wandering skill

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PACKAGE_DIR"

# Source ROS2 setup if available
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source local setup if available
if [ -f "install/setup.bash" ]; then
    source install/setup.bash 2>/dev/null || true
fi

# Find and source robonix-sdk setup to make robonix_sdk ROS2 messages available
ROBONIX_SDK_DIR=""
if [ -n "$ROBONIX_SDK_PATH" ] && [ -f "$ROBONIX_SDK_PATH/install/setup.bash" ]; then
    ROBONIX_SDK_DIR="$ROBONIX_SDK_PATH"
else
    # Search upward from package directory for robonix-sdk
    SEARCH_DIR="$PACKAGE_DIR"
    while [ "$SEARCH_DIR" != "/" ]; do
        if [ -d "$SEARCH_DIR/robonix-sdk" ] && [ -f "$SEARCH_DIR/robonix-sdk/install/setup.bash" ]; then
            ROBONIX_SDK_DIR="$SEARCH_DIR/robonix-sdk"
            break
        fi
        SEARCH_DIR="$(dirname "$SEARCH_DIR")"
    done
fi

# Source robonix-sdk setup AFTER local setup to ensure robonix_sdk messages are available
if [ -n "$ROBONIX_SDK_DIR" ] && [ -f "$ROBONIX_SDK_DIR/install/setup.bash" ]; then
    if source "$ROBONIX_SDK_DIR/install/setup.bash" 2>&1; then
        echo "[INFO] Sourced robonix-sdk setup.bash, robonix_sdk messages available" >&2
    else
        echo "[WARN] Failed to source robonix-sdk setup.bash" >&2
    fi
else
    echo "[WARN] robonix-sdk not found, robonix_sdk messages may not be available" >&2
fi

# Try to find and run the executable directly
EXECUTABLE_PATH=""
if [ -f "install/navigation_skills_provider/bin/wandering_skill" ]; then
    EXECUTABLE_PATH="install/navigation_skills_provider/bin/wandering_skill"
elif command -v wandering_skill > /dev/null 2>&1; then
    EXECUTABLE_PATH="$(which wandering_skill)"
fi

if [ -n "$EXECUTABLE_PATH" ] && [ -f "$EXECUTABLE_PATH" ]; then
    echo "Starting wandering skill via executable: $EXECUTABLE_PATH"
    exec "$EXECUTABLE_PATH"
fi

# Try ros2 run as fallback
if command -v ros2 > /dev/null 2>&1; then
    echo "Warning: Direct executable not found, trying ros2 run..."
    # Note: ros2 run may fail for ament_python console_scripts, but we try anyway
    exec ros2 run navigation_skills_provider wandering_skill || true
fi

# Final fallback: Use Python module directly
SKILL_SCRIPT="$PACKAGE_DIR/navigation_skills_provider/wandering_skill.py"
if [ ! -f "$SKILL_SCRIPT" ]; then
    echo "Error: Skill script not found: $SKILL_SCRIPT"
    exit 1
fi
echo "Starting wandering skill via Python script: $SKILL_SCRIPT"
exec python3 "$SKILL_SCRIPT"
