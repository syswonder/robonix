#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start Move to Object Skill Script
#
# Start script for move_to_object skill

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

# Find and source robonix-sdk setup to make robonixpy available
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

# Source robonix-sdk setup AFTER local setup to ensure robonixpy is in PYTHONPATH
if [ -n "$ROBONIX_SDK_DIR" ] && [ -f "$ROBONIX_SDK_DIR/install/setup.bash" ]; then
    OLD_PYTHONPATH="$PYTHONPATH"
    if source "$ROBONIX_SDK_DIR/install/setup.bash" 2>&1; then
        export PYTHONPATH="$PYTHONPATH:$OLD_PYTHONPATH"
        echo "[INFO] Sourced robonix-sdk setup.bash, PYTHONPATH includes robonixpy" >&2
    else
        echo "[WARN] Failed to source robonix-sdk setup.bash" >&2
        export PYTHONPATH="$OLD_PYTHONPATH"
    fi
else
    echo "[WARN] robonix-sdk not found, robonixpy may not be available" >&2
fi

# Try ros2 run first, fallback to Python module if not available
if command -v ros2 > /dev/null 2>&1 && ros2 pkg list 2>/dev/null | grep -q "^navigation_skills_provider$"; then
    echo "Starting move_to_object skill via ros2 run..."
    exec ros2 run navigation_skills_provider move_to_object_skill
else
    # Use Python module directly
    SKILL_SCRIPT="$PACKAGE_DIR/navigation_skills_provider/move_to_object_skill.py"
    if [ ! -f "$SKILL_SCRIPT" ]; then
        echo "Error: Skill script not found: $SKILL_SCRIPT"
        exit 1
    fi
    echo "Starting move_to_object skill via Python script..."
    exec python3 "$SKILL_SCRIPT"
fi
