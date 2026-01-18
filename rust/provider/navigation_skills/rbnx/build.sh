#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build Navigation Skills Package Script
#
# Build script for navigation_skills_provider package

set -e  # Exit on error

echo "Building navigation_skills_provider package..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PACKAGE_DIR"

# Source ROS2 setup if available
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS2 Humble setup.bash not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# Find robonix-sdk directory (robonixpy is provided by robonix-sdk, not pip)
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

if [ -n "$ROBONIX_SDK_DIR" ] && [ -f "$ROBONIX_SDK_DIR/install/setup.bash" ]; then
    echo "Found robonix-sdk at: $ROBONIX_SDK_DIR"
    # Source robonix-sdk setup to ensure robonixpy is available
    source "$ROBONIX_SDK_DIR/install/setup.bash" 2>/dev/null || true
else
    echo "Warning: robonix-sdk not found, robonixpy may not be available at build time"
    echo "Note: robonixpy will be available at runtime if robonix-sdk setup.bash is sourced"
fi

# Clean previous build if it exists
if [ -d "build" ] || [ -d "install" ]; then
    echo "Cleaning previous build artifacts..."
    rm -rf build install log
fi

# Build package using colcon
echo "Building navigation_skills_provider with colcon..."
if ! command -v colcon > /dev/null 2>&1; then
    echo "Error: colcon not found. Please install colcon-common-extensions."
    exit 1
fi

# Build with proper environment
colcon build \
    --symlink-install \
    --packages-select navigation_skills_provider

echo "Package built successfully!"
echo "Build completed successfully!"
