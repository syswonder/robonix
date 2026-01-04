#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Stop Capture RGB Capability Script
#
# Stop script for cap::vision.capture_rgb
# This script is called to stop the RGB publisher process
# CLI will also manage the process by PID, but this script can be used
# for additional cleanup if needed

# Kill by process name as fallback
pkill -f "rgb_publisher" || true

# Clean up any temporary files
rm -f /tmp/demo_rgb_cap.log

exit 0

