#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Stop Grasp Move Capability Script
#
# Stop script for cap::grasp.move
# This script is called to stop the grasp move process
# CLI will also manage the process by PID, but this script can be used
# for additional cleanup if needed

# Kill by process name as fallback
pkill -f "grasp_move" || true

# Clean up any temporary files
rm -f /tmp/demo_grasp_cap.log

exit 0

