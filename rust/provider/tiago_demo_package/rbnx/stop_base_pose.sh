#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Stop Base Pose Primitive Script
#
# Stop script for prm::base.pose (robot pose in map frame)
# Stops the pose converter node if it's running.

set -e

if [ -f /tmp/pose_converter.pid ]; then
    PID=$(cat /tmp/pose_converter.pid)
    if ps -p $PID > /dev/null 2>&1; then
        echo "Stopping pose converter node (PID: $PID)..."
        kill $PID
        rm /tmp/pose_converter.pid
        echo "Pose converter node stopped"
    else
        echo "Pose converter node not running"
        rm /tmp/pose_converter.pid
    fi
else
    echo "Pose converter node not found (may not be running)"
fi

exit 0
