#!/bin/bash
set -e

echo "[rtabmap-container] sourcing ROS2 Humble…"
source /opt/ros/humble/setup.bash

# Source rtabmap workspace if present (for introlab3it image)
if [ -f /opt/ros/humble/local_setup.bash ]; then
    source /opt/ros/humble/local_setup.bash
fi

echo "[rtabmap-container] launching RTAB-Map…"
ros2 launch /app/rtabmap.launch.py &
RTABMAP_PID=$!
sleep 3

echo "[rtabmap-container] starting bridge…"
exec python3 /app/rtabmap_bridge.py
