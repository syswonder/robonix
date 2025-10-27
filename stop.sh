#!/bin/bash

# Find ROS processes
PIDS=$(ps aux | grep 'ros' | grep -v grep | grep -v ros2cli | awk '{print $2}')

# Check if any processes were found
if [ -n "$PIDS" ]; then
    echo "Found ROS processes: $PIDS"
    echo "$PIDS" | xargs sudo kill -9
    echo "ROS processes killed"
else
    echo "No ROS processes found"
fi