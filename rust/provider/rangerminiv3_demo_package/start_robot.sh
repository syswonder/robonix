#!/bin/bash
set -e

# rbnx package build webots_demo_package
./rbnx/build.sh
source install/setup.bash

WORLD=${1:-test1.wbt}
USE_RVIZ=${2:-true}
USE_NAV=${3:-false}  # Disable nav2 by default
USE_RTABMAP=${4:-true}  # Enable rtabmap by default for RGBD SLAM

echo "Starting robot with:"
echo "  World: $WORLD"
echo "  RViz: $USE_RVIZ"
echo "  Nav2: $USE_NAV (disabled)"
echo "  RTAB-Map: $USE_RTABMAP (enabled for RGBD SLAM)"

ros2 launch ranger_mini_v3 robot_launch.py \
    world:=$WORLD \
    rviz:=$USE_RVIZ \
    nav:=$USE_NAV \
    rtabmap:=$USE_RTABMAP \
    2>&1 | grep -vE "(update_all|four_wheel_steering_controller|rviz|rgbd_odometry)" | tee /tmp/robot_launch.log