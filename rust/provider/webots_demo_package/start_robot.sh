#!/bin/bash
set -e

# rbnx package build webots_demo_package
./rbnx/build.sh
source install/setup.bash
ros2 launch ranger_mini_v3 robot_launch.py 2>&1 | grep -vE "(update_all|four_wheel_steering_controller)"