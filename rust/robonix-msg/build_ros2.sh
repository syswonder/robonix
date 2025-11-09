#!/bin/bash
set -e

echo "Building robonix_core ROS2 interface package..."

pip install "empy==3.3.4" --upgrade --ignore-installed --quiet
pip install lark --quiet

source /opt/ros/humble/setup.bash

colcon build 