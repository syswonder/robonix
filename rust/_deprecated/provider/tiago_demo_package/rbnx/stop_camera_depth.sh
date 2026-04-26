#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Stop Depth Camera Primitive Script
#
# Stop script for primitive::camera.depth (depth camera)
# Note: The depth camera is provided by webots_ros2_driver, so this is a no-op.
# The topic will stop when webots is stopped.

set -e

echo "Depth camera primitive stop requested (no-op: camera is managed by webots_ros2_driver)"
exit 0
