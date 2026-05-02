#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Stop RGB Camera Primitive Script
#
# Stop script for primitive::camera.rgb (RGB camera)
# Note: The camera is provided by webots_ros2_driver, so this is a no-op.
# The topic will stop when webots is stopped.

set -e

echo "RGB camera primitive stop requested (no-op: camera is managed by webots_ros2_driver)"
exit 0
