#!/bin/bash
# Get absolute path to the package directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_PATH="$(cd "$SCRIPT_DIR/../provider/demo_package" && pwd)"

ros2 service call /rbnx/srv/register robonix_core/srv/Register \
"{package_name: 'demo_package', package_type: 'cap', std_name: 'cap::vision.capture_rgb', description: 'Demo RGB camera package that outputs random color images', code_path: '$PACKAGE_PATH', input_names: [], input_ros_types: [], input_channels: [], output_names: ['image'], output_ros_types: ['sensor_msgs/msg/Image'], output_channels: ['/demo_package/rgb_image'], config_services: [], config_names: []}"