#!/bin/bash
# Get absolute path to the provider directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROVIDER_PATH="$(cd "$SCRIPT_DIR/../provider/demo_provider" && pwd)"

ros2 service call /rbnx/srv/register robonix_core/srv/Register \
"{provider_name: 'demo_provider', type: 'cap', std_name: 'cap::vision.capture_rgb', description: 'Demo RGB camera provider that outputs random color images', code_path: '$PROVIDER_PATH', input_names: [], input_ros_types: [], input_channels: [], output_names: ['image'], output_ros_types: ['sensor_msgs/msg/Image'], output_channels: ['/demo_provider/rgb_image'], config_services: [], config_names: [], dependencies: []}"