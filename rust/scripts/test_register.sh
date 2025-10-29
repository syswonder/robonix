#!/bin/bash
# Register a Camera capability to Robonix Core

source /opt/ros/humble/setup.bash

echo "Registering Camera capability..."

ros2 service call /rbnx/srv/register robonix_core/srv/Register \
'"{provider_name: \"camera1\", type: \"cap\", std_name: \"camera\", description: \"RGB camera for vision\", input_topics: [], output_topics: [\"/camera1/image_raw\", \"/camera1/camera_info\"]}"'

echo "Registration request sent."

