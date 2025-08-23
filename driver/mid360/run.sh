#! /bin/bash

# setup for ros2
source install/setup.bash

# setup for lib
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

# run
ros2 launch livox_ros_driver2 msg_MID360_launch.py