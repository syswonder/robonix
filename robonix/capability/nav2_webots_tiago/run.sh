source /opt/ros/humble/setup.bash

ros2 launch nav2_bringup bringup_launch.py map:=config/my_map.yml use_sim_time:=true params_file:=config/nav2_params.yml
