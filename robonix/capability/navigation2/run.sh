source /opt/ros/humble/setup.bash

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false map:=config/map.yml params_file:=config/nav2_params.yml