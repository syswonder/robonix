source /opt/ros/humble/setup.bash

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=config/nav2_params.yml