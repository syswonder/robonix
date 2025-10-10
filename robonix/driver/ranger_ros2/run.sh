source /opt/ros/humble/setup.bash
source install/setup.sh 
ros2 launch ranger_bringup ranger_mini_v2.launch.xml publish_odom_tf:=true port_name:=can2