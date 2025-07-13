#!/bin/bash

sudo ip link set eth0 up
sudo ip addr add 192.168.100.2/24 dev eth0

# check whether eth0 is ready
if ! ip link show eth0 | grep -q "state UP"; then
    echo "failed to bring up eth0, do you really plug in the ethernet cable? :("
    # print info of eth0
    ip link show eth0
fi

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/wheatfox/code/EmbodiedAIOS/src/piper_raspi_hc_sr04/fastrtps.xml
export ROS_LOCALHOST_ONLY=0
export ROS_IP=$(hostname -I| awk '{for(i=1;i<=NF;i++)if($i~/^192/)print $i}')

source /opt/ros/humble/setup.bash

if [ -z "$ROS_IP" ]; then
    echo "ROS_IP is not set, devices connected through ethernet will not be able to connect to this node!"
fi

echo "ROS_IP: $ROS_IP"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"

python3 raspi_hc_sr04/node.py

# create a default talker
# ros2 topic pub /chatter std_msgs/String "data: 'Hello World from Raspberry Pi :)'"
