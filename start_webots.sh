source /opt/ros/humble/setup.bash

# ensure all ros2 processes are stopped
bash stop.sh

cd Robonix
python3 manager/boot.py --config ../config/include/webots.yml
