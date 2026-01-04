bash init.sh
source /opt/ros/humble/setup.sh

# ensure all ros2 processes are stopped
bash stop.sh

cd robonix
python3 manager/boot.py --config ../config/include/ranger_test.yml
