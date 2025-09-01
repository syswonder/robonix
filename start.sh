bash init.sh

# ensure all ros2 processes are stopped
bash stop.sh

python3 manager/boot.py --config config/include/ranger_test.yml
