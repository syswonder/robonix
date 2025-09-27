# git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2

# make sure all submodules are initialized
git submodule update --init --recursive

cd src/livox_ros_driver2
# git apply ../livox_ros_driver2.patch # not need to patch now, the patched code is already at submodule repo

# build.sh
source /opt/ros/humble/setup.sh
./build.sh humble