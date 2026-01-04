# webots package

```bash
rbnx package build webots_demo_package
source install/setup.bash
ros2 launch ranger_mini_v3 robot_launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```