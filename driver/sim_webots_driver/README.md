# PR2 Webots Driver

PR2 robot simulation driver for Webots with Nav2 navigation.

## Quick Start

```bash
# Install dependencies
sudo apt install \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-webots-ros2-driver \
    ros-humble-rviz2

# Build and run
cd src/swd
colcon build
source install/setup.sh
ros2 launch swd ranger_launch.py
```

## Features

- PR2 robot with 8-wheel differential drive
- Dual laser scanners (base + tilt)
- Nav2 navigation with SLAM
- RViz2 visualization

## Usage

1. **Set initial pose**: Use "2D Pose Estimate" tool in RViz2
2. **Navigate**: Click "2D Nav Goal" to set destination
3. **Monitor**: Watch laser scans, costmaps, and path planning

> **Note**: Run in GUI terminal (Webots needs display)
