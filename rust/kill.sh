#!/usr/bin/env bash
# Force kill: nav, nav2, riv*, realsense, and all .robonix package processes

set -e

# 1. nav / nav2 / navigation2
pkill -9 -f 'nav2'                          2>/dev/null || true
pkill -9 -f 'navigation2'                    2>/dev/null || true
pkill -9 -f 'lifecycle_manager_navigation'  2>/dev/null || true
pkill -9 -f 'bt_navigator'                  2>/dev/null || true
pkill -9 -f 'controller_server'             2>/dev/null || true
pkill -9 -f 'planner_server'                 2>/dev/null || true
pkill -9 -f 'behavior_server'               2>/dev/null || true
pkill -9 -f 'waypoint_follower'             2>/dev/null || true
pkill -9 -f 'velocity_smoother'              2>/dev/null || true
pkill -9 -f 'smoother_server'               2>/dev/null || true
pkill -9 -f 'global_costmap|local_costmap'  2>/dev/null || true

# 2. rviz
pkill -9 -f 'rviz2'   2>/dev/null || true
pkill -9 -f 'rviz'    2>/dev/null || true

# 3. realsense
pkill -9 -f 'realsense'         2>/dev/null || true
pkill -9 -f 'realsense2_camera' 2>/dev/null || true
pkill -9 -f 'camera_435'       2>/dev/null || true

# 4. .robonix packages (from ps -ef: mid360_drv, ranger_drv, navigation2_base, pcld2lscan-rbnx)
pkill -9 -f '.robonix/packages/mid360_drv'       2>/dev/null || true
pkill -9 -f '.robonix/packages/ranger_drv'      2>/dev/null || true
pkill -9 -f '.robonix/packages/navigation2_base' 2>/dev/null || true
pkill -9 -f '.robonix/packages/pcld2lscan-rbnx'  2>/dev/null || true
pkill -9 -f 'livox_ros_driver2_node'             2>/dev/null || true
pkill -9 -f 'ranger_base_node'                   2>/dev/null || true
pkill -9 -f 'pointcloud_to_laserscan_node'       2>/dev/null || true
pkill -9 -f 'robot_state_publisher' 2>/dev/null || true

# 5. fastrtps shm
rm -f /dev/shm/sem.fastrtps_* /dev/shm/fastrtps_* 2>/dev/null || true

echo "kill.sh done."
