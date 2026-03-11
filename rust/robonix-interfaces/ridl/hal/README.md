# HAL — Hardware abstraction (embodied)

RIDL under `hal/` describes typical embodied hardware and their abstract interfaces. One namespace (e.g. `robonix/hal/arm`) can have multiple interfaces, similar to Android HAL with many AIDL files per module.

## Layout

- **base**: platform motion, status, joint state, battery (`motion_cmd`, `status`, `joint_state`, `battery_ok`)
- **arm**: manipulator joint trajectory, gripper, joint state (`joint_trajectory`, `gripper`, `joint_state`)
- **camera**: RGB image and camera info streams (`image`, `image_info`)
- **depth**: depth / 3D perception (`point_cloud`)
- **lidar**: 2D laser scan (`scan`)
- **force_torque**: F/T sensor stream (`wrench`)
- **localization**: pose stream (`pose`)

All use standard ROS 2 message types (sensor_msgs, geometry_msgs, trajectory_msgs, etc.) where applicable.
