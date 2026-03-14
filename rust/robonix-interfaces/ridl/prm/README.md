# prm (Primitives)

RIDL under `prm/` defines primitive interfaces for embodied hardware and sensors. One namespace (e.g. `robonix/prm/camera`) can have multiple interfaces.

## Mapping: prm::domain.interface

| prm:: | RIDL namespace | Interfaces |
|-------|----------------|------------|
| prm::camera | robonix/prm/camera | depth, rgb, rgbd, ir, intrinsics |
| prm::base | robonix/prm/base | navigate, move, pose_cov, odom, stop, relocalize, goal_status, controller, battery_ok, status, joint_state |
| prm::sensor | robonix/prm/sensor | pointcloud, lidar, imu |
| prm::arm | robonix/prm/arm | move_ee, move_joint, state_joint, joint_trajectory |
| prm::gripper | robonix/prm/gripper | close, open, set_width, state_width |
| prm::force_torque | robonix/prm/force_torque | wrench |
