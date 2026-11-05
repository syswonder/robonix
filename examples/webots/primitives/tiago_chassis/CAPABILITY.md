---
description: Mobile differential-drive base with gRPC motion commands and ROS 2 odometry.
---

# Tiago chassis (`robonix/primitive/chassis`)

This primitive exposes low-level motion over gRPC and ROS 2. It does not
advertise an MCP `cmd` tool to Pilot. Model-generated movement uses
`robonix/service/navigation/navigate`, which plans and avoids obstacles.

For a turn, obtain a fresh map-frame robot pose, retain its position, and set
the navigation goal's orientation to the desired heading. `navigate` accepts
`goal` as a `geometry_msgs/PoseStamped`, not velocity fields. For planar yaw
`theta` in radians, orientation is `{x: 0, y: 0, z: sin(theta/2), w: cos(theta/2)}`.
Use `frame_id: map` when the deployment enables Scene navigation verification.
Observe the camera before and after movement, and verify the final position
and heading; navigation acceptance alone does not mean the turn completed.
A pose goal specifies a final heading, not a full revolution or turn direction.

For localization, use `robonix/service/map/pose` or Scene's robot context.
Chassis odometry is not a substitute for a map-frame pose.

## Interfaces for service callers

- `robonix/primitive/chassis/move`: gRPC `ExecuteMoveCommand`, with a nested
  `command` payload. Nonzero `forward_m` takes priority over `rotate_deg`,
  which takes priority over velocity fields. Positive `rotate_deg` turns
  counterclockwise. Velocity mode applies the twist for `duration_sec`, or
  the deployment default when zero, then publishes a zero-Twist stop.
- `robonix/primitive/chassis/odom`: ROS 2 odometry output.
- `robonix/primitive/chassis/twist_in`: ROS 2 velocity input.

The motion response contains `status.data`, a JSON acknowledgement. The
bounded command uses elapsed time; callers must observe the resulting pose
rather than treating the acknowledgement as measured motion accuracy.
