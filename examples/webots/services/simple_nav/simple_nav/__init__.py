# SPDX-License-Identifier: MulanPSL-2.0
"""simple_nav — minimal goto-with-avoidance navigation service for webots tiago.

Bundles three layers:
  - atlas_bridge: registers `simple_nav` with atlas,
    declares the three navigation MCP capabilities, runs the rclpy node
    + FastMCP server.
  - nav_node: rclpy subscriptions to /odom (pose), /scanner (lidar
    avoidance), publisher to /cmd_vel. Goal state machine.
  - planner: pure pursuit toward a target pose with simple
    range-based slow/stop on obstacles in the forward arc.

No full Nav2, no A*. The goal is to demonstrate end-to-end pilot →
scene-object-pose → drive-to-pose with the new robonix dev-packaging
spec; richer planning (A* on /map, costmaps, recovery behaviours) is
a follow-up.
"""
