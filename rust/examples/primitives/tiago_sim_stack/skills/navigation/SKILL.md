---
name: navigation
description: Navigate the Tiago robot to a target position in the map using Nav2.
---

# Navigation Skill

Send the robot to any (x, y) coordinate on the map with a specified heading.

## Available Tools

- `navigate_to(x, y, yaw, frame_id)` — queue a navigation goal
- `send_nav_goal(x, y, yaw, frame_id)` — alias for `navigate_to`
- `get_nav_status(goal_id)` — check goal progress (QUEUED → ACCEPTED → SUCCEEDED / ABORTED)
- `cancel_nav_goal(goal_id)` — cancel an in-progress navigation

## Typical Workflow

1. Call `navigate_to(x=2.0, y=1.5, yaw=0.0)` — returns a `goal_id`
2. Poll `get_nav_status(goal_id)` until status is `SUCCEEDED` or `ABORTED`
3. If stuck, call `cancel_nav_goal(goal_id)` and try an alternative route

## Coordinate System

- Frame: `map` (default)
- Origin: bottom-left of the loaded map
- Yaw: radians, 0 = facing +x, counter-clockwise positive

## Known Limitations

- Navigation requires AMCL to be localized (check `get_robot_pose` first)
- The robot may fail to reach goals near obstacles; increase clearance

## When the user forbids Nav2 tools

If the user says **not** to use navigation / map goals (no `navigate_to`, no goal polling), use the **`object_search_wander`** skill instead: closed-loop `get_robot_pose` + `get_camera_image` + short `move_base` steps.
