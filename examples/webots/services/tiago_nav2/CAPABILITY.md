# Tiago Nav2 (`robonix/service/navigation`)

Wraps Nav2's `navigate_to_pose` action. **AVOID for interactive sessions.**

## When to use

ONLY when an external planner (or a previously-saved waypoint) hands you
known map-frame `(x, y, yaw)`. If you are reasoning from a camera image
and "want the robot to go toward that doorway over there", you do NOT
have map coordinates — the LLM's guess will be wrong, the robot will
drive somewhere unexpected, and you will burn seconds waiting for Nav2
to fail.

## When NOT to use (the common case)

For "find X / explore / drive toward what I see" tasks:
- Use `camera/snapshot` to perceive.
- Use `chassis/cmd` for small movements.
- Loop: snapshot → reason → cmd → snapshot.

## Tools

### `navigate(x, y, yaw=0, frame_id="map")`
Returns `{goal_id, status, nav_action}`. `nav_action=true` means Nav2's
ActionClient is connected; `false` means we fell back to publishing on
`/goal_pose`.

### `status(goal_id)`
Read the latest status of a previously-issued goal.
States: `QUEUED → SENT → ACCEPTED → SUCCEEDED|CANCELED|ABORTED|FAILED`.

### `cancel(goal_id)`
Cancel an in-flight goal (Nav2 action only — `/goal_pose` fallback goals
can't be cancelled).
