---
name: object_search_wander
description: Find a visible target (e.g. door) using camera + pose and approach with move_base only — no Nav2 navigate_to / goals.
---

# Object search without map navigation (wander + vision)

Use this when the user **must not** use navigation stack tools (`navigate_to`, `send_nav_goal`, `get_nav_status`, `cancel_nav_goal`), for example:

- Map frame coordinates are unknown, unreliable, or the user forbids map goals.
- You need **closed-loop** behaviour: move a little, **always** re-read pose and image.

## Tools you may use

| Tool | Role |
|------|------|
| `get_robot_pose()` | Pose in `map` (or configured frame): use to avoid long blind legs and to log where you have been. |
| `get_camera_image()` | **After almost every motion** — decide if the target (door, etc.) is visible and where it sits in the frame (left / center / right). |
| `move_base(linear_x, angular_z, duration)` | Short timed cmd_vel: prefer **small** steps (see below). |
| `get_lidar_scan()` | Optional: gaps / openings when the camera view is ambiguous. |

## Tools you must NOT use (for this task)

Do **not** call:

- `navigate_to`, `send_nav_goal`, `get_nav_status`, `cancel_nav_goal`

If those appear in the tool list, ignore them for this workflow.

## Core loop (mandatory)

Repeat until the target is centred “enough” and a short forward move brings you near it:

1. **`get_robot_pose()`** — record position/orientation (for your own reasoning; do not assume global plan correctness).
2. **`get_camera_image()`** — VLM sees the JPEG; decide if the target is visible.
3. If **not visible**: **rotate in place** with small increments, e.g. `angular_z = ±0.35` rad (≈20°), `duration = 0.8–1.2` s, `linear_x = 0`. Then go back to step 1.  
   - Alternate scan direction if you sweep past 360° without success.
4. If **visible but off-centre**: turn toward it with **small** `angular_z` (same order as above), **no** long forward move until the target is roughly centred.
5. If **visible and centred**: **short** advance — e.g. `linear_x = 0.2–0.5`, `angular_z = 0`, `duration = 0.8–1.5` s — then **immediately** repeat from step 1 (pose + image).  
   - Never chain multiple forward moves without a fresh image.
6. If **too close** / target fills frame: reduce `linear_x` or stop; use fine rotations only.

**Hard rule:** never call `move_base(...)` twice in a row. Every `move_base` must be followed by **both**
`get_robot_pose()` and `get_camera_image()` before the next motion command, even when you are “just rotating to scan”.

## Parameter discipline

- Prefer **many short** `move_base` calls over few aggressive ones (less drift, safer in clutter).
- After **every** motion that can change the scene, call **`get_robot_pose` + `get_camera_image`** before the next move.
- Do **not** try to dead-reckon multi-metre translations in map frame using only pose math unless the user explicitly allows it; still **confirm with the camera** after each segment.

## Strategy hints (doors / openings)

- Doors often appear as **vertical edges** + brighter/darker rectangle; in office scenes they are often on a **side wall** — if you only see a corner, you are likely misaligned; keep **small** rotation search.
- If the camera stays empty or only walls: combine **small in-place rotations** with **short** forward probes (≤1 m equivalent at low speed) separated by image checks — not one long “return to (0,0)” leg.

## Relation to other skills

- For map-based Nav2 goals, use the **`navigation`** skill instead (when allowed).
- For “what do I see?” without motion planning, see **`visual_inspection`**.
