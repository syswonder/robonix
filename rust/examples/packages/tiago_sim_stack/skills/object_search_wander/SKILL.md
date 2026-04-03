---
name: object_search_wander
description: Find a visible target (e.g. door) using camera + pose and approach with base_cmd only — no Nav2 base_navigate / goals.
---

# Object search without map navigation (wander + vision)

Use this when the user **must not** use navigation stack tools (`base_navigate`, `base_nav_status`, `base_nav_cancel`), for example:

- Map frame coordinates are unknown, unreliable, or the user forbids map goals.
- You need **closed-loop** behaviour: move a little, **always** re-read pose and image.

## Tools you may use

Contract-shaped MCP tools (payloads are **robonix-codegen `*_mcp` dataclasses**; see `rust/contracts`):

| Tool | Role |
|------|------|
| `robot_state()` | Returns `prm_base/RobotState` JSON — use `base_pose` for pose in `map` (or configured frame). |
| `camera_snapshot()` | Returns `sensor_msgs/Image` JSON (JPEG in `data`, base64). **After almost every motion** — decide if the target is visible. |
| `base_cmd(cmd)` | `cmd` is **`prm_base/MoveCommand`** (linear_x/y/z, angular_x/y/z). Publish duration is **`TIAGO_BASE_CMD_DURATION_SEC`** (default 1.0), not part of the message. Prefer **small** effective motion. |
| `lidar_snapshot()` | Optional: full `sensor_msgs/LaserScan` JSON when the camera view is ambiguous. |

## Tools you must NOT use (for this task)

Do **not** call:

- `base_navigate`, `base_nav_status`, `base_nav_cancel`

If those appear in the tool list, ignore them for this workflow.

## Core loop (mandatory)

Repeat until the target is centred “enough” and a short forward move brings you near it:

1. **`robot_state()`** — record position/orientation (for your own reasoning; do not assume global plan correctness).
2. **`camera_snapshot()`** — VLM sees the JPEG; decide if the target is visible.
3. If **not visible**: **rotate in place** with small increments via `base_cmd(MoveCommand(angular_z=±0.35))` (≈20°/s for ~1 s — tune `TIAGO_BASE_CMD_DURATION_SEC` if needed), `linear_x = 0`. Then go back to step 1.  
   - Alternate scan direction if you sweep past 360° without success.
4. If **visible but off-centre**: turn toward it with **small** `angular_z`, **no** long forward move until the target is roughly centred.
5. If **visible and centred**: **short** advance — e.g. `base_cmd(MoveCommand(linear_x=0.2–0.5))` — then **immediately** repeat from step 1 (pose + image).  
   - Never chain multiple forward moves without a fresh image.
6. If **too close** / target fills frame: reduce `linear_x` or stop; use fine rotations only.

**Hard rule:** never call `base_cmd` twice in a row without **both** `robot_state()` and `camera_snapshot()` in between, even when you are “just rotating to scan”.

## Parameter discipline

- Prefer **many short** `base_cmd` calls over few aggressive ones (less drift, safer in clutter).
- After **every** motion that can change the scene, call **`robot_state` + `camera_snapshot`** before the next move.
- Do **not** try to dead-reckon multi-metre translations in map frame using only pose math unless the user explicitly allows it; still **confirm with the camera** after each segment.

## Strategy hints (doors / openings)

- Doors often appear as **vertical edges** + brighter/darker rectangle; in office scenes they are often on a **side wall** — if you only see a corner, you are likely misaligned; keep **small** rotation search.
- If the camera stays empty or only walls: combine **small in-place rotations** with **short** forward probes separated by image checks — not one long “return to (0,0)” leg.

## Relation to other skills

- For map-based Nav2 goals, use the **`navigation`** skill instead (when allowed).
- For “what do I see?” without motion planning, see **`visual_inspection`**.
