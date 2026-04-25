---
name: pick
description: Pick up a named tabletop object in the ManiSkill3 simulation using a two-step workflow: inspect detections and grasp candidates first, then execute the selected grasp.
user-invocable: true
---

# Pick Skill

Use the visual inspection workflow to pick up a specified tabletop object in the
ManiSkill3 simulation environment.

## Available Tools

- `predict_grasps(text_prompt, max_grasps, score_threshold)` — detect the target
  object and predict 6-DoF grasp candidates. Returns a JSON list sorted by score.
  Each candidate has: `id`, `label`, `score`, `translation_world [3]`,
  `rotation_world [9]`, `approach_world [3]`, `width`, `backend`.

- `execute_grasp(grasp_id, lift_height, approach_steps, grasp_steps)` — execute a
  candidate from the last `predict_grasps` call. Runs the full pick sequence:
  open → approach → descend → close → lift.

- `pick_object(object_label, lift_height)` — unified shortcut. Prefer the
  two-step workflow below when operating interactively, so you can inspect the
  live detection/segmentation view and the refreshed grasp poses in Rerun before
  executing.

## Typical Workflow

### Preferred (inspect, then execute)

```
predict_grasps(text_prompt="mug", max_grasps=5)
```

After calling `predict_grasps`:
- Check Rerun `Detections` / `Segmentation` to confirm the target object label
  and mask are correct.
- Check `world/grasps/*` to inspect the refreshed grasp poses.
- Choose the desired candidate id, then execute:

```python
execute_grasp(grasp_id=0, lift_height=0.15)
```

The execute step performs the actual motion: open → approach → descend → close → lift.

### Quick (one call, less controllable)

```python
pick_object(object_label="mug", lift_height=0.15)
```

Use this only when you do not need to inspect detections and grasp candidates manually.

## Failure Policy

- Do not call `pick_object` or `execute_grasp` unless the target has been positively
  detected in the current view, or `predict_grasps` has returned at least one
  candidate.
- If `perception_detect(...)` returns an empty list twice in a row for the same
  object, stop retrying prompt variations. Report that the target is not visible
  or not detected reliably in the current frame.
- If `predict_grasps(...)` returns zero candidates, stop and ask for a better
  view or a different prompt. Do not keep retrying blind grasp attempts.
- Do not use whole-scene fallback grasps as if they were target-specific grasps.

## Grasp Selection Hints

- Default to `grasp_id=0` (highest score).
- For tall or elongated objects, prefer candidates where `approach_world[2]` is
  strongly negative (more top-down approach).
- Avoid candidates with `translation_world[2]` near zero (too close to the table
  surface — likely to hit the table on descent).
- If the first attempt fails, retry with `grasp_id=1` or increase `approach_steps`.

## Key Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `lift_height` | `0.15` | Lift height in metres after closing gripper |
| `approach_steps` | `40` | Control steps split between pre-grasp and descent |
| `grasp_steps` | `15` | Steps to hold gripper closed |
| `score_threshold` | `0.0` | Minimum candidate score to return |

## Backend

Configured by the `GRASP_BACKEND` environment variable on the grasp node:

- `open3d` (default) — YOLO bbox + SAM2 segmentation + geometric grasp
  generation. This is the preferred interactive workflow because it updates
  both segmentation and grasp poses in Rerun.
- `heuristic` — geometric fallback with limited scene understanding. Avoid this
  backend when you need reliable target selection.

## Known Limitations

- The `heuristic` backend always generates a top-down grasp; it will fail for
  objects that must be grasped from the side.
- Detection/segmentation overlays are only meaningful if the target is actually
  detected in the current camera view.
- The current demo assumes the robot/controller pairing configured in
  `env_node.py`. If you change robot or controller, revisit the action layout
  assumptions in the grasp executor.
