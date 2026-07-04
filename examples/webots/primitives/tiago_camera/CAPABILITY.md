---
description: Head-mounted RGB + depth camera — primary perception sensor; snapshot the scene.
---

# Tiago camera (`robonix/primitive/camera`)

Head-mounted RGB + depth camera. PRIMARY perception tool — call freely.

## Tools

### `snapshot` — `robonix/primitive/camera/snapshot`
- input: none
- returns: `sensor_msgs/Image` JSON. `data` is base64-encoded JPEG bytes
  (decode if you need to feed bytes back to a vision tool; for VLM-style
  reasoning you usually just inspect the JPEG directly via the host).
- frame_id: `head_front_camera_rgb_optical_frame` (override via env
  `TIAGO_RGB_FRAME_ID`).

### `depth_snapshot` — `robonix/primitive/camera/depth_snapshot`
- input: none
- returns: same shape, but the depth map normalized to a grayscale JPEG
  (closer = darker by convention). Use to gauge stand-off distance / find
  open space.

## Reasoning loop

For "find X" tasks: snapshot → describe what you see → if you don't see X,
issue a small chassis/cmd nudge → snapshot again. Don't try to navigate by
absolute coordinates when you don't have a map.
