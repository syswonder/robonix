---
name: visual_inspection
description: Capture and understand camera images using the robot's head camera and VLM.
---

# Visual Inspection Skill

Use the robot's RGB camera to observe the environment and understand what
is visible via multimodal VLM analysis.

## Available Tools

- `camera_snapshot()` — returns **JSON** of `sensor_msgs/Image` (contract `robonix/prm/camera/snapshot`): JPEG bytes in `data` (base64 per MCP wire).
- `camera_depth_snapshot()` — same message type for depth (grayscale JPEG in `data`).

## How It Works

When you call `camera_snapshot()`, the bridge fills a codegen **`sensor_msgs_mcp.Image`** and returns `to_dict()` JSON. Integrations may forward the image to a VLM for visual understanding. Describe what you see, identify objects, read text, or answer questions about the scene.

## Example

User: "What do you see in front of the robot?"

1. Call `camera_snapshot()` — decode `data` / attach image for VLM as your stack expects.
2. Describe the scene based on visual understanding.
3. If needed, call `camera_depth_snapshot()` for distance-related cues (normalized depth as JPEG in this bridge).

## Tips

- Camera may have a short delay after the robot moves; wait briefly.
- Depth is normalized to grayscale JPEG in the example bridge — not raw depth units.

## Finding and approaching objects without Nav2

To **search** for something (e.g. a door) and **move closer** using only differential drive — not `base_navigate` — follow **`object_search_wander`**: alternate **`robot_state`** and **`camera_snapshot`** with **short** `base_cmd` motions.
