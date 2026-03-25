---
name: visual_inspection
description: Capture and understand camera images using the robot's head camera and VLM.
---

# Visual Inspection Skill

Use the robot's RGB camera to observe the environment and understand what
is visible via multimodal VLM analysis.

## Available Tools

- `get_camera_image()` — returns a base64-encoded JPEG from the head camera
- `get_depth_image()` — returns a depth image (normalized grayscale JPEG)

## How It Works

When you call `get_camera_image()`, the result contains `image_base64` which
is automatically forwarded to the VLM for visual understanding. You can then
describe what you see, identify objects, read text, or answer questions about
the scene.

## Example

User: "What do you see in front of the robot?"

1. Call `get_camera_image()` — image is sent to VLM automatically
2. Describe the scene based on visual understanding
3. If needed, call `get_depth_image()` for distance information

## Tips

- Camera may have a short delay after the robot moves; wait briefly
- Depth values are normalized to grayscale — closer objects appear brighter

## Finding and approaching objects without Nav2

To **search** for something (e.g. a door) and **move closer** using only differential drive — not `navigate_to` — follow **`object_search_wander`**: alternate **`get_robot_pose`** and **`get_camera_image`** with **short** `move_base` rotations and forwards.
