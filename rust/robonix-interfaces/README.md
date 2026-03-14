# robonix-interfaces

This repository is the source of truth for Robonix public interfaces.

## Layout

- `ridl/`
  - handwritten public RIDL interfaces
  - defines `query`, `command`, `stream`, and `event`
  - canonical namespaces live under:
    - `robonix/prm/*` — primitives (camera, base, sensor, arm, gripper, etc.)
    - `robonix/system/*` — system services (skill, map, model, debug, etc.)
  - notable system contracts include service-like interfaces
  - skills use per-skill command pattern: each skill defines its own command RIDL with typed input/result (ROS msg); package-local RIDL in `ridl/` under the package generates `{package}_interfaces` and `{package}_interfaces_ros2`
- `lib/`
  - reusable ROS message packages and vendored ROS interface dependencies
  - only reusable data structures belong here
  - do not add `.srv` transport contracts here for new Robonix interfaces

## Rules

- **One RIDL file, one interface**: Each `.ridl` file defines exactly one interface. The interface name must match the file name (e.g. `depth.ridl` defines `stream depth`, `move.ridl` defines `command move`). Similar to Java: one public class per file, class name = file name.
- public interaction semantics are defined in RIDL
- reusable payload structures are defined as ROS messages under `lib/`
- new architecture does not require `robonix-sdk`

## Design discussion

- **Speaker / Mic**: `prm::speaker.play.stream`, `prm::speaker.play.file`, `prm::mic.stream`, `prm::mic.record` are not yet defined; they require `audio_msgs/msg/AudioData` or equivalent. Options: (1) add vendored audio_msgs, (2) use std_msgs/msg/String for file path and Float32 for duration where applicable, (3) define a minimal robonix_msg for PCM/audio metadata.
- **goal_status response**: `NavigationStatus` uses `string status` (e.g. success/failed/navigating). Consider an enum-style msg if tooling supports it.
- **navigate without result**: `prm::base.navigate` is command with input only; status is typically via goal_status query or event. Alternative: add optional result for immediate ack.

## Related docs

- package manifest: `../ROBONIX_PACKAGE_MANIFEST.md`
