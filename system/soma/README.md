# Soma - raw body YAML and URDF service

Soma loads a single robot's `soma.yaml` and referenced URDF, then exposes them
to other Robonix components over gRPC. `rbnx boot` reads the same
`robonix_manifest.yaml` to decide which primitive and skill packages to run;
Soma spawns them through `rbnx start` in two stages (see
`docs/soma_two_stage_bringup.md`).

Soma v2 does not render or interpret a self-description. Pilot calls
`robonix/system/soma/get_yaml` to receive the original YAML string, and other
services call `robonix/system/soma/get_urdf` to receive the corresponding URDF
XML string.

## Config

Soma v2 uses a flat, four-key schema. Every field is required.

```yaml
atlas_endpoint: 127.0.0.1:50051   # atlas gRPC endpoint
listen:         127.0.0.1:50091   # soma's own gRPC listen address
provider_id:    soma              # atlas provider_id
robot_yaml:     ./soma.yaml       # path to the single robot's soma.yaml
```

Relative paths in `robot_yaml` are resolved from the config file's directory.
Fields can also be supplied on the CLI (`--atlas`, `--listen`, `--provider-id`,
`--robot-yaml`); CLI flags win over config-file values. `rbnx boot` uses this
CLI mode — it does not write a soma config file on disk.

The robot's `soma.yaml` and any files it references (URDF, package
`robonix_manifest.yaml` for stage 1/2 bring-up) must live somewhere Soma can
read at startup.

## Run

Direct invocation for local testing:

```bash
cargo run -p robonix-soma -- --config ./soma.local.yaml
```

Or with an inline config JSON blob (what `rbnx boot` uses):

```bash
robonix-soma \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50091 \
  --provider-id soma \
  --robot-yaml /path/to/robot/soma.yaml
```

At startup Soma parses the robot YAML, loads the referenced URDF, spawns
primitive packages via `rbnx start` (stage 1), registers itself in Atlas, and
serves gRPC on `listen`. Skill packages are held until `rbnx boot` writes
`stage2\n` into the pipe on `$ROBONIX_SOMA_STAGE_FD` (stage 2). Soma stops
every package it launched on SIGINT/SIGTERM.

For every launched package, Soma treats the deployment entry's `name` as the
provider instance id and passes it through `RBNX_INSTANCE_NAME`. It accepts
startup only after that exact id has a fresh Atlas registration; registrations
from other concurrently starting providers are ignored. Deployment instance
names must be non-empty, whitespace-normalized, and unique across primitive,
service, and skill sections. If the expected id is already live in Atlas,
startup fails rather than taking over the existing provider.

Driver omission canonically selects `robonix/lifecycle/driver`. Soma verifies
the provider's runtime declaration, delivers entry `config` through
Driver(CMD_INIT), and activates primitives (skills remain inactive until first
use). Explicit shared or namespace Driver selections remain valid and strict.
For an omitted manifest only, an old generated artifact may fall back to its
exact namespace Driver. If neither the shared nor exact legacy Driver exists,
Soma records a startup failure with rebuild/migration guidance and reaps the
package. Missing, mismatched, dual, and failed Driver declarations are fatal;
If neither the shared binding nor the exact legacy binding exists, Soma fails
the package startup and reports the rebuild/migration error.

`--log` sets Soma's scribe file level (`debug`, `info`, `warn`, or `error`);
package stdout/stderr is written through scribe under `$SCRIBE_LOG_DIR` or
`./logs`.

## gRPC API

`robonix/system/soma/get_yaml` returns the loaded raw Soma YAML:

```srv
string robot_id  # empty = default robot
---
string robot_id
string yaml_text
```

`robonix/system/soma/get_urdf` returns the loaded raw URDF XML:

```srv
string robot_id  # empty = default robot
---
string robot_id
string urdf_xml
```

An empty request `robot_id` selects `default_robot`. If no `default_robot` is
configured and exactly one robot is loaded, Soma selects that only robot.

## Soma YAML Spec

Example:

```yaml
urdf:
  path: ./rover_arm.urdf
  root_link: base_link
  model_name: rover_arm

robot:
  id: rover_arm_01
  display_name: "four-wheel rover with 6-DOF arm"
  family: mobile_manipulator
  root_part: base
  dimensions: { length_m: 0.84, width_m: 0.56, height_m: 1.20 }
  footprint:
    base_frame: base_link
    points: [[0.42, 0.28], [0.42, -0.28], [-0.42, -0.28], [-0.42, 0.28]]
  mass_kg: 38
  passable_door_width_m: 0.78
  exports:
    - provider_id: rover_nav
      capabilities:
        - { path: robonix/service/navigation/navigate, description: "Navigate to a 2D goal." }
    - provider_id: skill_explore_room
      capabilities:
        - { path: robonix/skill/explore/room, description: "Explore the current room." }
  components:
    - id: base
      type: mobile_base
      urdf_link: base_link
      exports:
        - provider_id: rover_chassis
          capabilities:
            - { path: robonix/primitive/chassis/move, description: "Command chassis motion." }
            - { path: robonix/primitive/chassis/odom, description: "Read chassis odometry." }
      components:
        - id: left_wheel
          type: wheel
          urdf_joint: wheel_left_joint
          exports: []
    - id: head_camera
      type: rgbd_camera
      urdf_link: camera_optical_frame
      exports:
        - provider_id: rover_camera
          capabilities:
            - { path: robonix/primitive/camera/snapshot, description: "Capture an RGB image." }

description:
  summary: "Rover with chassis, RGB-D camera, and exploration skill."
  can_do: ["drive", "navigate", "capture RGB-D images"]
  cannot_do: ["manipulate objects"]
  notes: ["Soma serves this YAML and the referenced URDF as raw text."]
```

### URDF

| Field | Type | Required | Description |
|---|---|---|---|
| `path` | string | yes | URDF path, relative to the Soma YAML file directory unless absolute. |
| `root_link` | string | yes | Root link of this composite URDF. |
| `model_name` | string | no | Human-readable or simulator-facing model name. |

Soma preserves the original URDF XML. `get_urdf()` returns that raw XML text.

### Robot

| Field | Type | Required | Description |
|---|---|---|---|
| `id` | string | yes | Body id. One Soma YAML file describes one body, and this is the robot's unique identifier. |
| `display_name` | string | yes | Name used in natural-language descriptions. |
| `family` | string | yes | Robot family, such as `mobile_robot`, `mobile_manipulator`, `fixed_dual_arm_desktop`, or `drone`. Custom values are allowed. |
| `root_part` | string | no | Component id that represents the root body part. |
| `dimensions` | object | yes | Overall dimensions, usually with `length_m`, `width_m`, and `height_m`. |
| `footprint` | object | no | Collision polygon consumed by `robonix/system/soma/footprint`. |
| `footprint.base_frame` | string | with footprint | Frame containing the polygon, normally `base_link`. |
| `footprint.points` | array | with footprint | At least three finite `[x, y]` metre pairs; the polygon must enclose the origin. |
| `mass_kg` | float | yes | Overall mass. |
| `passable_door_width_m` | float | no | Conservative door-width threshold. |
| `exports` | array | yes | Provider-grouped capabilities available at robot scope. |
| `components` | array | yes | Hierarchical semantic body components. |

### Exports

`exports` is a list of provider groups. Each group names one provider and the
capabilities that provider contributes at the current scope.

```yaml
exports:
  - provider_id: tiago_webots_chassis
    capabilities:
      - { path: robonix/primitive/chassis/move, description: "Command chassis motion." }
      - { path: robonix/primitive/chassis/odom, description: "Read odometry." }
  - provider_id: skill_explore_room
    capabilities:
      - { path: robonix/skill/explore/room, description: "Explore the current room." }
```

Robot-level `exports` are for whole-body service and skill providers. Component
`exports` are for providers attached to that body part, such as a chassis,
camera, lidar, audio IO, or arm provider.

| Field | Type | Required | Description |
|---|---|---|---|
| `provider_id` | string | yes | Provider id that exposes the listed capabilities. |
| `capabilities` | array | yes | Capabilities exposed by that provider. |
| `capabilities[].path` | string | yes | Capability contract path, such as `robonix/primitive/chassis/move`. |
| `capabilities[].description` | string | no | Short natural-language capability description. |

### Components

`components` is the semantic body tree for the composite URDF. It does not need
to repeat every URDF joint one by one; actual transforms still come from the
URDF link / joint tree.

```yaml
components:
  - id: base
    type: mobile_base
    urdf_link: base_link
    exports:
      - provider_id: rover_chassis
        capabilities:
          - { path: robonix/primitive/chassis/move, description: "Command chassis motion." }
    components:
      - id: left_wheel
        type: wheel
        urdf_joint: wheel_left_joint
        exports: []
```

| Field | Type | Required | Description |
|---|---|---|---|
| `id` | string | yes | Component id, unique within this Soma YAML file. |
| `type` | string | yes | Component type. Common values include `mobile_base`, `body_part`, `lidar_2d`, `rgb_camera`, `rgbd_camera`, and `audio_io`; custom values are allowed. |
| `urdf_link` | string | no | URDF link represented by this component. |
| `urdf_joint` | string | no | URDF joint represented by this component. |
| `exports` | array | yes | Provider-grouped capabilities attached to this component. Use `[]` when none are attached. |
| `components` | array | no | Child components. |
| `state` | object | no | Runtime-state calibration for this component, such as a gripper joint's open position. |

### Runtime chassis and gripper state

Soma reads live state through standard primitive capabilities; it does not
depend on a task skill. A mobile-base component should export
`robonix/primitive/chassis/odom`. Soma reports linear speed, angular speed, and
`moving` from that provider's odometry.

An arm component should export `robonix/primitive/arm/joint_states`. A gripper
below that arm may define its open-position calibration:

```yaml
- id: arm
  type: manipulator
  exports:
    - provider_id: arm_controller
      capabilities:
        - { path: robonix/primitive/arm/joint_states, description: "Read arm and gripper joints." }
  components:
    - id: gripper
      type: parallel_jaw_gripper
      state:
        joint_name: gripper
        open_position_m: 0.080
        open_tolerance_m: 0.003
      exports: []
```

`joint_name` must match the incoming JointState name. Measure
`open_position_m` from an empty, fully open gripper; choose
`open_tolerance_m` from its feedback noise. Soma reports an open gripper when
the measured position is within that tolerance and otherwise reports it as
partially closed or likely holding.

### Description

`description` is a general natural-language description block.

```yaml
description:
  summary: "Short deployment summary."
  can_do: ["can do 1", "can do 2"]
  cannot_do: ["cannot do 1"]
  notes: ["note 1"]
```

| Field | Type | Required |
|---|---|---|
| `summary` | string | no |
| `can_do` | array | no |
| `cannot_do` | array | no |
| `notes` | array | no |
