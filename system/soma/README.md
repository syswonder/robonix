# Soma - raw body YAML and URDF service

Soma loads a single robot's `soma.yaml` and referenced URDF, then exposes them
to other Robonix components over gRPC. `rbnx boot` reads the same
`robonix_manifest.yaml` to decide which primitive and skill packages to run;
Soma spawns them through `rbnx start` in two stages (see
`docs/docs/architecture/deployment-and-startup.md`).

Soma preserves the self-description as raw YAML and URDF for consumers. It
also interprets the existing `robot.components` tree when normalizing a
`robonix/primitive/health/stream` frame into `SomaHealthSnapshot`; this keeps
health component paths and kinds aligned with the robot description. A fresh
health-primitive frame takes precedence; if its TTL expires, Soma continues
publishing the ROS runtime-state fallback.

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

`robonix/system/soma/get_urdf` returns the loaded raw URDF XML and, on request,
the files referenced by URDF-local relative mesh or texture paths:

```srv
string robot_id  # empty = default robot
bool include_assets
---
string robot_id
string urdf_xml
UrdfAsset[] assets
```

Absolute browser URLs and `package://` references are not attached. Relative
resources must resolve below the directory containing the URDF. Soma indexes
these paths at startup and reads the files only for
`get_urdf(include_assets=true)`, so a missing mesh does not prevent non-rendering
deployments from starting.

For render resources that may exceed the bounded legacy response, use the
manifest and streaming capabilities instead:

- `robonix/system/soma/get_urdf_asset_manifest` returns the URDF XML, a
  content-derived resource-set ID, total bytes, and each resource's relative
  path, byte size, SHA-256 digest, and media type.
- `robonix/system/soma/stream_urdf_asset` streams one indexed resource from an
  optional byte offset. Soma bounds each response message to at most 4 MiB.

Soma hashes a model's resources on the first manifest request and caches that
immutable manifest for the process lifetime. Deployments should restart Soma
after replacing a URDF resource. New clients should use these two capabilities;
`get_urdf(include_assets=true)` remains available for older clients and small
models.

`robonix/system/soma/footprint` returns the active robot's 2D collision
polygon, base frame, inscribed radius, and circumscribed radius. Generic
services such as Scene and Navigation consume this contract instead of
carrying robot-model dimensions of their own.

`robonix/system/soma/get_health` and `robonix/system/soma/health` expose the
latest normalized hardware state. Health primitive reading names use stable
Soma paths such as `body/base/left_wheel`; optional controls append
`/driver_temp`, `/enabled`, `/communication_ok`, `/online`, or `/error`.
Top-level `HealthState` power fields are attached to the component whose
`type` is `battery`. Hardware topology comes only from `robot.components`;
Soma does not infer deployment-specific joints or devices.

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

Soma preserves the original URDF XML. `get_urdf()` returns that raw XML text
and can include its URDF-local resources for clients that need to render the
visual geometry.

#### URDF resource path convention

Robot descriptions intended for Soma and Vitals must use URDF-local relative
paths for Mesh and texture resources. The `filename` value is resolved from the
directory containing the URDF, not from the repository root, current working
directory, or `soma.yaml` directory:

```xml
<mesh filename="meshes/stl/base_link.stl"/>
<mesh filename="meshes/dae/arm/link_1.dae"/>
<texture filename="textures/body.png"/>
```

Use `/` separators, preserve filename case, and keep each path below the URDF
directory. Absolute paths, parent traversal, and URI schemes such as
`package://` are not portable through Soma.

See [Robot render asset distribution](ROBOT_MODEL_ASSETS.md) for the complete
path format, small-model Git thresholds, large-model artifact manifest,
`fetch_model.py` workflow, Soma-to-Client streaming behavior, cache limits, and
release checklist.

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
| `type` | string | yes | Component type. Common values include `mobile_base`, `wheel`, `battery`, `body_part`, `lidar_2d`, `rgb_camera`, `rgbd_camera`, and `audio_io`; custom values are allowed. |
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
