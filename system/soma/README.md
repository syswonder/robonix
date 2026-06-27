# Soma - raw body YAML and URDF service

Soma loads one or more deployment directories, caches each deployment's raw
`soma.yaml` and referenced URDF text, then exposes them to other Robonix
components over gRPC.

Soma v2 does not render or interpret a self-description. Pilot calls
`robonix/system/soma/get_yaml` to receive the original YAML string, and other
services call `robonix/system/soma/get_urdf` to receive the corresponding URDF
XML string.

## Config

```yaml
atlas_endpoint: 127.0.0.1:50051
listen: 127.0.0.1:50091
provider_id: soma
robonix_root: ../../
default_robot: test_ci_robot
start_packages: false
rbnx_bin: rbnx
deployments:
  - examples/test_ci
```

Relative `robonix_root` is resolved from the config file directory. Relative
`deployments` entries are then resolved from that Robonix root. For example,
when the config file is `system/soma/config.yaml`, `robonix_root: ../../` and
`deployments: [examples/test_ci]` point at `<repo>/examples/test_ci` no matter
which directory `cargo run -p robonix-soma -- --config <path>` is launched from.

If `robonix_root` is omitted, Soma tries `ROBONIX_SOURCE_PATH`, `ROBONIX_ROOT`,
then searches upward from the current directory for the Robonix repository root.

Each deployment directory must contain:

- `robonix_manifest.yaml`
- `soma.yaml` or compatible names such as `robonix.soma.yaml`
- the URDF file referenced by `urdf.path`

## Run

```bash
cargo run -p robonix-soma -- --config ./soma.local.yaml
```

At startup Soma reads every deployment, optionally runs each local primitive and
skill package with `rbnx start -p <package_dir> --endpoint <atlas>`, prints a
startup report, registers itself in Atlas, and serves gRPC on `listen`.

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
