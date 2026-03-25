# robonix-interfaces

Source tree for **ROS 2 IDL** (`lib/**`) and **generated** gRPC **`.proto`** (`robonix_proto/**`).

## Policy

- **Single source of truth:** ROS IDL only — `lib/<ros_package>/msg/*.msg`, `srv/*.srv`.
- **All** files under `robonix_proto/` are **produced by** `ridlc --lang proto` from that IDL. **Do not hand-edit** `.proto` in this tree; **do not add vendor- or app-specific `.proto` files here** — extend the system by adding or changing ROS IDL under `lib/` and regenerating. Regenerate after IDL changes.
- **Topic / pub-sub prm:** On ROS, endpoints are normal topics with the same `.msg` payload. On gRPC, use the **generated `message`** types (same shapes as ROS) for streaming or bridging — still **only** from ridlc output, not ad-hoc protos.
- **RPC (`.srv`):** Generated `service { rpc … }` unary RPCs per ridlc rules below.
- **Pub-sub over gRPC:** The first line of a `.srv` may be a **codegen-only** directive, e.g. `# @robonix.grpc stream_server sensor_msgs/msg/Image`, to emit **server/client streaming** RPCs (see `docs/.../interface-spec.md`). ROS 2 does not execute these as native services.

Regenerate protos from ROS IDL (writes one file per ROS package; **does not delete** stale `.proto` if you removed a package from `lib/`—delete those files manually or wipe the directory first):

```bash
cd rust
cargo run -p ridlc -- --lang proto \
  -I robonix-interfaces/lib \
  -o robonix-interfaces/robonix_proto
```

For a **clean** regen: `rm -f robonix-interfaces/robonix_proto/*.proto` then run the command above.

Then regenerate Python stubs for examples:

```bash
cd rust && ./examples/scripts/gen_proto_python.sh
```

> Full `-I robonix-interfaces/lib` scans all packages (including `common_interfaces`) and may take a while; use CI or narrow includes for incremental checks.

---

## Layout (excerpt)

| Path | Role |
|------|------|
| `lib/prm_base/msg/`, `lib/prm_base/srv/` | ROS IDL for base primitives `robonix/prm/base/*` |
| `lib/prm/README.md` | Index of `prm_*` packages |
| `lib/prm_camera/` | Camera: payloads mostly **common_interfaces** (see tables) |
| `lib/vlm/srv/` | System VLM `robonix/sys/model/vlm` |
| `lib/robonix_msg/` | Shared message types |
| `lib/common_interfaces/` | Upstream ROS message trees (submodule / vendor) |
| `robonix_proto/` | **Generated** gRPC `.proto` (see command above) |

---

## ridlc → gRPC naming

For ROS package **`pkgname`**, output is **`robonix_proto/pkgname.proto`** with `package robonix.pkgname;`:

- Each **`.msg`** → a **`message`** of the same name.
- Each **`.srv`** → `Name_Request` / `Name_Response` plus **`rpc Name`** on **`PkgnameService`** (PascalCase package + `Service`).

**Pub-sub only (`.msg` in proto):** there is **no** RPC in that file for the message itself; the **`message`** type is still the gRPC-side payload type for streams or bridges. If you need a streaming RPC wrapper, extend **ridlc** or a small **generated** shim — not hand-written duplicate `.proto` definitions.

---

## System services (`robonix/sys/...`)

| Abstract interface ID | Mode | Dir | ROS IDL (in repo) | gRPC (generated) |
|------------------------|------|-----|-------------------|------------------|
| `robonix/sys/model/vlm/chat` | RPC | — | `lib/vlm/srv/Chat.srv` | `robonix_proto/vlm.proto` → `robonix.vlm.VlmService` / `Chat` |
| `robonix/sys/model/vlm/describe` | RPC | — | `lib/vlm/srv/Describe.srv` | same file → `Describe` |

---

## Primitives `robonix/prm/base` (`lib/prm_base`, package **`prm_base`**)

| Abstract interface ID | Mode | Direction (pub-sub) | ROS IDL (in repo) | gRPC (`robonix_proto/prm_base.proto`, generated) |
|------------------------|------|------------------------|-------------------|---------------------------------------------------|
| `robonix/prm/base/move` | pub-sub | **input** | `lib/prm_base/msg/MoveCommand.msg` | `message MoveCommand` (payload type) |
| `robonix/prm/base/navigate` | RPC | — | `lib/prm_base/srv/Navigate.srv` | `PrmBaseService` / `Navigate` → `/robonix.prm_base.PrmBaseService/Navigate` |
| `robonix/prm/base/nav_status` | RPC | — | `lib/prm_base/srv/GetNavigationStatus.srv` | `PrmBaseService` / `GetNavigationStatus` |
| `robonix/prm/base/cancel_nav` | RPC | — | `lib/prm_base/srv/CancelNavigation.srv` | `PrmBaseService` / `CancelNavigation` |
| `robonix/prm/base/stop` | RPC | — | `lib/prm_base/srv/Stop.srv` | `PrmBaseService` / `Stop` |

**Not checked in under this package; standard ROS types** (implementations must declare `msg` + topic in `DeclareInterface.metadata_json`):

| Abstract interface ID | Mode | Direction | ROS IDL (under `lib/common_interfaces/...`) | gRPC (generated package) |
|------------------------|------|-----------|-----------------------------------------------|---------------------------|
| `robonix/prm/base/odom` | pub-sub | **output** | `nav_msgs/msg/Odometry.msg` | `nav_msgs.proto` → `Odometry` |
| `robonix/prm/base/pose_cov` | pub-sub | **output** | `geometry_msgs/msg/PoseWithCovarianceStamped.msg` | `geometry_msgs.proto` |
| `robonix/prm/base/joint_state` | pub-sub | **output** | `sensor_msgs/msg/JointState.msg` | `sensor_msgs.proto` |
| `robonix/prm/base/goal_status` | RPC (preferred) | — | `lib/prm_base/srv/GetNavigationStatus.srv` | `PrmBaseService` / `GetNavigationStatus` (topic payload may still use `robonix_msg/NavigationStatus.msg` if you publish status) |

**Default topic names** (overridable in metadata): set at registration; prefer instance prefixes, e.g. `.../prm/base/move/cmd_vel`, `.../odom`.

---

## Primitives `robonix/prm/camera` (payloads in **common_interfaces**)

ROS packages **`sensor_msgs`**, **`robonix_msg`**, **`prm_camera`** (streaming RPC only). Camera **payload** types live in `sensor_msgs`; `prm_camera` supplies ridlc directives for gRPC streaming (see docs **interface-spec**).

| Abstract interface ID | Mode | Direction | ROS IDL (path) | gRPC (generated) |
|------------------------|------|-----------|----------------|------------------|
| `robonix/prm/camera/rgb` | pub-sub | **output** | Topic: `sensor_msgs/msg/Image.msg`; stream RPC: `lib/prm_camera/srv/SubscribeRgb.srv` (`# @robonix.grpc stream_server …`) | **Server streaming** of `sensor_msgs.Image`: `prm_camera.proto` → `PrmCameraService/SubscribeRgb` |
| `robonix/prm/camera/depth` | pub-sub | **output** | same `Image.msg` | same |
| `robonix/prm/camera/ir` | pub-sub | **output** | same | same |
| `robonix/prm/camera/intrinsics` | pub-sub | **output** | `lib/common_interfaces/sensor_msgs/msg/CameraInfo.msg` | `CameraInfo` |
| `robonix/prm/camera/rgbd` | pub-sub | **output** | `lib/robonix_msg/msg/RGBD.msg` | `robonix_msg.proto` |

---

## Primitives `robonix/prm/sensor`

| Abstract interface ID | Mode | Direction | ROS IDL (path) | gRPC (generated) |
|------------------------|------|-----------|----------------|------------------|
| `robonix/prm/sensor/pointcloud` | pub-sub | **output** | `lib/common_interfaces/sensor_msgs/msg/PointCloud2.msg` | `PointCloud2` |
| `robonix/prm/sensor/lidar` | pub-sub | **output** | `lib/common_interfaces/sensor_msgs/msg/LaserScan.msg` | `LaserScan` |
| `robonix/prm/sensor/imu` | pub-sub | **output** | `lib/common_interfaces/sensor_msgs/msg/Imu.msg` | `Imu` |

---

## `robonix/prm/arm`, `gripper`, `force_torque`

Add `lib/prm_arm/` etc. with `.msg`/`.srv` over time; see `ridl/prm/**` for intent. Until then, declare payload types in **metadata**; generated proto path remains **`robonix_proto/<ros_pkg>.proto`** from ridlc.
