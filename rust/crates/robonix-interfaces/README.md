# robonix-interfaces

**ROS 2 IDL** (`lib/**`), **generated** Protobuf (`robonix_proto/**`), and optional **iceoryx2 Python payloads** (`robonix_iceoryx2_py/**`).

## Architecture (how the pieces fit)

1. **`rust/contracts/**/*.toml`** — **canonical logical interface** (`[contract].id`), optional semantics, **`[io]`** references into ROS IDL, and **`[mode].type`** (`rpc` | `rpc_server_stream` | `rpc_client_stream` | `topic_out` | `topic_in`) that fixes the gRPC *shape* for that id. See [`rust/contracts/README.md`](../contracts/README.md).
2. **`lib/**`** — **ROS 2 concrete schemas**: `.msg` and `.srv` are the vocabulary contracts use; they are also what native ROS 2 stacks publish/service-call.
3. **`ridlc`** — **contract-centric codegen**: resolves all types from `lib`, emits per-package `robonix_proto/<pkg>.proto` (messages + legacy package `*Service` from `.srv` where applicable), and **`robonix_contracts.proto`** (`package robonix.contracts`) — one gRPC service per contract id for cross-runtime bridging.
4. **`robonix_iceoryx2_py/**`** — **`ridlc --lang python`** ctypes for **messages only** (SHM / iceoryx2); unrelated to contract ids. See `robonix_iceoryx2_py/README.md`.

**ROS mental model vs contract `[mode]`:** pub/sub direction is a **registration** concern. **`topic_out`** / **`topic_in`** use **`[io.msg]`** with **`msg`**; unary **`rpc`** and streaming **`rpc_server_stream`** / **`rpc_client_stream`** use **`[io.srv]`** with **`srv`** (see `rust/contracts/README.md`).

## Policy

- **Do not hand-edit** anything under `robonix_proto/` or `robonix_iceoryx2_py/`; change **`lib`** IDL and/or **`rust/contracts`**, then rerun `ridlc`.
- **Do not add ad-hoc `.proto`** under `robonix_proto/` for Robonix-owned types — extend **`lib`** and regenerate, or reference **`protobuf://...`** from a contract when intentionally pointing at protos outside `lib`.
- **Prefer contracts** for stable **interface ids** and gRPC service names (`robonix_contracts.proto`). Per-package `*Service` RPCs from `.srv` remain for ROS-shaped unary APIs not yet lifted into a contract file.

Regenerate protos (IDL + contracts; writes one file per ROS package **and** `robonix_contracts.proto`; **does not delete** stale `.proto` if you removed a package—delete manually or wipe the dir first):

```bash
cd rust
cargo run -p ridlc -- --lang proto \
  -I robonix-interfaces/lib \
  --contracts contracts \
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
| `../contracts/**` | **Logical interface** TOML (ids + `[io]` + `[mode]`); drives `robonix_contracts.proto` |
| `lib/prm_base/msg/`, `lib/prm_base/srv/` | ROS IDL for base primitives `robonix/prm/base/*` |
| `lib/prm/README.md` | Index of `prm_*` packages |
| `lib/prm_camera/` | Camera: payloads mostly **common_interfaces** (see tables); gRPC services from **contracts** |
| `lib/vlm/srv/` | System VLM `robonix/sys/model/vlm` (payloads referenced by contracts) |
| `lib/robonix_msg/` | Shared message types |
| `lib/common_interfaces/` | Upstream ROS message trees (submodule / vendor) |
| `robonix_proto/` | **Generated** `.proto` (per-package + `robonix_contracts.proto`) |
| `robonix_iceoryx2_py/` | **Generated** ctypes for `.msg` (iceoryx2); see folder README |

---

## ridlc → gRPC naming

For ROS package **`pkgname`**, output is **`robonix_proto/pkgname.proto`** with `package robonix.pkgname;`:

- Each **`.msg`** → a **`message`** of the same name.
- Each **`.srv`** → `Name_Request` / `Name_Response` plus **`rpc Name`** on **`PkgnameService`** (PascalCase package + `Service`).

**Contract-bound streaming / unary** for a `robonix/...` id is **not** expressed as extra magic on `.srv`; it is **`rust/contracts` → `robonix_contracts.proto`** (`robonix.contracts.*` services). Per-package protos still expose **message** types and optional **legacy** unary RPCs from `.srv`.

**Pub-sub only (`.msg` in proto):** there is **no** per-message RPC in that package file; the **`message`** type is the shared payload for ROS topics, gRPC streams (via contracts), and bridges.

---

## System services (`robonix/sys/...`)

| Abstract interface ID | Mode | Dir | ROS IDL (in repo) | gRPC (generated) |
|------------------------|------|-----|-------------------|------------------|
| `robonix/sys/model/vlm/chat` | RPC | — | `lib/vlm/srv/Chat.srv` | `robonix_proto/vlm.proto` → `robonix.vlm.VlmService` / `Chat` |
| `robonix/sys/model/vlm/chat` | Server streaming | — | `lib/vlm/srv/ChatStream.srv`, `lib/vlm/msg/ChatStreamEvent.msg` | same file → `ChatStream` → `stream ChatStreamEvent` |
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

ROS packages **`sensor_msgs`**, **`robonix_msg`**; camera **gRPC** service shapes come from **`rust/contracts/prm/camera_*.toml`** → `ridlc --contracts` → **`robonix_proto/robonix_contracts.proto`** (e.g. `PrmCameraRgb`).

| Abstract interface ID | Mode | Direction | ROS IDL (path) | gRPC (generated) |
|------------------------|------|-----------|----------------|------------------|
| `robonix/prm/camera/rgb` | pub-sub | **output** | Topic: `sensor_msgs/msg/Image.msg`; gRPC: contract `rust/contracts/prm/camera_rgb*.toml` → `robonix_contracts.proto` `PrmCameraRgb.Stream` → `stream sensor_msgs.Image` |
| `robonix/prm/camera/depth` | pub-sub | **output** | same `Image.msg` | same |
| `robonix/prm/camera/ir` | pub-sub | **output** | same | same |
| `robonix/prm/camera/intrinsics` | pub-sub | **output** | `lib/common_interfaces/sensor_msgs/msg/CameraInfo.msg` | `CameraInfo` |
| `robonix/prm/camera/rgbd` | pub-sub | **output** | `lib/robonix_msg/msg/RGBD.msg` | `robonix_msg.proto` |

### Zero-copy transport

When `shared_memory` transport is negotiated, camera interfaces use **`robonix_msg/msg/ZeroCopyFrame.msg`** instead of `sensor_msgs/Image`. The `ZeroCopyFrame` carries a buffer handle (not pixel data), so publishing/subscribing involves only a ~128-byte descriptor. Pixel data remains in the Robonix-managed SHM region and is never serialized.

| Component | Purpose |
|-----------|---------|
| `robonix_msg/BufferFormat` | Pixel format constants (RGB8, BGR8, NV12, DEPTH_U16, …) — shared by Rust `BufferFormat` enum and Python `rbnx_buffer.py` |
| `robonix_msg/MemoryDomain` | Memory domain (CPU / GPU / Unified) |
| `robonix_msg/ZeroCopyFrame` | Image-specific buffer descriptor with width/height/stride/format |
| `robonix_msg/BufferDescriptor` | General N-dimensional buffer descriptor (tensors, point clouds, etc.) |

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
