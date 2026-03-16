# Examples

Robonix PoC examples: calling queries provided by robonix-server.

## Prerequisites

1. Build robonix-server: `cargo build --manifest-path robonix-server/Cargo.toml`
2. Start robonix-server: `../start_server` (in another terminal)

## Example list

| Example | Description |
|---------|-------------|
| **callquery** | Rust client calling any registered query |
| **list_interfaces** | List runtime nodes and channels (gRPC InspectRuntime) |
| **python_ping_client** | Python client with manifest, calls ping query |
| **stream_demo** | Stream: stream_server (pose publisher) + stream_client (subscriber), connected via robonix-server |
| **query_demo** | Query: semantic_server (semantic_query) + semantic_client, with sample semantic map |
| **skill_demo** | Command: skill_server (greet) + skill_client, per-skill command with typed ROS msg |
| **prm_camera_vendor** | 相机厂商示例：prm::camera (rgb, depth, rgbd, intrinsics)，不实现 ir |
| **prm_arm_vendor** | 机械臂厂商示例：prm::arm (move_ee, joint_trajectory) + prm::gripper (close, open) |
| **map_semantic_service** | 地图服务示例：robonix/system/map/semantic_query |

## Run

```bash
# List nodes and channels (build robonix-server first)
./list_interfaces
./list_interfaces 127.0.0.1:50051

# Call ping (Rust)
./callquery robonix-server robonix/system/debug/ping '"hello"'

# Call ping (Python): rbnx build and start the node
rbnx build -p python_ping_client && rbnx start -p python_ping_client -n call_ping
```

## rbnx package control

Build and run with rbnx: `rbnx build -p <package>` → `rbnx start -p <package> -n <node>` (one node per start; start blocks until the process exits).

- Each example has `robonix_manifest.yaml` and `package.xml` (custom ROS2 deps; see `skill_demo/package.xml`, `prm_camera_vendor/package.xml`).
- **Import resolution**: Run `python3 rust/tools/gen_pyrightconfig.py` after building examples to regenerate `pyrightconfig.json` (for VS Code/Basedpyright). Build examples first so `rbnx-build/` exists.

## User logic completion

| Example | Server/Provider | Client/Consumer |
|---------|-----------------|-----------------|
| **query_demo** | `server.start(handler)` — handler fills `response` | `client.call(request)` |
| **stream_demo** | `publisher.publish(msg)` in timer | `subscriber.start(on_msg)` |
| **skill_demo** | `server.execute = execute` — execute returns result, optionally `goal_handle.publish_feedback()` | `client.send(request)` or `send_goal_async(..., feedback_callback=...)` |

See `docs/src/chapter3-developer-guide/ridlc.md` §5 for full documentation.

## Test scripts

Start robonix-server (`../start_server`) first, then:

```bash
# Individual tests
./test_query_demo.sh   # semantic_query server + client
./test_skill_demo.sh   # execute command server + client
./test_stream_demo.sh  # pose stream publisher + subscriber

# Run all
./test_all_demos.sh
```
