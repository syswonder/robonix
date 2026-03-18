# ridlc

`ridlc` is the Robonix Interface Definition Language compiler.

It reads `.ridl` definitions and generates a directly buildable ROS 2 workspace
for the current RIDL-first architecture.

## What it generates

For `--lang python --layout workspace`, `ridlc` emits:

- `src/generated/robonix_interfaces`
  - generated Python runtime bindings
  - copied gRPC runtime client modules
- `src/generated/robonix_interfaces_ros2`
  - generated ROS `.srv` and `.action` interface package
- `src/vendor/*`
  - vendored ROS dependencies required by the generated workspace
- `src/app/robonix_interfaces_app`
  - user-editable application skeletons
  - includes a generated `combined_runtime.py` entry point for multi-interface hosting

The current implementation focuses on `--lang python`.

## Runtime model

RIDL describes interface semantics, not fixed transport endpoint names.

`robonix-server` acts as the runtime control plane:

- generated nodes register or resolve `query`, `command`, and `stream` interfaces over gRPC
- the server allocates opaque ROS-compatible channel names
- the generated ROS 2 code then binds those names to services, actions, or topics

This registration logic belongs in the generated stubs themselves. In the
target architecture, package managers and deployment tools should launch
components and inspect status, but not manually perform interface registration
on behalf of RIDL-generated code.

Current ROS 2 mapping:

- `query` -> service
- `command` -> action
- `stream` -> topic

## Generate gRPC helpers

From `rust/ridlc`:

```bash
python3 -m venv .venv
.venv/bin/pip install grpcio-tools
./proto/gen_grpc.sh
```

This produces:

- `proto/gen/robonix_runtime_pb2.py`
- `proto/gen/robonix_runtime_pb2_grpc.py`

`ridlc` copies them into generated output automatically.

## Generate a workspace

Example:

```bash
mkdir -p tmp
cargo run -- --lang python --layout workspace \
    -I ../robonix-interfaces/lib/robonix_runtime_interfaces \
    -I ../robonix-interfaces/lib/rcl_interfaces \
    -I ../robonix-interfaces/lib/common_interfaces \
    -o tmp \
    -i ../robonix-interfaces/ridl
```

## Current namespace model

New canonical handwritten RIDL lives under:

- `robonix/hal/*`
- `robonix/system/*`

The old `robonix/prm/*` namespace may still appear in legacy assets or migration
tests, but it is no longer the target architecture model.

## Generated API shape

Each interface generates:

- abstract base classes
- gRPC registration / resolution helpers
- concrete ROS 2 bootstrap classes
- convenience `create_*` factory functions
- matching app-layer skeleton modules under `src/app`

Those generated helpers are the intended place where startup-time interaction
with `robonix-server` happens.

For example:

- `query status` -> `create_status_server(...)`, `create_status_client(...)`
- `command motion_cmd` -> `create_motion_cmd_server(...)`, `create_motion_cmd_client(...)`
- `stream pose` -> `create_pose_publisher(...)` or `create_pose_subscriber(...)`

Stream direction matters:

- `output` stream field -> generated publisher path
- `input` stream field -> generated subscriber path

## App layer

The generated `src/app/robonix_interfaces_app` package is the intended place for
user customization.

Do not edit:

- `src/generated`
- `src/vendor`

Instead, customize:

- generated per-interface skeletons under `src/app/robonix_interfaces_app/...`
- `combined_runtime.py` when one process needs to host multiple interfaces

## User logic completion (Python)

| Primitive | Server/Provider | Client/Consumer |
|-----------|-----------------|-----------------|
| **Query** | `server.start(handler)` — pass `handler(request, response) -> response` | `client.call(request)` → `Response \| None` |
| **Stream** | Call `publisher.publish(msg)` (e.g. in a timer) | `subscriber.start(callback)` — pass `callback(msg)` |
| **Command** | Assign `server.execute = fn` where `fn(request, goal_handle) -> result`; optionally `goal_handle.publish_feedback(fb)` | `client.send(request)` → `goal_handle`; then `goal_handle.get_result_async()`; or use `send_goal_async(..., feedback_callback=...)` for feedback |

See [ridlc §5](https://github.com/syswonder/robonix-book/blob/main/src/chapter3-developer-guide/ridlc.md#5-用户逻辑补全python) for full details.

## Tests

`ridlc` currently has two main validation paths:

- `./tests/test_codegen.sh`
  - validates generated file layout
  - checks Python syntax
  - verifies app skeleton output
- `./tests/run_zenoh_rmw_e2e.sh`
  - generates a temporary workspace
  - builds generated, vendor, app, and test packages together
  - starts `robonix-server`
  - runs one combined test application process
  - verifies end-to-end `query`, `stream`, and `command` behavior with `rmw_zenoh_cpp`

The end-to-end test requires a real ROS 2 environment and does not require
`robonix-sdk`.

## Related docs

- runtime gRPC API: `proto/README.md`
- test layout: `tests/README.md`
