# ridlc

`ridlc` is the Robonix Interface Definition Language compiler.

It reads `.ridl` files and generates:

- Python API code for Robonix nodes
- gRPC meta-API client files used to talk to `robonix-server`
- a directly buildable ROS 2 workspace by default

The current implementation focuses on `--lang python`.

## Goals

RIDL describes interface semantics, not transport details.

Today the generated Python code includes ROS 2 bindings, but runtime channel allocation is handled by `robonix-server` over gRPC:

- `stream` -> transport stream channel
- `command` -> transport command channel
- `query` -> transport request/response channel

For ROS 2, those channels map to:

- `stream` -> topic
- `command` -> action
- `query` -> service

The generated code does not hardcode topic names, action names, or service names. It asks `robonix-server` for them at runtime.

`robonix-server` currently acts as a central runtime control plane:

- RIDL-generated nodes use the gRPC `RobonixRuntime` API for registration and channel resolution
- the server returns opaque runtime channel names owned by the server
- the generated code then binds those channels to normal ROS 2 topic/action/service objects

This means the control plane is gRPC, while the generated Python transport binding is still ROS 2.

## Generate gRPC Meta API Once

From the `ridlc` directory:

```bash
python3 -m venv .venv
.venv/bin/pip install grpcio-tools
./proto/gen_grpc.sh
```

This generates:

- `proto/gen/robonix_runtime_pb2.py`
- `proto/gen/robonix_runtime_pb2_grpc.py`

`ridlc --lang python` copies these files into the generated runtime package automatically.

## Generate Code

Example:

```bash
mkdir -p tmp
cargo run -- --lang python --layout workspace \
    -I ../robonix-interfaces/lib/rcl_interfaces \
    -I ../robonix-interfaces/lib/common_interfaces \
    -o tmp \
    -i ../robonix-interfaces/ridl
```

Or use the helper script:

```bash
./test.sh
```

## Output Layout

For `-o ./tmp --layout workspace`, the generated tree looks like:

```text
tmp/
  src/
    robonix_interfaces/
      package.xml
      setup.py
      setup.cfg
      resource/
        robonix_interfaces
      robonix_runtime_pb2.py
      robonix_runtime_pb2_grpc.py
      robonix/
        prm/
          base/
            get_status_query.py
            move_command.py
          localization/
            odom_stream.py
    robonix_interfaces_ros2/
      package.xml
      CMakeLists.txt
      srv/
        PrmBaseGetStatus.srv
      action/
        PrmBaseMove.action
    robonix_msgs/
      package.xml
      CMakeLists.txt
      msg/
        CommandResult.msg
```

There are two important parts:

1. Runtime Python package code under `src/robonix_interfaces`
2. ROS interface packages under `src/robonix_interfaces_ros2` and `src/robonix_msgs`

## Generated Python API

Each RIDL interface generates:

- abstract base classes
- gRPC runtime helpers
- concrete ROS 2 bootstrap classes
- convenience factory functions

### Query example

`query get_status` generates:

- `GetStatusClient`
- `GetStatusServer`
- `resolve_get_status_service(...)`
- `register_get_status_server(...)`
- `Ros2GetStatusClient`
- `Ros2GetStatusServer`
- `create_get_status_client(...)`
- `create_get_status_server(...)`

### Command example

`command move` generates:

- `MoveClient`
- `MoveServer`
- `resolve_move_client_action(...)`
- `register_move_server(...)`
- `Ros2MoveClient`
- `Ros2MoveServer`
- `create_move_client(...)`
- `create_move_server(...)`

### Stream example

`stream odom` generates:

- `OdomPublisher`
- `OdomSubscriber`
- `register_odom_provider(...)`
- `resolve_odom_consumer_topic(...)`
- `Ros2OdomPublisher`
- `Ros2OdomSubscriber`
- `create_odom_publisher(...)`
- `create_odom_subscriber(...)`

## Runtime Model

The intended startup flow is:

1. Create a gRPC client for `robonix-server`
2. Register or resolve the interface through the generated helper
3. Receive a concrete runtime channel name
4. Initialize the ROS 2 publisher/subscriber/action/service with that channel
5. Run the node normally

That flow is already encoded in the generated `Ros2...` classes and `create_...` helper functions.

By default, `robonix-server` exposes this runtime API on `127.0.0.1:50051` / `0.0.0.0:50051` depending on how it is launched. The exact listen and advertised endpoint can be configured through:

- `ROBONIX_META_GRPC_ADDR`
- `ROBONIX_META_GRPC_ENDPOINT`

## Server Usage Example

Example for a query server:

```python
import grpc
import rclpy

from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.prm.base import create_get_status_server


def main():
    grpc_channel = grpc.insecure_channel("127.0.0.1:50051")
    runtime_client = RobonixRuntimeStub(grpc_channel)

    server = create_get_status_server(runtime_client, node_id="status_server_1")

    def handler(request, response):
        response.res.data = "ok"
        return response

    server.start(handler)
    rclpy.spin(server)


if __name__ == "__main__":
    main()
```

Example for a command server:

```python
import grpc
import rclpy

from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.prm.base import create_move_server


class MyMoveServer:
    pass


def main():
    grpc_channel = grpc.insecure_channel("127.0.0.1:50051")
    runtime_client = RobonixRuntimeStub(grpc_channel)

    server = create_move_server(runtime_client, node_id="move_server_1")
    server.start()
    rclpy.spin(server)


if __name__ == "__main__":
    main()
```

## Client Usage Example

Example for a query client:

```python
import grpc

from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.prm.base import create_get_status_client


def main():
    grpc_channel = grpc.insecure_channel("127.0.0.1:50051")
    runtime_client = RobonixRuntimeStub(grpc_channel)

    client = create_get_status_client(
        runtime_client,
        requester_id="skill_1",
        target="status_server_1",
    )

    # Construct the generated request object from the built ROS interface package.
    req = client._srv_type.Request()
    req.req.data = "ping"
    resp = client.call(req)
    print(resp)


if __name__ == "__main__":
    main()
```

Example for a stream subscriber:

```python
import grpc
import rclpy

from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from robonix.prm.localization import create_odom_subscriber


def main():
    grpc_channel = grpc.insecure_channel("127.0.0.1:50051")
    runtime_client = RobonixRuntimeStub(grpc_channel)

    sub = create_odom_subscriber(
        runtime_client,
        requester_id="consumer_1",
        target="odom_provider_1",
    )

    sub.start(lambda msg: print(msg))
    rclpy.spin(sub)


if __name__ == "__main__":
    main()
```

## robonix-server Compatibility Notes

`robonix-server` still contains its legacy ROS `/rbnx/*` service APIs for primitive/service/skill/task management. Those APIs are separate from the RIDL runtime control plane.

For RIDL-generated nodes:

- use the gRPC runtime meta API for channel registration and resolution
- use ROS 2 for the actual transport objects created after resolution

## ROS 2 Package Notes

The output directory includes `package.xml` and Python package files, but the generated `.srv` and `.action` files under `rosidl/` must also be built into importable ROS interface packages.

Today, the generated Python code expects modules such as:

- `robonix_interfaces_ros2.srv`
- `robonix_interfaces_ros2.action`

So in practice you should:

1. Put the generated output into a ROS workspace
2. Build the Python package
3. Build or vendor the generated ROS IDL package(s) from `rosidl/`
4. Source the workspace

Once those ROS interface modules are available, the generated Python code can import the service/action types and run directly.

## Current Status

Current Python generation supports:

- base API classes
- gRPC runtime registration and resolution helpers
- ROS 2 bootstrap helpers for `stream`, `command`, and `query`
- generated `.srv` and `.action` files
- generated ROS 2 Python package boilerplate

## Tests

The `tests/` directory now contains two validation paths:

- `./tests/test_codegen.sh`
  - runs `ridlc`
  - checks the generated file set
  - verifies generated Python syntax
  - verifies the generated app skeleton under `src/app/robonix_interfaces_app`
- `./tests/run_zenoh_rmw_e2e.sh`
  - prepares a ROS workspace from generated output
  - builds generated Python, ROS interface, and app packages
  - starts `robonix-server`
  - starts one combined test application that imports multiple generated RIDL modules in a single process
  - runs query/service, stream/topic, and command/action flows with `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
  - verifies runtime channel allocation and end-to-end transport usage for service, topic, and action flows in the combined-app scenario

The end-to-end test requires a real ROS2 environment with `rclpy`
and `rmw_zenoh_cpp`. It does not require `robonix-sdk`.

## Related Docs

- gRPC meta API details: `proto/README.md`
