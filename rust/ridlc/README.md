# ridlc

`ridlc` is the Robonix Interface Definition Language compiler.

It reads `.ridl` files and generates:

- Python API code for Robonix nodes
- gRPC meta-API client files used to talk to `robonix-server`
- ROS IDL source files (`.srv`, `.action`) for ROS 2 transport bindings
- ROS 2 package boilerplate (`package.xml`, `setup.py`, `setup.cfg`, `resource/`)

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

`ridlc --lang python` copies these files into the output directory automatically.

## Generate Code

Example:

```bash
mkdir -p tmp
cargo run -- --lang python \
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

For `-o ./tmp`, the generated tree looks like:

```text
tmp/
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
  rosidl/
    robonix_interfaces_ros2/
      srv/
        PrmBaseGetStatus.srv
      action/
        PrmBaseMove.action
```

There are two important parts:

1. Python package code under `robonix/...`
2. ROS IDL source files under `rosidl/...`

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

## Related Docs

- gRPC meta API details: `proto/README.md`
