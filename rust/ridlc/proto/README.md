# Robonix runtime meta API (gRPC)

This directory defines the **meta API** used by robonix nodes to talk to **robonix-server**: channel registration and resolution. Communication uses **gRPC** (not ROS services) so the management plane is simple and language-agnostic.

## Service: `RobonixRuntime`

- **RegisterNode** – register a process and declare RIDL namespaces/capabilities (optional).
- **RegisterStream** / **ResolveStream** – register as stream provider or resolve consumer topic name.
- **RegisterCommand** / **ResolveCommand** – register as command server or resolve action name for client.
- **RegisterQuery** / **ResolveQuery** – register as query server or resolve service name for client.

All channel names (topic, action, service) are allocated by robonix-server and returned in the response.

## Generate Python stubs

From the `ridlc` repo root:

```bash
pip install grpcio-tools
chmod +x proto/gen_grpc.sh
./proto/gen_grpc.sh
```

Output is under `proto/gen/`:

- `robonix_runtime_pb2.py` – message types
- `robonix_runtime_pb2_grpc.py` – `RobonixRuntimeStub` and `RobonixRuntimeServicer`

RIDL-generated Python expects to import these (e.g. `from robonix_runtime_pb2 import RegisterQueryRequest`).

**When using ridlc:** run `./proto/gen_grpc.sh` once in the ridlc repo so `proto/gen/` exists. Then `ridlc --lang python -o <out> -i <ridl> ...` will copy these two files into `<out>/` and emit ROS2 package files (`package.xml`, `setup.cfg`, `setup.py`, `resource/`). The output directory is then a ready-to-use ament_python package: add it to your workspace and build with colcon; no extra PYTHONPATH or manual copy needed.

## robonix-server

Implement the `RobonixRuntime` service in your server and expose it on a fixed gRPC endpoint. Nodes created from RIDL-generated code receive a `runtime_client` (e.g. `RobonixRuntimeStub`) connected to this endpoint and call the RPCs above for registration and resolution.
