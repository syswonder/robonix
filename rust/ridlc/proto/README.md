# Robonix runtime meta API (gRPC)

This directory defines the **meta API** used by robonix nodes to talk to **robonix-server**: channel registration and resolution. Communication uses **gRPC** (not ROS services) so the management plane is simple and language-agnostic.

## Service: `RobonixRuntime`

- **RegisterNode** – register a process and declare RIDL namespaces/capabilities (optional).
- **RegisterStream** / **ResolveStream** – register as stream provider or resolve consumer topic name.
- **RegisterCommand** / **ResolveCommand** – register as command server or resolve action name for client.
- **RegisterQuery** / **ResolveQuery** – register as query server or resolve service name for client.

All channel names (topic, action, service) are allocated by robonix-server and returned in the response.

The current `robonix-server` implementation returns **opaque channel names** rather than semantic names derived from the RIDL path. This keeps business identifiers out of the transport endpoint and leaves room for future policy changes in channel allocation. At the moment they are emitted under a server-owned prefix such as:

- `/rbnx/ch/s/n<uuid>` for streams
- `/rbnx/ch/c/n<uuid>` for commands
- `/rbnx/ch/q/n<uuid>` for queries

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

**When using ridlc:** run `./proto/gen_grpc.sh` once in the ridlc repo so `proto/gen/` exists. Then `ridlc --lang python --layout workspace -o <out> -i <ridl> ...` will copy these two files into `<out>/src/robonix_interfaces/`. The output directory is then a directly buildable ROS workspace containing the runtime package plus generated interface packages; no extra PYTHONPATH or manual copy needed.

## robonix-server

`robonix-server` now implements `RobonixRuntime` directly and exposes it on a dedicated gRPC endpoint.

Default runtime endpoint behavior:

- Listen address: `ROBONIX_META_GRPC_ADDR` or `0.0.0.0:50051`
- Advertised endpoint returned by `RegisterNode`: `ROBONIX_META_GRPC_ENDPOINT` or the listen address

Nodes created from RIDL-generated code receive a `runtime_client` (e.g. `RobonixRuntimeStub`) connected to this endpoint and call the RPCs above for registration and resolution.

This runtime meta API is a **control-plane API only**. The generated Python code still creates standard ROS 2 publishers/subscribers/actions/services after it receives the allocated channel name.
