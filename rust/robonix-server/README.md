# robonix-server

`robonix-server` is the central runtime and management process for Robonix.

It currently hosts two different API layers:

- legacy ROS `/rbnx/*` service APIs used by the existing Robonix CLI/runtime modules
- the gRPC `RobonixRuntime` meta API used by RIDL-generated nodes

## What it does

`robonix-server` starts:

- primitive, service, skill, task, ping, and core ROS service endpoints on `/rbnx/*`
- the optional web UI
- the gRPC runtime meta API for RIDL-generated transport bootstrap

The runtime meta API is a control-plane API. It does not replace ROS transport objects. Instead it:

1. registers providers and nodes
2. resolves a target provider for a consumer/client
3. allocates and returns an opaque runtime channel name
4. lets the generated ROS 2 code create its publisher/subscriber/action/service using that channel

## Runtime meta API

The implemented gRPC service is `RobonixRuntime` from:

- `../ridlc/proto/robonix_runtime.proto`

Implemented RPCs:

- `RegisterNode`
- `RegisterStream`
- `ResolveStream`
- `RegisterCommand`
- `ResolveCommand`
- `RegisterQuery`
- `ResolveQuery`

Current allocation strategy:

- stream channels: `/rbnx/ch/s/n<uuid>`
- command channels: `/rbnx/ch/c/n<uuid>`
- query channels: `/rbnx/ch/q/n<uuid>`

These names are intentionally opaque. The namespace and interface name are used for lookup and policy decisions, but are not embedded directly into the final transport endpoint.

## Environment variables

- `ROBONIX_META_GRPC_ADDR`
  - gRPC listen address for the runtime meta API
  - default: `0.0.0.0:50051`
- `ROBONIX_META_GRPC_ENDPOINT`
  - endpoint string returned to clients by `RegisterNode`
  - default: same as `ROBONIX_META_GRPC_ADDR`
- `ROBONIX_WEB_ASSETS_DIR`
  - enables the web UI when set together with `ROBONIX_WEB_PORT`
- `ROBONIX_WEB_PORT`
  - web UI port
- `RUST_LOG`
  - log level control

## Notes

- The gRPC runtime meta API and the legacy ROS `/rbnx/*` APIs coexist in the current server.
- RIDL-generated code should talk to the gRPC runtime meta API, not to the legacy ROS registration services.
- The actual message/service/action types used by generated Python code still come from ROS IDL.
- The legacy `/rbnx/*` side of the server still contains `robonix_sdk`-based service contracts. That is migration debt, not the target interface model.
- Static package metadata belongs in `../ROBONIX_PACKAGE_MANIFEST.md`; live instance state and final channel allocation belong in the runtime registry hosted by `robonix-server`.
