# robonix-atlas

**robonix-atlas** is the **control plane** for Robonix: **service registration**, **discovery**, **interface** metadata, **channel negotiation** across **transports**, and **skill** (SKILL.md) queries for agents.

It implements the `RobonixRuntime` service from [`proto/robonix_runtime.proto`](../../proto/robonix_runtime.proto) (also vendored under `robonix-codegen` for codegen).

## Build

```bash
cargo build -p robonix-atlas
```

## Run

Listen/advertise addresses are controlled by environment variables:

- **`ROBONIX_META_GRPC_ADDR`** — Bind address for the gRPC server (default `0.0.0.0:50051`).  
- **`ROBONIX_META_GRPC_ENDPOINT`** — Endpoint string returned to clients (default: same as listen addr); set when clients must use a different host/port than the bind address.  
- **`ROBONIX_DATA_PLANE_HOST`** — Hostname prepended to allocated **mcp** / **grpc** ports in `DeclareInterface` (default `localhost`). Set to a Docker service name (e.g. `bridge`) when other containers must reach the MCP HTTP server by DNS.  

```bash
export ROBONIX_META_GRPC_ADDR=0.0.0.0:50051
cargo run -p robonix-atlas
```

## Main gRPC API

Typical calls: **`RegisterNode`**, **`DeclareInterface`**, **`QueryNodes`**, **`NegotiateChannel`**, **`QueryAllSkills`** (plus `QuerySkillMd`, `ReleaseChannel`, `InspectRuntime`).
