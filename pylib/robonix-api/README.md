# robonix-api

Python client library for writing [Robonix](https://github.com/syswonder/robonix) capability providers — primitives, services, and skills — and talking to the Atlas registry.

> Robonix is an embodied-AI operating system. `robonix-api` is the **client-side helper**: it gives you `Primitive` / `Service` / `Skill` classes that handle Atlas registration, the Driver lifecycle gRPC server, and thin wrappers over `rclpy` / `FastMCP` / `grpcio`. The Atlas server itself is a separate Rust binary (`robonix-atlas`, see the main repo).

## Install

```bash
pip install robonix-api
```

Optional extras:

```bash
pip install "robonix-api[codegen]"   # adds grpcio-tools, only needed if you run `rbnx codegen` from Python
```

ROS 2 (`rclpy`, `sensor_msgs`, `geometry_msgs`, etc.) is **not** a pip dependency — install ROS 2 Humble via `apt` on the host. `robonix-api`'s ROS helpers import `rclpy` lazily, so the rest of the library works even without a ROS install.

## Hello, primitive

```python
from robonix_api import Primitive, Ok, Deferred

primitive = Primitive(
    id="my_lidar",
    namespace="robonix/primitive/lidar",
)

@primitive.on_init
def init(cfg: dict):
    topic = cfg.get("lidar_topic", "/scan")
    if not primitive.wait_for_topic(topic, "LaserScan", 30.0):
        return Deferred(f"no LaserScan on {topic} yet")
    primitive.create_publisher(
        contract_id="robonix/primitive/lidar/lidar2d",
        topic=topic,
        msg_type="LaserScan",
    )
    return Ok()

if __name__ == "__main__":
    primitive.run()
```

That's a complete Robonix primitive — registers with Atlas, serves the Driver lifecycle, waits for the upstream topic, declares a ROS 2 capability for downstream consumers, and blocks on SIGTERM.

`namespace` declares the provider's primary contract grouping. Regular
contracts normally use that prefix. Declaring another contract is allowed;
robonix-api and Atlas emit a visible warning unless the contract TOML marks it
as `cross_namespace = true`. The warning is diagnostic and never blocks boot,
discovery, or calls.

Current codegen provides the shared `robonix/lifecycle/driver` contract, and
package authors normally omit Driver from the manifest: rbnx and robonix-api
select/register shared automatically. Explicit shared selection remains valid,
and an explicitly selected exact `<namespace>/driver` remains compatible. A
legacy manifest may use current shared runtime stubs while it is migrated, but
a shared selection never downgrades to legacy. Every provider must expose
exactly one lifecycle Driver; zero, multiple, unrelated, and partial generated
services fail with rebuild/migration guidance instead of promoting the provider
to `ACTIVE`. An omitted lifecycle *handler* is different: it logs a warning and
completes that transition as an `Ok` no-op, so the Driver can still walk the
provider through `REGISTERED → INACTIVE → ACTIVE`.

Managed omission and explicit shared selection request
`robonix/lifecycle/driver` with the compatibility marker cleared. An explicit
exact legacy selection sets `ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK=1`;
despite its historical name, the marker permits only a legacy manifest whose
legacy generated service pair is wholly absent to use the complete shared
pair. If the selected pair is partial, neither permitted pair exists, or Driver
declaration fails, startup is fatal. Direct launches with no selection use
shared only; direct exact-legacy launches use legacy when that service exists.

## What's in the box

- **`ATLAS`** — module-level singleton client (`ATLAS.register`, `ATLAS.find_primitive`, `ATLAS.connect`, ...)
- **`Primitive` / `Service` / `Skill`** — provider base classes with `on_init` / `on_activate` / `on_deactivate` / `on_shutdown` decorators
- **`@provider.provides_grpc(contract)` / `@provider.provides_mcp(contract)`** — Layer-2 sugar for typed handlers
- **`Ok` / `Err` / `Deferred`** — lifecycle return values
- **`mcp_contract`** — standalone FastMCP decorator (use when you manage your own FastMCP app, e.g. in `scene_service`)

The Atlas wire protocol (atlas_pb2 / atlas_pb2_grpc) is pre-generated and bundled in the wheel. Per-contract stubs (`robonix_contracts_pb2`, MCP typed dataclasses) are generated **per deployment** by `rbnx codegen` against your contract TOMLs — `robonix-api` automatically picks them up from `<pkg>/rbnx-build/codegen/` at runtime.

## Provider network binding

Provider lifecycle gRPC, user gRPC, and MCP servers keep the compatible
all-interface default (`0.0.0.0`). A deployment that must make every provider
local-only sets this before provider construction:

```bash
export ROBONIX_PROVIDER_BIND_HOST=127.0.0.1
export ROBONIX_ADVERTISE_HOST=127.0.0.1
```

`ROBONIX_PROVIDER_BIND_HOST` must be an IPv4 address literal and controls every
server socket owned by `robonix-api`. When it is a non-wildcard address and no
advertise override is set, that same address is advertised to Atlas
automatically. Cross-host deployments should retain `0.0.0.0` and set
`ROBONIX_ADVERTISE_HOST` to the provider address reachable by consumers.

## Versioning

`robonix-api` tracks the Robonix v0.1.x series. Wire format and public API are frozen within v0.1.x. Pre-release builds (`0.1.0rc*`) are published to Test PyPI first; stable releases to PyPI.

The Atlas server (Rust crate `robonix-atlas`) must be on a compatible minor version. Mixing `robonix-api 0.1.x` with an Atlas server on a different minor version is unsupported.

## Links

- [Robonix monorepo](https://github.com/syswonder/robonix) — Atlas server, `rbnx` CLI, examples, full developer guide
- [Issues](https://github.com/syswonder/robonix/issues)

## License

Mulan PSL v2.
