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
        contract_id="robonix/primitive/lidar/lidar",
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

`CapabilityTimeDescriptor` is optional on `declare_capability`,
`declare_ros2_topic` and `create_publisher`. It binds a sensor capability to
its existing clock domain and describes acquisition semantics and pipeline
uncertainty. Rebuild package stubs and Atlas together before declaring time
metadata; a declaration fails explicitly if the local schema lacks the field.
Query and Connect with older response bindings expose `time_descriptor=None`,
which means the metadata is unavailable to that client. Regenerate bindings to
consume metadata stored by Atlas.

Duplicate declarations carrying metadata raise `AlreadyExists`. Omitting
`time_descriptor` on a repeat retains the legacy success behaviour and keeps
the stored descriptor; it does not clear metadata. Unregister and register the
provider again to replace its declarations.

Contract descriptors also expose `llm_callable`. It defaults to `true`; a
contract may set it to `false` to keep its implementations out of Pilot's
model-facing catalog without changing registration, discovery, or calls.

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

- **`ATLAS`** — module-level singleton client (`ATLAS.register_primitive` / `register_service` / `register_skill`, `ATLAS.declare_capability`, `ATLAS.query_primitives` / `query_services` / `query_skills`, `ATLAS.find_capability`, `ATLAS.connect_channel`, ...)
- **`Primitive` / `Service` / `Skill`** — provider base classes with `on_init` / `on_activate` / `on_deactivate` / `on_shutdown` decorators
- **`@provider.provides_grpc(contract)` / `@provider.provides_mcp(contract)`** — Layer-2 sugar for typed handlers
- **`Ok` / `Err` / `Deferred`** — lifecycle return values
- **`mcp_contract`** — standalone FastMCP decorator (use when you manage your own FastMCP app, e.g. in `scene_service`)

The Atlas wire protocol (atlas_pb2 / atlas_pb2_grpc) is pre-generated and bundled in the wheel. Per-contract stubs (`robonix_contracts_pb2`, MCP typed dataclasses) are generated **per deployment** by `rbnx codegen` against your contract TOMLs — `robonix-api` automatically picks them up from `<pkg>/rbnx-build/codegen/` at runtime.

## LLM catalog

`robonix_api.llm` is the shared model catalog and resolver used by
Scene, Memory, Memsearch, the VLM Verifier, CI scripts and other consumers. It
answers "which model endpoint should this request use?" from a declarative
catalog and leaves the provider call to the consumer's own client. The
`metrics` helper appends one JSON line per model request for usage accounting.
It does not proxy traffic, hold keys or track health. See
[`robonix_api/llm/README.md`](robonix_api/llm/README.md) for the catalog format,
the Python API and the deployment variables.

## Consumer channels

`ATLAS.connect_capability(...)` and the `Primitive` / `Service` / `Skill`
`connect_capability(...)` methods are deprecated and will be removed in a future
release. Migrate to `connect_channel(...)` and use the Channel methods below
instead of creating protocol clients from `channel.endpoint`. The old methods
currently retain their endpoint-only behavior for all transports.

Run `rbnx codegen -p <package> --mcp` before using `connect_channel`. It uses
the generated service type mappings to encode requests and decode responses:

```python
from robonix_api import ATLAS, Transport
from map_mcp import GetMode_Request

with ATLAS.connect_channel(
    consumer_id="my_consumer", provider_id="mapping",
    contract_id="robonix/service/map/get_mode", transport=Transport.MCP,
) as channel:
    response = channel.call(GetMode_Request())
    print(response.mode)
```

`Primitive`, `Service`, and `Skill` share the provider methods below.

| API | Purpose |
|---|---|
| `ATLAS.connect_channel(*, consumer_id, provider_id, contract_id, transport)` | Prepare a Channel and its communication resources. |
| `provider.connect_channel(provider, contract_id, transport)` | Accept a discovered capability or provider record; use this instance's ID as the consumer. |
| `channel.call(request, *, timeout=None)` | Submit one RPC execution; gRPC streaming calls take or return iterators. |
| `channel.publish(message)` | Publish repeatedly to a ROS 2 `topic_in`. |
| `channel.subscribe(callback)` | Subscribe once to a ROS 2 `topic_out`. |
| `channel.status(*, timeout=30.0)` / `channel.cancel(*, timeout=30.0)` | Query or cancel an async execution using its `run_id`. |
| `channel.close(*, timeout=5.0)` / `with channel:` | Close communication and release the allocation after confirming termination; failed close raises and retains the handle. |
| `ATLAS.get_channel(channel_id)` | Recover a retained Channel, or return `None` if absent. |
| `channel.run_id` / `channel.raw_response` | Inspect the execution ID or decoded business response. |

Each RPC channel allows one `call`; create a new channel for another execution.
`call` accepts a generated request or a dictionary and returns the generated
response, subject to the streaming forms below. These methods are synchronous;
asyncio callers should use a worker thread. MCP sessions share a client-owned
event loop and connect directly to providers without shell proxy settings.

For long tasks, `status()` and `cancel()` resolve companion capabilities and
pass `run_id` automatically. Missing legacy IDs produce empty companion requests.
`close(timeout=5)` requests cancellation and polls every two seconds until a
terminal state is confirmed. Cancellation acceptance, local timeouts, invalid
states, and disconnections do not release active work. Failed close reports the
channel ID; `ATLAS.get_channel(id)` returns the retained handle for another
status or close attempt. A lost start response cannot be addressed without a
confirmed run ID. There is no automatic business-request retry or persistence
across consumer process restarts.

The client holds at most 1024 allocations (`ROBONIX_ATLAS_CHANNEL_LIMIT` overrides
the limit), reserving two for companion calls.
Companions can also use unused business slots. Protocol failures without a
response retain their own allocations; capacity exhaustion is reported.
Successful close returns the slot.
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
