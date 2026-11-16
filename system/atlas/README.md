# robonix-atlas

`robonix-atlas` is Robonix's capability-discovery component — one of the
12 system components in the EAIOS whitepaper. It holds the catalog of
every running primitive / service / skill and the contract interfaces
each one provides.

## What it owns

- **Registration**: each capability provider calls
  `RegisterPrimitive` / `RegisterService` / `RegisterSkill` on startup,
  declares each interface with `DeclareCapability(contract_id, transport,
  endpoint, params)`, and pings `Heartbeat` while it stays alive.
- **Discovery**: consumers (pilot, executor, scene, downstream services)
  resolve providers via `Query` (filter by namespace / kind /
  capability id) without needing to know who is running where.
- **Channels**: `ConnectCapability` opens a tracked consumer→provider
  channel; `DisconnectCapability` releases it. Channels make
  `rbnx channels` / `rbnx inspect` useful for operations.
- **Lifecycle state**: every provider's `LifecycleState`
  (REGISTERED / INACTIVE / ACTIVE / TERMINATED / ERROR) is tracked here,
  pushed via `SetLifecycleState`. Heartbeat-driven eviction marks lapsed
  providers TERMINATED.
- **Contract registry**: contract TOMLs under the repo's `capabilities/`
  are loaded at startup so `QueryContract` returns IDL metadata to any
  client.

Contract TOMLs may set `llm_callable = false` under `[contract]` when an
interface is intended for system orchestration rather than direct planning
model use. Atlas only publishes this metadata; registration, discovery, and
calling remain available to other consumers. Omitted values default to `true`.

The wire schema lives in [`proto/atlas.proto`](proto/atlas.proto).

Sensor declarations may include `CapabilityTimeDescriptor` with their clock
domain, acquisition semantics and pipeline uncertainty. Atlas validates and
retains it with the declaration, returns it in Query and Connect, and preserves
the absence of a known fixed delay. Rebuild protobuf bindings to use these fields.

Python declarations carrying time metadata propagate `AlreadyExists` for a
duplicate capability instead of reporting success with the old descriptor.
Unregister and register the provider again before replacing its declarations.
Repeating a declaration without `time_descriptor` retains legacy success and
keeps the stored descriptor. Old Python Query and Connect response bindings
expose `time_descriptor=None`; regenerate them to consume stored metadata.

`AtlasRegistry::connect` returns `(channel_id, endpoint, params, time_descriptor)`;
direct registry callers must handle the fourth value. The existing
`AtlasClient::connect_capability` keeps its three-value return; callers that
need metadata use `connect_capability_with_time`.

The `Atlas Python metadata` CI job builds Atlas, generates Python bindings from
this schema, and runs `pylib/robonix-api/tests/test_atlas_time_descriptor.py`
against an Atlas process. The tests cover provider declaration, Query, Connect,
channel cleanup, optional delay presence, the old-stub declaration error, and
reads with old response types.

## Client execution channels

`AtlasClient::connect_channel::<C>(consumer_id, provider_id, contract_id, transport)`
prepares one execution using a compile-time contract binding. `Mcp` uses JSON
requests and responses. gRPC bindings use generated protobuf messages; no
reflection, descriptor files, or Cargo feature is needed. Business traffic goes
directly to the provider. `AtlasClient::connect_capability(...)` is deprecated
and will be removed in a future release. It currently retains its tuple return
value. Migrate to `connect_channel::<C>(...)` and replace manual MCP or gRPC
client setup with `channel.call(...)`.

| API | Purpose |
|---|---|
| `atlas.connect_channel::<C>(consumer_id, provider_id, contract_id, transport).await` | Prepare a `Channel<C>` using a contract binding. |
| `channel.call(request).await` | Submit one execution with the binding's input and output types. |
| `channel.status().await` / `channel.cancel().await` | Query or cancel async work; resolve companions and pass `run_id` automatically. |
| `channel.close().await` | Confirm termination and release resources, waiting up to five seconds for active work to end. |
| `channel.close_with_timeout(duration).await` | Set the termination wait using `std::time::Duration`. |
| `atlas.get_channel::<C>(channel_id)` | Return `Result<Option<Channel<C>>>` for a retained allocation; the binding must match. |
| `channel.run_id()` / `channel.is_async()` | Inspect the execution ID or whether status/cancel companions are present. |
| `channel.is_terminal()` / `channel.is_closed()` | Check confirmed execution completion or allocation release. |

Include codegen's `contract_proto_modules.rs` for protobuf messages and their
Channel bindings in the existing IDL modules. For example:

```rust,ignore
let channel = atlas.connect_channel::<pb::map::ServiceMapGetMode>(
    consumer_id, provider_id, "robonix/service/map/get_mode", Transport::Grpc,
).await?;
let response = channel.call(pb::map::GetModeRequest {}).await?;
channel.close().await?;
```

All four gRPC forms use `call`: unary takes and returns a message; server
streaming returns a stream; client streaming takes a boxed request stream;
bidirectional streaming takes and returns streams. Output streams yield
`Result<Message>` with a bounded 16-message buffer. gRPC `topic_out` and
`topic_in` use server and client streaming respectively. Empty protobuf
messages use `()`. ROS 2 execution channels remain available through Python.

The shared `channel::validate_contract::<C>` function validates contract metadata
before communication; bindings supply types, metadata, and message adapters.
Bindings with a fixed contract ID require an exact match by default. The generated
`LifecycleDriver` binding also accepts legacy `*/driver` contracts with the same
Driver IDL, using their Atlas service routes.
`Mcp` leaves its ID empty and uses the contract selected at connection time.
Async `status()` and `cancel()` return generated protobuf responses for gRPC and
JSON for MCP.

A channel starts one execution. For async `rpc` capabilities, both `/status` and
`/cancel` companions must support the selected transport. Each companion gets its
own allocation. An MCP tool error on the start call lets `close()` release the
channel without canceling a run. `close()` waits up to five seconds for confirmed
completion. `close_with_timeout(Duration)` changes the wait. Async close cancels
and polls status every two seconds. Stream close stops sending and drains
responses until the server ends the stream. A local timeout, lost connection, or
cancellation acceptance does not confirm termination. If the server never ends a stream,
close fails and retains its allocation rather than canceling it locally and
claiming completion.

Errors report retained channel IDs. `get_channel::<C>(id)?` recovers the typed handle from
any clone of the same client. `is_terminal()` reports confirmed completion and
`is_closed()` reports release. Dropping a waiter does not abandon the execution,
and business requests are never automatically retried.

Each client holds at most 1024 allocations (`ROBONIX_ATLAS_CHANNEL_LIMIT` overrides
the limit), reserving two for companions.
Setup failures clean up allocations; failed cleanup remains recoverable by ID.
Execution records are local to the consumer process and do not survive restart.

## Namespace diagnostics

A provider's namespace is its primary grouping for discovery and operator
output. Domain contracts normally use that prefix, but namespace is not an
authorization boundary: Atlas accepts and serves a capability whose contract
id is outside the provider namespace.

Contract TOMLs may set `cross_namespace = true` for shared framework contracts
that are intentionally implemented across provider domains. Other mismatches
are logged and returned as `Capability.namespace_mismatch`; `rbnx caps -v`
shows the warning without changing provider lifecycle state or callability.
Existing per-namespace contracts, including current `*/driver` contracts,
remain supported.

## Build

From the repo root:

```sh
cargo build -p robonix-atlas
```

The crate is part of the top-level Cargo workspace; `make install` puts
the binary at `~/.cargo/bin/robonix-atlas`.

## Run

Atlas runs as a long-lived process; in normal flow `rbnx boot` starts it
as the first system component. Manual launch:

```sh
robonix-atlas --listen 127.0.0.1:50051 \
              --capabilities /path/to/robonix/capabilities
```

Configuration:

- `--listen` / `ROBONIX_ATLAS_LISTEN` — gRPC listen address. Default
  `0.0.0.0:50051`, so Atlas is reachable from other hosts. Atlas has no
  authentication: any client that can reach the port can register, query, and
  change lifecycle state. Bind it to `127.0.0.1` or firewall the port when
  other hosts do not need it.
- `--capabilities` / `ROBONIX_ATLAS_CAPABILITIES` — one or more contract directories,
  comma-separated. When unset, Atlas uses `$ROBONIX_SOURCE_PATH/capabilities`;
  when that is unset too, the contract registry is empty and `QueryContract`
  returns `found=false`.
- `log` — Scribe log-file level (`debug`/`info`/`warn`/`error`), read from the
  `log` key of `--config-json`, which `rbnx` fills from the manifest. Falls back
  to `SCRIBE_FILE_LEVEL`, then `info`. A standalone `--log` flag is accepted but
  not applied.

Atlas reads reference timestamps and local steady time through the in-process
[`robonix-chronos-clock`](../chronos/clock/README.md) library, without requiring
a running Chronos service. Registration, takeover, heartbeat, and channel
creation return `UNAVAILABLE` before mutating state if reference time cannot
be read, is unavailable, or cannot fit the unsigned timestamp. Unverified Linux
reference time is accepted. Heartbeat eviction uses steady time, so reference
clock steps cannot expire or extend a provider's timeout. A steady read failure
or regression stops Atlas with an error rather than leaving eviction disabled.
`AtlasRegistry::now_ms()` now returns `Result<u64, tonic::Status>`.

Heartbeat eviction settings are `ROBONIX_ATLAS_HEARTBEAT_TIMEOUT_MS` (default
90000; 0 disables eviction), `ROBONIX_ATLAS_GC_AFTER_TERMINATED_MS` (default
600000), and `ROBONIX_ATLAS_EVICTION_INTERVAL_MS` (default 10000).

Run `cargo test -p robonix-atlas --test clock_failures` on Linux with `cc` for
clock failure, recovery and heartbeat expiry tests over real gRPC connections.
Fault injection runs in a subprocess and does not change host time.
