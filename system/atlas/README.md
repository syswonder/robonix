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
  resolve providers via `QueryCapabilities` (filter by namespace / kind /
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

Sensor declarations may include the existing `CapabilityTimeDescriptor` with
their clock domain, acquisition semantics and pipeline uncertainty. Atlas
validates and retains it with the declaration, returns it in Query and Connect,
and preserves the absence of a known fixed delay. This is static metadata;
it does not ingest timestamps or establish clock synchronization.

Atlas reads reference timestamps and local steady time through the in-process
[`robonix-chronos-clock`](../chronos/clock/README.md) library, without requiring
a running Chronos service. Registration, takeover, heartbeat, and channel
creation return `UNAVAILABLE` before mutating state if reference time cannot
be read, is unavailable, or cannot fit the unsigned timestamp. Unverified Linux
reference time is accepted. Heartbeat eviction uses steady time, so reference
clock steps cannot expire or extend a provider's timeout. A steady read failure
or regression stops Atlas with an error rather than leaving eviction disabled.
`AtlasRegistry::now_ms()` now returns `Result<u64, tonic::Status>`.

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

- `--listen` / `ROBONIX_ATLAS_LISTEN` — gRPC listen address. Default `127.0.0.1:50051`.
- `--capabilities` / `ROBONIX_CAPABILITIES` — one or more contract directories,
  comma-separated. Defaults to the registered repo's `capabilities/`.
- `--log` / `RUST_LOG` — env_logger filter. Defaults to `robonix_atlas=info`.

Heartbeat eviction settings are `ROBONIX_ATLAS_HEARTBEAT_TIMEOUT_MS` (default
90000; 0 disables eviction), `ROBONIX_ATLAS_GC_AFTER_TERMINATED_MS` (default
600000), and `ROBONIX_ATLAS_EVICTION_INTERVAL_MS` (default 10000).

Run `cargo test -p robonix-atlas --test clock_failures` on Linux for real gRPC
and Scribe failure/recovery tests. These compile a test-only `LD_PRELOAD` clock
interposer with `cc` and run in an isolated subprocess; host time is unchanged.

## Out of scope (v0.1)

Atlas does not own transport (see [nexus](../nexus/)), time (see
[chronos](../chronos/)), or persistent logs (see [scribe](../scribe/)).
Atlas's heartbeat eviction half-covers what [vitals](../vitals/) will
own in v0.2.
