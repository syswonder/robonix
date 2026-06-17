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

The wire schema lives in [`proto/atlas.proto`](proto/atlas.proto).

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

## Out of scope (v0.1)

Atlas does not own transport (see [nexus](../nexus/)), time (see
[chronos](../chronos/)), or persistent logs (see [scribe](../scribe/)).
Atlas's heartbeat eviction half-covers what [vitals](../vitals/) will
own in v0.2.
