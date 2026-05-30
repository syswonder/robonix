# robonix-executor

`robonix-executor` is the RTDL plan execution service for Robonix. It receives
Pilot `Plan` messages, validates the RTDL node arena, and dispatches each `do`
node to a primitive, service, or skill provider through Atlas.

## Build

From the repo root:

```sh
cargo build -p robonix-executor
```

The crate is part of the top-level Cargo workspace and is built / installed
by `make build` and `make install` respectively.

## Run

Manual launch:

```sh
robonix-executor --atlas 127.0.0.1:50051 --listen 127.0.0.1:50072
```

Important configuration:

- `--atlas` / `ROBONIX_ATLAS_ENDPOINT`: Atlas endpoint. Defaults to `127.0.0.1:50051`.
- `--listen` / `ROBONIX_EXECUTOR_LISTEN`: Executor gRPC listen address. Defaults to `127.0.0.1:50072`.
- `--id` / `ROBONIX_EXECUTOR_PROVIDER_ID`: provider id registered with Atlas. Defaults to `executor`.
- `--log`: env_logger filter. Falls back to `RUST_LOG`, then `robonix_executor=info`.

## RTDL Execution

Executor interprets `Plan.nodes` from `Plan.root_index`:

- `sequence`: executes child nodes in order. A failed child marks the batch as failed, but later children still run.
- `parallel`: starts one async task per child and waits for all children. A failed branch does not cancel sibling branches.
- `do`: dispatches one `CapabilityCall`.

The Execute stream emits `CapabilityCallEvent.started` and
`CapabilityCallEvent.result` for each `do` node. In `parallel` branches, result
events are emitted in completion order, not RTDL order. The final
`BatchComplete` event contains `any_failed=true` when at least one capability
call failed.
