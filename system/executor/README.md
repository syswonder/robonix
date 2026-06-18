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

- `sequence`: executes child nodes in order. Stops at the first failed child
  (fail-fast); later siblings are skipped.
- `parallel`: starts one async task per child and waits for all children. A
  failed branch does not cancel sibling branches.
- `do`: dispatches one `CapabilityCall`.

Every `Plan.nodes` entry must include non-empty `op_id` and `description`
fields. Executor copies both fields into every `RtdlNodeState` so callers can
match execution results back to the original RTDL node.

Executor keeps an in-process table of active RTDL plans. The builtin
`cancel_plan` capability can mark a plan as cancelled by `plan_id`; `sequence`
skips children that have not started yet, `parallel` shares the cancellation
state across branches, and the target plan still emits `plan_complete` with
`any_failed=true`.

Executor declares two gRPC contracts:

- `robonix/system/executor/execute`: Pilot submits one `Plan` and receives an
  `RtdlEvent` stream.
- `robonix/system/executor/cancel_all_plans`: cancels every active RTDL plan.

The Execute stream emits `RtdlEvent` messages:

| `event_kind` | Meaning |
|--------------|---------|
| `0` plan_started | Executor began executing the plan |
| `1` node_state | RTDL node state change with `plan_id`, `node_index`, `node_kind`, `op_id`, `description`, `state`, `operator_detail`, and optional `leaf_result` |
| `2` plan_complete | Entire plan finished |

In `parallel` branches, result events are emitted in completion order, not RTDL
order. The final `RtdlPlanComplete` event contains `any_failed=true` when at
least one capability call failed. For `do` nodes, terminal events carry the
concrete `pilot/CapabilityCallResult` in `RtdlNodeState.leaf_result` and leave
`operator_detail` empty. Non-leaf `sequence` and `parallel` nodes emit one
terminal event with `leaf_result` unset and an English `operator_detail`
summary.

## Async capability polling

When a provider registers a sibling `status` contract in the same namespace
(e.g. `robonix/service/navigation/status` alongside `navigate`), executor
detects that per cap call and polls `status` every **2 seconds** until a
terminal state (`SUCCEEDED`, `FAILED`, `CANCELED`, `TIMEOUT`) is reported.

MCP handler requirements for async caps:

- Initial async cap response JSON must include `run_id`.
- Status cap response JSON must include `state` (`PENDING`, `RUNNING`, `SUCCEEDED`,
  `FAILED`, `CANCELED`, `TIMEOUT`, `PAUSED`); optional `detail` for human-readable text.
  Missing `state` is treated as a failed status response.
- When `run_id` is omitted on status/cancel requests, handlers should query the
  most recent run.
- A `cancel` contract in the same namespace is recommended; its absence logs a
  warning but does not block execution.

Sync caps (no sibling `status` contract) complete when the initial MCP call
returns, as before.

## Builtin capabilities

Executor declares builtin MCP capabilities under
`robonix/system/executor/builtin/*`. They execute in-process when Pilot routes a
`do` node back to the Executor provider id.

- `read_file`, `write_file`, `patch_file`, `list_dir`, `run_command`: workspace
  file and shell helpers.
- `cancel_plan`: best-effort cancellation for an in-flight RTDL plan. Args:
  `plan_id` (required) and `wait_ms` (optional, default 5000). Async capability
  calls with sibling `cancel` contracts receive cancel requests; synchronous
  calls already in progress are allowed to return naturally.
