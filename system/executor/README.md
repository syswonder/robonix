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

Result verification rules live in the deployment manifest's `system.executor`
block. A rule is optional; calls with no matching rule keep their original
result. For example:

```yaml
system:
  executor:
    verification:
      overlap: true                     # optional; defaults to false
      rules:
        - target_contract_id: robonix/service/navigation/navigate
          target_provider_id: simple_nav  # optional provider-specific override
          verifier_provider_id: scene_verifier
          verifier_args:
            scene_provider_id: scene
```

An exact `target_provider_id` + `target_contract_id` rule wins over a
contract-only rule. Duplicate rules at the same specificity are rejected at
startup. `verifier_args` must be a JSON/YAML object and is forwarded unchanged
inside the verifier request. `overlap` applies to the entire verification
configuration and defaults to `false`, preserving synchronous verification.

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

When a provider registers required async sub-contracts for a capability
(e.g. `robonix/service/navigation/navigate/status` and
`robonix/service/navigation/navigate/cancel` for
`robonix/service/navigation/navigate`), executor detects that per cap call and
polls `status` every **2 seconds** until a terminal state (`SUCCEEDED`,
`FAILED`, `CANCELED`, `TIMEOUT`) is reported.

MCP handler requirements for async caps:

- Initial async cap response JSON must include `run_id`.
- Status cap response JSON must include `state` (`PENDING`, `RUNNING`, `SUCCEEDED`,
  `FAILED`, `CANCELED`, `TIMEOUT`, `PAUSED`); optional `detail` for human-readable text.
  Missing `state` is treated as a failed status response.
- When `run_id` is omitted on status/cancel requests, handlers should query the
  most recent run.
- Every async cap must register both `<contract_id>/status` and
  `<contract_id>/cancel`. Registering only one is a provider configuration error.

Sync caps (no `<contract_id>/status` and `<contract_id>/cancel` pair) complete when the initial MCP call
returns, as before.

## Result verification

After a configured capability reaches `SUCCEEDED`, Executor calls
`robonix/service/verifier/verify` on the rule's `verifier_provider_id`. The
request carries the original call id plus an opaque JSON payload containing the
target provider, contract, RTDL node description, arguments, output, and the
configured `verifier_args`.

Verification is fail-closed and has a fixed 60-second timeout. `passed=true`
preserves the original successful result. `passed=false` changes the original
node to `FAILED` with `result verification failed: ...`; an unavailable,
timed-out, or malformed verifier response changes it to `FAILED` with
`result verification unavailable: ...`. Failed, cancelled, and timed-out
target calls are never verified. Executor emits only one terminal node event,
after verification has finished when `overlap=false`.

With `overlap=true`, a matched successful capability call first emits
`VERIFYING` with its result and continues the RTDL tree while verification runs
in the background. Sequence and parallel operators with verifying descendants
also emit `VERIFYING` without blocking their parents. Every leaf and operator
later emits exactly one final `SUCCEEDED`, `FAILED`, or `CANCELED` state after
its verification dependencies finish. Verification never cancels or changes
the execution of sequence or parallel siblings. Executor waits for every final
state before `plan_complete` and includes verification failures in `any_failed`.

`VERIFYING` is also the cancellation boundary for a leaf: its capability call
has already completed, so `cancel_plan` does not cancel that leaf or replace its
final verification outcome. The verifier continues running and the leaf later
becomes `SUCCEEDED` when verification passes or `FAILED` when verification
fails. Cancellation still applies to nodes that are running or have not
started, and sequence/parallel operators and the plan may therefore finish as
`CANCELED` even when an already-verifying leaf finishes successfully.

## Builtin capabilities

Executor declares builtin MCP capabilities under
`robonix/system/executor/builtin/*`. They execute in-process when Pilot routes a
`do` node back to the Executor provider id.

- `read_file`, `write_file`, `patch_file`, `list_dir`, `run_command`: workspace
  file and shell helpers.
- `read_capability_doc`: read the generated capability documentation for a
  registered contract so Pilot can inspect the abstract interface before use.
- `cancel_plan`: best-effort cancellation for an in-flight RTDL plan. Args:
  `plan_id` (required) and `wait_ms` (optional, default 5000). Async capability
  calls receive cancel requests through `<contract_id>/cancel`; synchronous
  calls already in progress are allowed to return naturally.
- `get_all_plans`: list every in-flight RTDL plan (no args). Returns each
  `plan_id` with a short `description` of the task (the root node's description,
  which carries the model's `rtdl_description`), `op_count`, `cancelled`, and the
  number of armed `stop_points`. The plan-list level of inspect-then-act:
  discover running plans, then drill in with `get_plan_status`.
- `get_plan_status`: inspect an in-flight RTDL plan. Args: `plan_id` (required).
  Returns each op as JSON with `op_id`, `kind`, `description`, current `state`
  (`pending`/`running`/`verifying`/`succeeded`/`failed`/`canceled`/`timeout`/`paused`) and
  any armed `stop_point`. Lets the LLM read a running plan's structure + live
  progress before issuing a `stop_plan_at` / `cancel_plan` (inspect first, then
  act). Errors if the plan is not active (stale/wrong id or already finished) —
  it is a query, so "not found" is an error; use `get_all_plans` to check which
  plans are still running.
- `stop_plan_at`: arm a stop point on an in-flight RTDL plan — when execution
  reaches the node with the given `op_id`, the **whole plan is cancelled** (same
  teardown as `cancel_plan`). Args: `plan_id` (required), `op_id` (required), and
  `when` (optional, `on_complete` default): `on_enter` cancels the moment the op
  is reached, before its call runs (the op never executes); `on_complete` cancels
  right after the op's call finishes. `op_id`s are the per-node identifiers
  surfaced in `RtdlNodeState` events. Setting a stop point on a plan that is no
  longer active is a success no-op; an `op_id` that never executes never fires.
