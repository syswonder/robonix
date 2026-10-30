# robonix-executor

Executes Pilot's RTDL plans: validates input, schedules `sequence` / `parallel`
nodes, and runs each `do` capability call. External calls resolve through Atlas
and use MCP; builtin capabilities run in-process.

## Build and run

From the repository root, with Atlas running:

```sh
cargo build -p robonix-executor
./target/debug/robonix-executor --atlas 127.0.0.1:50051 --listen 127.0.0.1:50061
```

| Flag | Environment variable | Default / purpose |
|---|---|---|
| `--atlas` | `ROBONIX_ATLAS_ENDPOINT` | Atlas at `127.0.0.1:50051` |
| `--listen` | `ROBONIX_EXECUTOR_LISTEN` | gRPC on `127.0.0.1:50061` |
| `--id` | `ROBONIX_EXECUTOR_PROVIDER_ID` | Provider id `executor` |
| `--config` | `ROBONIX_CONFIG_PATH` | Optional YAML: `atlas_endpoint`, `listen`, `id`, `verification` |
| `--config-json` | — | `system.executor` manifest block passed by `rbnx` |

Connection settings use CLI > flag environment variable > YAML > default.
`ROBONIX_ATLAS` is an additional fallback before YAML. Manifest verification
settings override YAML verification settings.

Logging uses Scribe: `system.executor.log` (via `--config-json`) overrides
`SCRIBE_FILE_LEVEL` (default `info`); `SCRIBE_CONSOLE_LEVEL` defaults to `warn`.
The standalone `--log` flag is accepted but currently not applied.

## Execution and control

All gRPC contracts are under `robonix/system/executor/`:

| Contract | Purpose |
|---|---|
| `execute` | Submit a complete `Plan`; receive an event stream |
| `execute_stream` | Submit a plan incrementally; receive the same events |
| `control_plan` | `cancel` one `plan_id` or `cancel_all`; `wait_ms=0` uses 5000 ms |
| `list_active_plans` | Query active plans and node states |
| `cancel_all_plans` | Compatibility interface for canceling all plans |
| `get_health` | Report Executor health |

Both execution RPCs share one scheduler:

- `sequence` runs children in order and skips later children after a failure;
  `parallel` runs branches concurrently without canceling siblings on failure.
- Input requires a nonempty plan ID, unique op/call IDs, nonempty node
  descriptions, object-valued call arguments, at most 256 nodes and 32 levels.
  Complete plans must form one tree; active plan IDs cannot be reused.
- Events are `plan_started`, `node_state`, and `plan_complete`. Node events
  retain node identity; leaf results carry the capability output. Parallel
  results arrive in completion order. Failures and cancellation set `any_failed`.
- Cancellation stops new work and requests cancellation of async calls.
  Synchronous calls may finish naturally. Active state is in memory and is
  removed after running work and verification finish.

Incremental input follows `START → NODE / CLOSE → END`. Execution can begin
before `END`; operators close innermost first. Invalid input returns
`invalid_argument`; capability failures use node events. Input loss before
`END` or response disconnection triggers cancellation and cleanup.
See the [stream protocol](../../capabilities/lib/executor/msg/ExecuteStreamInput.msg),
[event IDL](../../capabilities/lib/executor/msg/RtdlEvent.msg), and
[contracts](../../capabilities/system/executor/) for field definitions.

## Async calls and verification

Async MCP capabilities register both `<contract_id>/status` and
`<contract_id>/cancel`; registering only one is an error. Executor polls status
every 2 seconds until `SUCCEEDED`, `FAILED`, `CANCELED`, or `TIMEOUT`.
Status responses provide `state` and optional `detail`. The initial response's `run_id`
identifies subsequent status/cancel requests; without it, the provider must
resolve the current or latest run. Calls without either sub-contract complete
when their initial response returns.

Verification is optional. Configure it in the deployment manifest:

```yaml
system:
  executor:
    verification:
      overlap: false
      rules:
        - target_contract_id: robonix/service/navigation/navigate
          target_provider_id: simple_nav  # optional
          verifier_provider_id: scene_verifier
          verifier_args:
            scene_provider_id: scene
```

Provider-specific rules override contract-only rules; duplicate rules are
rejected. `verifier_args` must be an object and is forwarded unchanged.
Only successful calls with a matching rule invoke `robonix/service/verifier/verify`.
A failed verdict, unavailable verifier, malformed response, or 60-second timeout
makes the node fail.

- `overlap: false` (default): wait for verification before completing the node.
- `overlap: true`: emit `VERIFYING` and continue execution; wait for verification
  before final node/operator states and `plan_complete`. Verification failures
  do not stop later siblings. Canceling a plan does not cancel a leaf already
  in `VERIFYING` or overwrite its verification result.

## Builtin capabilities

Under `robonix/system/executor/builtin/`:

- `read_file`, `write_file`, `patch_file`, `list_dir`, `run_command`: workspace
  file and shell helpers.
- `read_capability_doc`: read a registered provider's documentation so Pilot can
  inspect the abstract interface before use. Args: `provider_id` (required).
  Returns the provider's `CAPABILITY.md` when it registered one, followed by the
  full description of each capability it offers. The planning catalog prints
  only the opening paragraph of a description, so this is where the rest of a
  long one is read; serving descriptions as well as the manual means providers
  that never wrote a `CAPABILITY.md` still answer.
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


## RTDL conditions and loops

RTDL supports `if` (`condition`, `then`, optional `else`), `repeat` (`count`, `body`), and bounded `while` (`condition`, `max_iterations`, `body`). A `do.bind` stores successful, verified output in the plan's execution context. Reference it with `{"var":"name","path":["field",0]}` in arguments or conditions. Conditions accept booleans and `{"op":"eq|ne|lt|le|gt|ge|and|or|not","args":[...]}` expressions. Missing values and non-boolean conditions fail explicitly.

Variables are isolated per plan; loops update the same binding each iteration. Bound calls wait for verification even under overlap mode, and loop iterations wait for all descendant verification. Parallel siblings must be independent; their results may be consumed after the join. Repeat counts are 0–10000 and while requires a 1–10000 iteration limit. A while condition still true at the limit fails. See [the protocol](../pilot/rtdl_protocol.md) for the complete grammar.

Control execution writes `[executor/rtdl]` info records for the complete execution path. Plan records include session, round, tree and call counts, final state and elapsed time. Node records include the node kind and description; `do` records include the resolved arguments, provider, contract, binding name, final state and elapsed time. Binding records report the stored name, JSON value type and byte size. Control nodes also report their configured limit, condition result, selected branch, iteration start and completion, and exit reason. Condition evaluation errors and exhausted while limits produce warning records. Read these with `rbnx logs -t executor`; previews are bounded and execution-context values are not dumped.

## Source map

| File | Responsibility |
|---|---|
| [main.rs](src/main.rs), [config.rs](src/config.rs) | Startup, registration, configuration |
| [grpc_server.rs](src/grpc_server.rs) | Execution, control, query, and health RPCs |
| [plan_input.rs](src/plan_input.rs) | Plan validation and incremental construction |
| [plan_scheduler.rs](src/plan_scheduler.rs) | Scheduling and cleanup |
| [capability_execution.rs](src/capability_execution.rs) | One `do` call's execution and verification flow |
| [active_plans.rs](src/active_plans.rs) | Active state, queries, and cancellation |
| [rtdl_events.rs](src/rtdl_events.rs), [pb.rs](src/pb.rs) | Event helpers and generated protobuf modules |
| [verification.rs](src/verification.rs), [dispatch/](src/dispatch/) | Result verification and capability dispatch |

## Validation

```sh
cargo test -p robonix-executor --all-targets
cargo build -p robonix-atlas -p robonix-executor -p robonix-pilot
python3 scripts/test_execute_stream.py
```

The cross-process test uses a local fake VLM, requires `grpcio` and
`grpcio-tools`, and retains logs under `/tmp`.
