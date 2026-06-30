<!-- SPDX-License-Identifier: MulanPSL-2.0 -->
# robonix testing harness

Deterministic full-stack tests: the **real** robonix stack (atlas, executor,
pilot, primitives, services, skills) runs against a **real** Webots sim, and the
**only** faked component is the VLM. A fake VLM makes planning reproducible —
every task maps to a scripted sequence of capability calls — so a green run
means the real components built, registered, dispatched, and executed correctly
on real sensor data.

This is the harness behind `.github/workflows/testing.yml` (the self-hosted GPU
job). You can also run it by hand against any booted deploy.

## Test layers

| Layer | What it asserts | Driver |
|-------|-----------------|--------|
| **L1 interface / RPC** | each system component's gRPC contract answers correctly | `run_interfaces.py` — atlas via `rbnx caps/contracts/tools/channels/inspect --json`; pilot via `rbnx ask` (SubmitTask) |
| **L2 capability** | each MCP tool is dispatched and succeeds on real sensor data | `run.py` over `scenarios/cap/*.yaml` |
| **L3 flow** | a real multi-step task completes, including **exception injection** + recovery | `run.py` over `scenarios/flow/*.yaml` |
| **L4 logging** | every run leaves a full trace | per-scenario `logs/*.jsonl`, scribe logs, sim/ROS logs — all uploaded as artifacts |

## Pieces

| Path | Role |
|------|------|
| `fake_vlm/server.py` | OpenAI-compatible `/v1/chat/completions` (streaming). Returns scripted **RTDL envelopes**, not OpenAI tool-calls. Stdlib + PyYAML. |
| `scenarios/cap/*.yaml` | single-capability tests. |
| `scenarios/flow/*.yaml` | multi-step task flows, incl. fault injection + recovery. |
| `run.py` | runs cap + flow scenarios; writes `logs/<family>.<name>.jsonl`; asserts dispatch, args, results, round count, fault recovery; prints coverage. |
| `run_interfaces.py` | L1 RPC contract tests; writes `logs/iface.*`. |
| `logs/` | per-run traces (git-ignored). |

## Why RTDL, not OpenAI tool-calls

Pilot does **not** drive the VLM in OpenAI `tool_calls` mode — it hard-errors on
that (`planner.rs`: "VLM returned tool_calls in RTDL mode"). Instead the
assistant returns exactly one JSON object in `content` with four keys:

```json
{
  "content": "narration",
  "rtdl_description": "label",
  "rtdl": {"op":"sequence","op_id":0,"description":"...","children":[
    {"op":"do","op_id":0,"description":"...","cap":"<provider>.<area>_<leaf>","args":{...}}
  ]},
  "task_update": {"goal":"...","success_criterion":"...","status":"in_progress|done"}
}
```

The fake server reads the capability **catalog out of pilot's system prompt**
(`- capability_name: tiago_camera.camera_snapshot`), so scenarios reference a
cap by a short matcher (`camera_snapshot`) and the server resolves it to the
exact provider-qualified name at request time. Round index = number of
`assistant` messages already in history; the server serves `steps[round]` and a
terminal `done` envelope past the end.

## Scenario format

```yaml
name: camera_snapshot
task: "ci-test: capture one rgb camera snapshot from the robot"
steps:
  - status: in_progress
    caps:
      - match: camera_snapshot
        args: {}
  - status: done
expect_contracts:
  - camera/snapshot
expect_args: []
```

- `task` — the prompt submitted to pilot; matched into the user text by
  containment, so it must be distinctive.
- `steps[n].caps[].match` — substring/leaf of an advertised capability name.
- `steps[n].caps[].args` — the args object passed to the `do` node.
- `steps[n].status` — `in_progress` keeps planning; `done` ends the turn.
- `expect_contracts` — substrings that must appear in some dispatched
  `contract_id` (from the `rbnx ask --json` plan event).
- `expect_args` — substrings that must appear in the dispatched `args_json`.
- `allow_leaf_failure` — set true if a non-success leaf result is acceptable.

Flow scenarios (`scenarios/flow/`) add:

- `expect_min_rounds` — require at least N planning rounds (proves multi-step).
- `expect_leaf_failure` — contract substrings whose leaf is *expected* to fail
  (the injected fault). Such a failure is required and is not counted against
  the run; an ordinary unexpected failure still fails the scenario.
- `expect_final_done` — require the turn to end without a FAILED status
  (default true) — i.e. the system recovered from the injected fault.

**Exception injection** is done purely in the scenario: a step calls a real
capability with invalid args (e.g. a required field omitted) so the leaf
genuinely fails, then a later step recovers. This drives pilot's real
replan-on-failure path with no test hooks in production code.

## Run it by hand

Boot a deploy with pilot pointed at the fake VLM, then run the scenarios:

```bash
# 1. fake VLM
python3 testing/fake_vlm/server.py --port 18080 &

# 2. boot the webots deploy against it
cd examples/webots
VLM_BASE_URL=http://127.0.0.1:18080/v1 VLM_API_KEY=fake VLM_MODEL=fake-vlm \
  rbnx boot --no-update-check &

# 3. interface tests, then cap + flow scenarios
python3 ../../testing/run_interfaces.py --server 127.0.0.1:50051
python3 ../../testing/run.py --server 127.0.0.1:50051
```

Both exit non-zero on any failure and write per-test traces under `testing/logs/`.

`run.py` exits non-zero if any scenario fails an assertion or pilot reports
FAILED. The coverage section lists every `contract_id` exercised across the run.

## Adding coverage

The shipped scenarios cover the no/simple-arg RPC tools (camera/lidar snapshot,
memory save+search, voiceprint list, speech tts, map save). To add a tool:

0. Copy an existing `scenarios/*.yaml` and edit it (PyYAML is the only dep).
1. Find its capability — only `[mode] type = "rpc"` contracts become planner
   tools; `topic_in/out` ones are ROS data channels, not callable.
2. Read its `.srv` IDL under `capabilities/lib/<area>/srv/` for the arg fields.
3. Add a scenario; first-run logs from `fake_vlm/server.py` print the live
   catalog **with arg schemas** if a matcher doesn't resolve — use that to fix
   the `match`/`args`.

Tools still needing scenarios: chassis `move`, navigation `navigate`
(both need pose/velocity args), and the `explore` skill (remote package — its
leaf name shows up in the first-boot catalog).
