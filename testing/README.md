<!-- SPDX-License-Identifier: MulanPSL-2.0 -->
# robonix testing harness

Deterministic integration tests: the Robonix runtime boots against a real
Webots sim and exercises the normal service paths. The VLM endpoint is
scripted so RTDL planning is reproducible, and audio uses ALSA's `null`
device because the runner has no physical mic or speaker. A green run means
the checked-out code built, booted, registered capabilities, dispatched RTDL
plans, and produced the expected runtime outputs on the simulated robot.

This is the harness behind `.github/workflows/testing.yml` (the self-hosted GPU
job). You can also run it by hand against any booted deploy.

## What is real vs substituted

| Area | CI behavior | Why |
|------|-------------|-----|
| Webots simulator | Real Webots container, started per run | Exercises ROS topics, sensors, TF, mapping/nav/explore against a simulated robot. |
| ROS graph | Real ROS topics from the sim and service containers | Catches topic, frame, DDS domain, and map/nav integration issues. |
| Atlas / executor / pilot | Real binaries from the checked-out ref | Tests registry, dispatch, planning loop, and executor leaf handling. |
| Camera / lidar | Real Webots sensor data | Snapshot scenarios assert returned image/point-cloud metadata. |
| Mapping / nav2 / explore | Real service/skill containers | Scenarios exercise map save, object-derived navigation, and bounded explore status/cancel. |
| Memory / voiceprint | Real services | Scenarios assert save/search and list behavior. |
| VLM | Deterministic fake OpenAI-compatible endpoint | Real LLM calls would make CI nondeterministic and require secrets. The fake only scripts RTDL plans. |
| Scene perception | Real scene service over Webots RGB-D, camera intrinsics, and ConceptGraphs | Scenarios require non-robot objects from the live scene registry and use one as the navigation target. |
| Speech/audio hardware | Real speech service plus audio driver using ALSA `null` devices | The runner has no physical speaker/mic. CI checks TTS-to-speaker and mic/speaker driver wiring through normal ALSA devices, not acoustic output. |

This workflow is therefore not a perception-quality or physical-audio quality
test. It is a build/boot/dispatch/runtime integration test for the
Webots-backed robot stack, with deterministic substitution only at the VLM
planning boundary.

## Pieces

| Path | Role |
|------|------|
| `fake_vlm/server.py` | OpenAI-compatible `/v1/chat/completions` (streaming). Returns scripted **RTDL envelopes**, not OpenAI tool-calls. Stdlib + PyYAML. |
| `SCENARIO_SPEC.md` | Normative YAML scenario grammar. |
| `scenarios/cap/*.yaml` | single-capability tests. |
| `scenarios/flow/*.yaml` | multi-step task flows, incl. fault injection + recovery. |
| `run.py` | runs cap + flow scenarios; writes `logs/<family>.<name>.jsonl`; asserts each RTDL leaf's args/results, fault recovery, and final status; prints coverage. |
| `run_interfaces.py` | direct atlas/pilot/audio RPC checks; writes `logs/iface.*`. |
| `report.py` | renders `testing/report/index.html`, Markdown summary, embedded logs, and optional LLM-assisted analysis. |
| `build_report_from_artifacts.py` | canonical report builder used by both direct Webots workflow runs and ChatOps Pages publishing. |
| `collect_diagnostic_context.py` | collects bounded PR metadata/diff plus relevant logs for failure/success analysis; it does not upload the whole repository. |
| `llm_diagnose.py` | calls the configured DeepSeek-compatible chat endpoint and writes `llm-analysis.json`; failures are non-fatal. |
| `logs/` | per-run traces (git-ignored). |
| `report/` | generated report files (git-ignored). |

Execution order after boot:

1. `testing/run_interfaces.py` checks direct APIs: atlas discovery, pilot task
   submission, and audio gRPC through ALSA `null` devices.
2. `testing/run.py` runs every YAML scenario under `scenarios/cap/` and
   `scenarios/flow/`.
3. The workflow uploads per-scenario JSONL event streams, boot logs, provider
   logs, sim stdout, ROS logs, final `rbnx caps`, and a summary JSON.

## Reports and sharing

Every run produces the same report shape, regardless of whether it was started
by a push, PR event, manual dispatch, or `@robonix-ci test`:

1. GitHub job summary: `testing/report/summary.md` is appended to
   `$GITHUB_STEP_SUMMARY`, so reviewers can scan pass/fail status without
   downloading artifacts.
2. HTML report artifact: `testing/report/index.html` summarizes scenarios,
   failures, dispatched contracts, coverage, embedded logs, and optional
   LLM-assisted analysis. It is uploaded with the raw logs.
3. Pages report: `@robonix-ci test` republishes the same report through the
   trusted CI bot workflow and links it from the PR status comment.
4. Raw logs: scenario JSONL streams, provider logs, sim stdout, ROS logs,
   final caps, fake VLM log, boot log, and remote provider provenance.

Report generation is split from the self-hosted GPU run. The GPU job uploads raw
artifacts only; a hosted report job downloads those artifacts, builds the final
HTML, and, when `DEEPSEEK_API_KEY` is configured, asks DeepSeek to write a
human-readable analysis paragraph. The LLM context is bounded to PR metadata,
PR file patches from GitHub, scenario summary, environment metadata, and capped
artifact logs. It does not send the full repository tree, and the model output is
advisory only: pass/fail remains determined by deterministic harness checks.

The public Pages report is rebuilt by the bot workflow from trusted workflow
metadata plus selected logs. The report generator rejects symlinks, applies log
size caps, and redacts common token/password patterns before embedding logs or
sending diagnostic context to the LLM.

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
(`- capability_name: tiago_camera.camera_snapshot`). Scenarios must reference
the exact provider-qualified planner capability name, such as
`tiago_camera.camera_snapshot` or `scene.scene_list_objects`; the fake VLM does
not do suffix or substring resolution.

Pilot feeds RTDL executor feedback back into VLM history as structured
`leaf_result` JSON (`call_id`, `contract_id`, `success`, `output`, `error`). The
fake VLM advances through ordinary `steps` by observed contract success, not by
raw request count. If Pilot asks again before a prior result is visible in
history, the fake VLM returns an empty wait RTDL for the already-served step
instead of resubmitting side-effecting work such as navigation. A step that sets
`retry_delay_s` is stricter: the fake VLM also requires that step's
`expect.output` assertions to pass before it advances, and may serve the same
step again while real perception/scene graph state converges. The runner always
checks the full step-local expectations when scoring the scenario.

Timing is modeled inside `steps`, similar to timeline/marble tests: `time_s`
sets an absolute offset from the first planning round and `delay_s` sleeps
relative to the current step. `retry_delay_s` sleeps before re-serving a step
whose previous leaf result failed that step's output checks. Because this is a
real Webots/ROS integration test, these waits use wall-clock time rather than a
fake clock.

## Scenario YAML

The scenario YAML grammar is defined in `SCENARIO_SPEC.md`. In short:

- `steps` is a scripted timeline of fake-VLM RTDL rounds.
- Each step's `rtdl` field is the actual RTDL tree returned for that round; one
  step may contain one leaf or many leaves under `sequence`/`parallel`.
- `sequence`, `parallel`, and `do` nodes are written explicitly; the harness
  does not wrap a separate cap list into a hidden tree.
- `do.cap` selects an exact planner-advertised capability name.
- Every committed `do` node has `expect.contract`, `expect.success`, and
  optional `expect.args`/`expect.output` checks.
- `expect.capture` extracts named values from a proven leaf output; later steps
  can reference them with `{var: name}` or `$name` string interpolation. Result
  dependencies are intentionally across VLM rounds, not hidden within one tree.
- Failed leaves are errors unless the exact `do` node expects `success: false`.

**Exception injection** is done purely in the scenario: a step calls a real
capability with invalid args (e.g. a required field omitted) so the leaf
genuinely fails, then a later step recovers. This drives pilot's real
replan-on-failure path with no test hooks in production code.


## Verify scene map persistence

For debugging map save/load regressions, use `testing/verify_scene_map_persistence.py`
against an already booted Webots deploy. It calls the Scene public map API, then
uses the mapping and sim containers only as verifiers: SQLite `quick_check`,
RTAB-Map table counts, preview PNG size, metadata, Scene's fresh-occupancy
confirmation for the requested load, and a non-empty `/map` must all be valid.
The report records any width or height difference between the saved preview and
the live occupancy grid as a diagnostic warning because RTAB-Map may
re-quantize the grid boundary on load.

```bash
# Verify an existing saved map without writing a new artifact.
python3 testing/verify_scene_map_persistence.py \
  --map-id codex_preview_fix_20260709_191939 \
  --skip-save

# Full save -> inspect artifact -> load -> compare live /map.
python3 testing/verify_scene_map_persistence.py \
  --map-id manual_apartment_$(date +%Y%m%d_%H%M%S)
```

This script intentionally does not expose RTAB-Map paths through the Robonix map
contract. The provider artifact is opaque to normal callers; direct container
inspection here is a diagnostic check only.

## Run it by hand after Quickstart

The scenario runner does not start Webots for you. It assumes the local Webots
deploy is already running, exactly like the normal quickstart path. Local users
should start the normal GUI simulator; `ROBONIX_SIM_STREAM` is only for CI or
headless debugging.

Terminal 1, from the repository root:

```bash
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
unset ROBONIX_SIM_STREAM WEBOTS_HEADLESS_MODE
bash examples/webots/sim/start.sh
```

Terminal 2, from the repository root, start the deterministic VLM. Port `18421`
is used here to avoid common local proxy ports; any free localhost port is fine
as long as `VLM_BASE_URL` below matches it.

```bash
python3 testing/fake_vlm/server.py --port 18421
```

The fake VLM logs deterministic prompt byte/four-byte-token estimates for every
request and returns a streaming `usage` chunk when Pilot requests one. This
makes multi-round prompt regressions visible in `fake_vlm.log` without changing
the scripted RTDL behavior or calling an external model.

Terminal 3, boot Robonix against that fake VLM:

```bash
cd examples/webots
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export VLM_BASE_URL=http://127.0.0.1:18421/v1
export VLM_API_KEY=fake
export VLM_MODEL=fake-vlm
rbnx boot --no-update-check
```

Terminal 4, from the repository root, run the checks:

```bash
export ROBONIX_ATLAS=127.0.0.1:50051
python3 testing/run_interfaces.py --server "$ROBONIX_ATLAS"
python3 testing/run.py --server "$ROBONIX_ATLAS" \
  --timeout 900 \
  --summary-json /tmp/robonix-local-summary.json
```

For a targeted live run:

```bash
python3 testing/run.py --server "$ROBONIX_ATLAS" \
  --only object_navigation \
  --timeout 900
```

To inspect the same HTML shape used by CI:

```bash
python3 testing/report.py \
  --summary-json /tmp/robonix-local-summary.json \
  --out-dir /tmp/robonix-local-report
python3 -m http.server --directory /tmp/robonix-local-report 18083
```

Then open `http://127.0.0.1:18083/`.

Do not drive the robot manually through RViz, teleop, or another nav client
while the scenario suite is running. Navigation and explore scenarios issue real
actions, so another client can preempt or alter the expected result.

`run_interfaces.py` and `run.py` both exit non-zero on failure and write
per-test traces under `testing/logs/`. `run.py` exits non-zero if any scenario
fails an assertion or pilot reports FAILED. The coverage section lists every
`contract_id` exercised across the run.

## Adding coverage with scenario YAML

The shipped scenarios cover builtins, camera/lidar snapshot, memory
save+search, voiceprint list, speech speak, map save, scene object listing,
explore start/status/cancel, and object-derived navigation.

For ordinary coverage, adding a testcase is just adding or editing one scenario
YAML file under `testing/scenarios/builtin/`, `testing/scenarios/cap/`, or
`testing/scenarios/flow/`. `testing/run.py` auto-discovers committed
`.yaml`/`.yml` files in those directories; there is no registry to update.

Use this checklist:

0. Copy an existing `testing/scenarios/**/*.yaml` and edit it.
1. Find its capability — only `[mode] type = "rpc"` contracts become planner
   tools; `topic_in/out` ones are ROS data channels, not callable.
2. Read its `.srv` IDL under `capabilities/lib/<area>/srv/` for the arg fields.
3. Add a scenario whose `steps[].rtdl` mirrors the RTDL tree you want Pilot to
   receive. Put assertions inside each `do.expect`, not in a global footer.
4. Validate the offline parser and fake-VLM model:

   ```bash
   python3 testing/simulate_ci.py
   ```

5. Run the new case against a booted Webots deploy:

   ```bash
   python3 testing/run.py --server 127.0.0.1:50051 \
     --only <scenario_name> \
     --timeout 900
   ```

First-run logs from `fake_vlm/server.py` print unresolved exact cap names
against the live catalog; use that to fix `cap` and `args`.

Code changes are only needed when the YAML needs behavior the harness does not
already support, for example:

- a new assertion operator or variable transform in `SCENARIO_SPEC.md`;
- new fake-VLM planning behavior that cannot be expressed as timeline steps,
  RTDL nodes, captures, or variables;
- a new provider/capability that is not built, booted, or advertised yet;
- new required CI environment, simulator, or report metadata.

For PR testing, `@robonix-ci test` runs the GitHub merge ref
`refs/pull/<PR>/merge`. That means scenario YAML added in the PR is included in
the tested checkout, as long as GitHub can create the merge ref. If the PR does
not merge cleanly into its base branch, Webots is not run and the report should
describe that infrastructure state instead of pretending the scenario suite
passed.

Remaining useful coverage: direct chassis velocity with strict cleanup,
long-duration navigation completion, and non-CI real detector scene-graph
assertions once model assets are pinned on the runner.
