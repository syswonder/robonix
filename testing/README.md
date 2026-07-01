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

## Checks

## Pieces

| Path | Role |
|------|------|
| `fake_vlm/server.py` | OpenAI-compatible `/v1/chat/completions` (streaming). Returns scripted **RTDL envelopes**, not OpenAI tool-calls. Stdlib + PyYAML. |
| `SCENARIO_SPEC.md` | Normative YAML scenario grammar. |
| `scenarios/cap/*.yaml` | single-capability tests. |
| `scenarios/flow/*.yaml` | multi-step task flows, incl. fault injection + recovery. |
| `run.py` | runs cap + flow scenarios; writes `logs/<family>.<name>.jsonl`; asserts dispatch, args, results, round count, fault recovery; prints coverage. |
| `run_interfaces.py` | direct atlas/pilot/audio RPC checks; writes `logs/iface.*`. |
| `report.py` | converts `testing/logs/summary.json` into `testing/report/index.html` and a Markdown job summary. |
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

Every run produces three levels of evidence:

1. GitHub job summary: `testing/report/summary.md` is appended to
   `$GITHUB_STEP_SUMMARY`, so reviewers can scan pass/fail status without
   downloading artifacts.
2. HTML report artifact: `testing/report/index.html` summarizes scenarios,
   failures, dispatched contracts, and coverage. It is uploaded with the raw
   logs.
3. Pages report: `@robonix-ci test` publishes the same report through the
   trusted CI bot workflow and links it from the PR status comment.
4. Raw logs: scenario JSONL streams, provider logs, sim stdout, ROS logs,
   final caps, fake VLM log, boot log, and remote provider provenance.

The public Pages report is rebuilt by the bot workflow from trusted workflow
metadata plus selected logs. The report generator rejects symlinks, applies log
size caps, and redacts common token/password patterns before embedding logs.

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
not do suffix or substring resolution. Round index = number of additional
`user` result messages already in history; the server serves `steps[round]` and
a terminal `done` envelope past the end.

Timing is modeled inside `steps`, similar to timeline/marble tests: `time_s`
sets an absolute offset from the first planning round and `delay_s` sleeps
relative to the current step. A step with no `caps` is an empty RTDL wait node,
so it can fill blank timeline spans without dispatching robot work. Because
this is a real Webots/ROS integration test, these waits use wall-clock time
rather than a fake clock.

## Scenario YAML

The scenario YAML grammar is defined in `SCENARIO_SPEC.md`. In short:

- `steps` is a scripted timeline of fake-VLM RTDL rounds.
- `steps[].time_s` and `steps[].delay_s` model blank time inside the timeline.
- `steps[].caps[].cap` selects exact planner-advertised capability names.
- `steps[].caps[].args` may use `from_result` to feed a previous leaf output
  field into a later capability argument.
- `expect_leaves` binds each assertion to one exact `robonix/...` runtime
  contract id, expected success state, optional args subset, and optional output
  checks.
- Failed leaves are errors unless the scenario explicitly expects that leaf with
  `success: false`.

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

The shipped scenarios cover builtins, camera/lidar snapshot, memory
save+search, voiceprint list, speech speak, map save, scene object listing,
explore start/status/cancel, and object-derived navigation. To add a tool:

0. Copy an existing `scenarios/*.yaml` and edit it (PyYAML is the only dep).
1. Find its capability — only `[mode] type = "rpc"` contracts become planner
   tools; `topic_in/out` ones are ROS data channels, not callable.
2. Read its `.srv` IDL under `capabilities/lib/<area>/srv/` for the arg fields.
3. Add a scenario; first-run logs from `fake_vlm/server.py` print the live
   catalog **with arg schemas** if a matcher doesn't resolve — use that to fix
   the `match`/`args`.

Remaining useful coverage: direct chassis velocity with strict cleanup,
long-duration navigation completion, and non-CI real detector scene-graph
assertions once model assets are pinned on the runner.
