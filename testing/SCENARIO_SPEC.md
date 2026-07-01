<!-- SPDX-License-Identifier: MulanPSL-2.0 -->
# Scenario YAML Spec

Scenario files under `testing/scenarios/` drive the deterministic fake VLM used
by the Webots CI harness. They are not generic YAML fixtures: each file defines
one task prompt, the RTDL timeline the fake VLM should return to pilot, and the
runtime assertions the runner should check against `rbnx ask --json` events.

## Design

The scenario timeline follows the same basic idea as timeline-based tests in
other ecosystems:

- Marble tests represent blank time before an event.
- Fake-clock tests advance time before checking the next effect.
- Coroutine scheduler tests advance virtual time before the next assertion.

Robonix CI cannot safely virtualize Webots, ROS, DDS, or provider wall clocks,
so scenario timing uses real wall-clock sleeps inside the fake VLM. The point is
still the same: time is part of the scripted scenario, not an external shell
probe and not an extra robot capability call.

## File Shape

```yaml
name: example
task: "ci-test: do something"

steps:
  - time_s: 0.0
    status: in_progress
    rtdl_description: first action
    content: Optional assistant text.
    caps:
      - match: camera_snapshot
        args: {}
        description: grab one RGB frame

  - time_s: 5.0
    status: in_progress
    rtdl_description: blank wait or second action

  - delay_s: 2.0
    status: in_progress
    caps:
      - match: memory_save
        args: {data: "example fact"}

  - status: in_progress
    caps:
      - match: navigation_navigate
        args:
          goal:
            pose:
              position:
                x:
                  from_result:
                    contract: robonix/system/scene/goal_near
                    field: x

  - status: done

expect_contracts:
  - robonix/primitive/camera/snapshot
expect_args:
  - example fact
expect_outputs:
  - '"encoding": "jpeg"'
expect_leaf_failure: []
expect_min_rounds: 2
expect_final_done: true
allow_leaf_failure: false
```

## Top-Level Fields

| Field | Required | Meaning |
|-------|----------|---------|
| `name` | yes | Stable scenario id. Defaults to the filename only if omitted. |
| `task` | yes | Task prompt passed to `rbnx ask`; fake VLM matches this text by containment in pilot's user prompt. |
| `steps` | yes | Ordered fake-VLM RTDL timeline. One step is served per planning round. |
| `expect_contracts` | no | Exact full `robonix/...` contract ids that must be dispatched and have at least one successful leaf result. |
| `expect_args` | no | Substrings that must appear in dispatched `args_json`. |
| `expect_outputs` | no | Substrings that must appear in successful leaf outputs. Use this for semantic correctness. |
| `expect_leaf_failure` | no | Exact full `robonix/...` contract ids that are expected to fail. |
| `expect_min_rounds` | no | Minimum number of `EVT_PLAN` rounds. |
| `expect_final_done` | no | Defaults to `true`; fails if pilot emits a FAILED final status. |
| `allow_leaf_failure` | no | Defaults to `false`; avoid except for intentionally broad diagnostics. Prefer `expect_leaf_failure`. |

## Step Fields

| Field | Required | Meaning |
|-------|----------|---------|
| `time_s` | no | Absolute wall-clock offset from the scenario's first fake-VLM planning round. If the fake VLM reaches this step early, it sleeps until this time. |
| `delay_s` | no | Relative wall-clock sleep before serving this step. Do not combine with `time_s`. |
| `status` | no | Task status sent in `task_update`: `in_progress` or `done`. Defaults to `in_progress`; a final `done` step ends the turn. |
| `rtdl_description` | no | Description on the fake-VLM RTDL envelope. |
| `content` | no | Assistant text streamed with the RTDL envelope. |
| `goal` | no | Optional `task_update.goal`; defaults to `ci scenario`. |
| `success_criterion` | no | Optional `task_update.success_criterion`; defaults to `scripted steps complete`. |
| `caps` | no | Capability calls to include in this round. Missing or empty means an empty RTDL sequence, i.e. a timeline gap/no-op planning round. |

## Capability Fields

| Field | Required | Meaning |
|-------|----------|---------|
| `match` | yes | Selector against planner-advertised capability names, for example `camera_snapshot` for `tiago_camera.camera_snapshot`. |
| `args` | no | Args object passed to the RTDL `do` node. Defaults to `{}`. |
| `description` | no | Human-readable leaf description. Defaults to `match`. |

`steps[].caps[].match` is intentionally not a contract id. It selects a
planner-visible capability name from pilot's live prompt. Runtime assertions use
`expect_contracts` and `expect_leaf_failure`, which must be exact full
`robonix/...` contract ids from executor leaf results.

### Dynamic Args From Previous Results

Any value under `steps[].caps[].args` may be replaced with:

```yaml
from_result:
  contract: robonix/system/scene/goal_near
  field: x
  default: 0.0   # optional
```

The fake VLM resolves this when serving the step by scanning previous pilot
`user` result messages for a JSON/text result containing the exact `contract`
string, then extracting `field` from that result. If no value is found and no
`default` is provided, the generated argument value is `null`; the scenario
should then fail through `expect_args`, `expect_outputs`, or the runtime call.

Use this for composition flows where a later capability must consume data from
an earlier capability, for example `scene/goal_near` producing the map-frame
pose used by `navigation/navigate`.

## Timing Rules

- `time_s` is absolute from the first fake-VLM request for the scenario.
- `delay_s` is relative to the current step.
- If a step's target `time_s` has already passed, the fake VLM does not sleep.
- A step with no `caps` is the supported way to fill blank time in the
  timeline without dispatching robot work.
- Timing is implemented in the fake VLM, not in `run.py`, so it is part of the
  scenario transcript.

Example:

```yaml
steps:
  - time_s: 0.0
    status: in_progress
    caps:
      - match: explore_explore
        args: {area_hint: "ci local room", timeout_s: 5.0, max_speed_m_s: 0.1}
  - time_s: 8.0
    status: in_progress
    caps:
      - match: explore_status
        args: {run_id: ""}
      - match: explore_cancel
        args: {run_id: ""}
  - status: done
```

This means: start explore at t=0, leave a blank timeline interval until t=8,
then poll and cancel the most recent explore task.

## Assertion Rules

- `expect_contracts` checks both plan dispatch and successful executor leaf
  output for each full contract id.
- `expect_args` checks planner argument JSON, so routing and parameterization
  are covered.
- `expect_outputs` checks successful leaf output text. Use it for object ids,
  map ids, image dimensions, navigation states, saved facts, and similar
  behavior-level evidence.
- `expect_leaf_failure` makes a failed leaf required and non-fatal for that
  exact contract id.
- Any other failed leaf fails the scenario unless `allow_leaf_failure: true`.

## Current Scenario Families

- `scenarios/cap/`: single-capability checks.
- `scenarios/flow/`: multi-step checks that compose capabilities or inject
  failures and then recover.
