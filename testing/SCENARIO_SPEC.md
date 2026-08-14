# Scenario Spec

Scenario files under `testing/scenarios/` drive Pilot through the deterministic
fake VLM while every RTDL leaf still executes against the live Robonix
deployment. The fake VLM only makes planning deterministic; it does not replace
scene perception, navigation, exploration, audio, mapping, memory, or executor
logic.

This file is the source of truth for committed Webots integration scenarios.
If a test needs a new assertion form, add it here and in the harness before
using it in YAML.

## Core Model

One `steps[]` item is one VLM planning round. For that round, the fake VLM
returns the step's `rtdl` tree as one assistant RTDL envelope. The harness does
not invent an implicit shape around the YAML: if the test needs a `sequence`, a
`parallel`, or a nested tree, write that tree directly.

The live Pilot loop is asynchronous. Pilot executes an RTDL tree, then sends
leaf feedback back to the VLM as structured JSON user content:

```json
{"leaf_result":{"call_id":"4:0","contract_id":"robonix/...","success":true,"output":"...","error":""}}
```

The fake VLM advances by that evidence, not by raw request count. Ordinary steps
advance once their expected RTDL leaves have observed the declared contract and
success state. Steps with `retry_delay_s` additionally require their full output
assertions and captures to match before advancing; if a real result exists but
does not satisfy the output checks, the fake VLM may serve the same step again
after the delay. Use this only for real readiness polling, such as waiting for
scene objects to appear, not for hiding failures.

The runner scores the same ordered step model. It matches each YAML step to the
next executed plan round whose leaves satisfy that step's expectations. A leaf
from a later or unrelated round cannot satisfy an earlier step, and one leaf
result cannot satisfy two expectations.

Top-level fields:

| Field | Required | Meaning |
| --- | --- | --- |
| `name` | yes | Stable scenario id; also used for log/report file names. |
| `task` | yes | User task text. The fake VLM selects a scenario by this string. |
| `steps` | yes | Timeline of RTDL envelopes served to Pilot. |
| `expect_final_done` | no | Defaults to `true`; fails if Pilot reports a failed final state. |
| `allow_leaf_failure` | no | Defaults to `false`; only use for debugging, not committed CI scenarios. |

## Step Fields

| Field | Meaning |
| --- | --- |
| `time_s` | Absolute wall-clock offset from the first planning round. Use this to leave time for Webots, exploration, perception, or scene graph updates. |
| `delay_s` | Relative wall-clock sleep before serving this step. Prefer `time_s` for stable timelines. |
| `retry_delay_s` | Relative sleep before re-serving this step after a prior result failed this step's full expectations. |
| `content` | Assistant narration in the RTDL envelope. |
| `rtdl_description` | Envelope-level RTDL label. |
| `status` | Shorthand for `task_update.status`; typically `in_progress` or `done`. |
| `task_update` | Optional explicit task update object. |
| `rtdl` | The RTDL tree for this VLM round. Required unless the step only marks `done`. |

Example with multiple RTDL leaves in one VLM round:

```yaml
steps:
  - status: in_progress
    rtdl_description: capture sensor context
    rtdl:
      op: parallel
      id: sensor_context
      children:
        - op: do
          id: camera_snapshot
          cap: tiago_camera.camera_snapshot
          args: {}
          expect:
            contract: robonix/primitive/camera/snapshot
            success: true
            output:
              json: {encoding: jpeg}
        - op: do
          id: lidar_snapshot
          cap: tiago_lidar.lidar_snapshot
          args: {}
          expect:
            contract: robonix/primitive/lidar/snapshot
            success: true
            output:
              checks:
                - select: $.ranges
                  assert: {op: min_length, value: 1}
```

Example with a later round depending on a previous leaf result:

```yaml
steps:
  - time_s: 45.0
    retry_delay_s: 3.0
    status: in_progress
    rtdl_description: list scene objects
    rtdl:
      op: sequence
      id: select_scene_object
      children:
        - op: do
          id: list_scene_objects
          cap: scene.scene_list_objects
          args: {}
          expect:
            contract: robonix/system/scene/list_objects
            success: true
            output:
              checks:
                - select: $.objects
                  assert: {op: min_length, value: 1}
                - select: $.objects[]
                  where:
                    all:
                      - {path: $.label, op: ne, value: robot}
                      - {path: $.id, op: starts_with, value: scene.object.}
                  assert: {op: exists}
            capture:
              object_id:
                select: $.objects[]
                where:
                  all:
                    - {path: $.label, op: ne, value: robot}
                    - {path: $.id, op: starts_with, value: scene.object.}
                path: $.id
```

## RTDL Nodes

Supported node ops are the RTDL ops Pilot accepts here:

| Op | Required fields | Meaning |
| --- | --- | --- |
| `sequence` | `children` | Execute child nodes in order. |
| `parallel` | `children` | Execute child nodes concurrently. |
| `do` | `cap`, `args`, `expect` | Execute one planner capability. |

`cap` must be the exact provider-qualified planner capability name advertised in
Pilot's system prompt, for example `scene.scene_list_objects` or
`nav2.navigation_navigate`. Short names and substring matching are intentionally
not supported.

Every committed `do` node must carry `expect`. This keeps the planned work and
its correctness check in the same place and prevents scenarios from passing just
because a call was made.

Test-only node fields:

| Field | Applies to | Meaning |
| --- | --- | --- |
| `id` | any node | Stable scenario-local RTDL node name for reports and fake-VLM bookkeeping. Not sent to Pilot. This is not a scene object id. |
| `expect` | `do` | Runtime leaf assertion for this planned call. Not sent to Pilot. |
| `capture` | inside `expect` | Named values extracted from this leaf for later steps. Not sent to Pilot. |
| `once` | `do` | Suppress this node if the same node id has already been served in this scenario. Not sent to Pilot. |

## Leaf Assertions

Assertions are scoped to the RTDL `do` node that planned the call. The runner
first finds the next RTDL plan round whose leaf set matches the YAML step, then
matches only leaves whose `call_id` belongs to that plan round.

```yaml
- op: do
  id: navigate_object_goal
  cap: nav2.navigation_navigate
  args:
    goal: { ... }
  expect:
    contract: robonix/service/navigation/navigate
    success: true
    args:
      goal:
        header: {frame_id: map}
    output:
      json: {state: SUCCEEDED}
```

Supported assertion clauses:

| Clause | Meaning |
| --- | --- |
| `contract` | Required full `robonix/...` runtime contract id expected from this leaf. |
| `success` | Expected leaf success boolean; defaults to `true`. |
| `args` | JSON subset that must be present in the dispatched call args. |
| `output.text_equals` | Exact text output match. |
| `output.text_regex` | Regular expression over leaf output text. Use anchors for exactness. |
| `output.error_regex` | Regular expression over leaf error text, for expected failed leaves. |
| `output.text_lines` | Exact output lines that must be present. |
| `output.json` | JSON subset that must be present in parsed leaf output. |
| `output.checks` | Structured selector checks over parsed leaf output. See below. |

Failed leaves are errors unless the exact failing `do` node declares
`success: false` and its assertion passes. A contract-only allow-list is not
enough.

## Selectors

`output.checks` and `capture` use the same small JSON selector grammar. This is
not full JSONPath. Supported forms are intentionally limited so tests stay easy
to review:

| Selector | Meaning |
| --- | --- |
| `$` | The whole parsed output or selected item. |
| `$.field` | Object field. |
| `$.a.b` | Nested object fields. |
| `$.array[]` | Every item in an array field. |
| `$.array[].field` | Field from every item in an array. |

The selector always runs against parsed JSON output. If a capability returns a
JSON string in the `output` field, the harness parses it before applying checks.

## Structured Checks

`output.checks` is a list. Each entry selects values, optionally filters selected
items with `where`, then applies `assert` to the remaining values.

```yaml
output:
  checks:
    - select: $.objects
      assert: {op: min_length, value: 1}
    - select: $.objects[]
      where:
        all:
          - {path: $.label, op: ne, value: robot}
          - {path: $.id, op: starts_with, value: scene.object.}
      assert: {op: exists}
```

Fields:

| Field | Required | Meaning |
| --- | --- | --- |
| `select` | no | Selector over the parsed output. Defaults to `$`. |
| `where` | no | Predicate expression evaluated against each selected value. Values that fail are removed. |
| `assert` | no | Predicate over the selected values after filtering. Defaults to `{op: exists}`. |

`where` and `assert` use the same predicate grammar:

```yaml
all:
  - {path: $.label, op: ne, value: robot}
  - {path: $.id, op: starts_with, value: scene.object.}
```

Logical predicates:

| Form | Meaning |
| --- | --- |
| `all: [pred, ...]` | Every child predicate must pass. |
| `any: [pred, ...]` | At least one child predicate must pass. |
| `not: pred` | Child predicate must fail. |

Leaf predicates:

| Field | Required | Meaning |
| --- | --- | --- |
| `path` | no | Selector relative to the current value. Defaults to `$`. |
| `op` | yes | Predicate operation. |
| `value` | op-dependent | Expected value. Not used by `exists`. |

Supported `op` values:

| Op | Meaning |
| --- | --- |
| `exists` | At least one value was selected. |
| `eq` / `ne` | Equal / not equal. |
| `in` / `not_in` | Value membership against a scalar or list. |
| `starts_with` | String prefix check. |
| `contains` | String containment check. |
| `regex` | Python regular expression search over the stringified value. |
| `gt` / `gte` / `lt` / `lte` | Numeric comparisons. |
| `min_length` | Selected value has length greater than or equal to `value`. |

Do not add field-specific shortcuts. Write the field explicitly as
`path + op + value` so the same mechanism works for any capability output
shape.

## Captures and Variables

Later steps may use values captured from earlier leaf outputs. Captures live
inside the same `expect` that proves the leaf was the intended result.

```yaml
- op: do
  id: goal_near_object
  cap: scene.scene_goal_near
  args:
    object_id: {var: object_id}
  expect:
    contract: robonix/system/scene/goal_near
    success: true
    output:
      json: {reachable: true}
      text_regex: 'approach pose for'
    capture:
      nav_x: {select: "$", path: $.x, transform: float}
      nav_y: {select: "$", path: $.y, transform: float}
      nav_yaw: {select: "$", path: $.yaw, transform: float}
```

Capture fields:

| Key | Meaning |
| --- | --- |
| `select` | Selector over parsed JSON output. Required for JSON captures. |
| `where` | Optional predicate filter for selected array items. Same grammar as structured checks. |
| `path` | Selector relative to the first selected item after filtering. Defaults to `$`. |
| `regex` | Alternative text capture from output text with a regular expression. |
| `group` | Regex capture group number; defaults to `1`. |
| `source` | `output` by default; use `error` to capture from failure text. |
| `transform` | `float`, `int`, `str`, `sin_half`, or `cos_half`. |

Use `select + path` for JSON captures and `regex` for text captures. Removed
legacy selector/filter shortcuts are not accepted by committed scenarios.

A later node can reference captured variables:

```yaml
position:
  x: {var: nav_x}
  y: {var: nav_y}
orientation:
  z: {var: nav_yaw, transform: sin_half}
  w: {var: nav_yaw, transform: cos_half}
```

Variable references are valid only in later steps. This matches the actual
planning loop: Pilot executes the current RTDL tree, sends leaf results back to
the VLM, and only the next VLM round can plan with those results. A `sequence`
node orders work in one returned RTDL tree, but this harness does not add a
test-only dataflow language where one leaf's output is interpolated into a later
leaf in the same tree. Model result-dependent plans as `capture` in one step and
`{var: name}` in a later step.

The fake VLM resolves `{var: name}` as the original typed value and `$name`
inside strings as text interpolation.

## Writing A New Test

1. Pick the smallest real user flow that proves the behavior. Do not add CI-only
   service modes or fake capability results.
2. Write each VLM round as one `steps[]` item. Use a single step with a
   `parallel` RTDL tree when one round should dispatch multiple leaves.
3. Put `expect` on every `do` node. Assert the runtime `contract`, success state,
   important args, and output facts that prove the behavior actually worked.
4. If a later round depends on a previous result, capture the value in the first
   step and use `{var: name}` in a later step.
5. Run `python3 testing/simulate_ci.py` before spending Webots/GPU time.
