# Scenario Spec

Scenario files under `testing/scenarios/` drive Pilot through the deterministic
fake VLM while every RTDL leaf still executes against the live Robonix
deployment. The fake VLM only makes planning deterministic; it does not replace
scene perception, navigation, exploration, audio, mapping, memory, or executor
logic.

## Core Model

One `steps[]` item is one scenario planning round: the fake VLM returns that
step's `rtdl` tree as one assistant RTDL envelope. Its `rtdl` field is the exact
RTDL tree returned to Pilot for that round. The harness does not invent an
implicit shape around the YAML: if the scenario needs a `sequence`, a `parallel`,
or a nested tree, write that tree directly.

The live Pilot loop is asynchronous, so it may ask the VLM again before the
previous leaf result has appeared in the conversation history. Pilot feeds RTDL
leaf feedback back to the VLM as structured JSON user content:
`{"leaf_result":{"call_id", "contract_id", "success", "output", "error"}}`.
The fake VLM advances by that evidence, not by raw request count: it serves the
first step whose expected RTDL leaves are not yet proven by prior leaf results.
If that step has already been served and is still waiting for a result, the fake
VLM returns an empty wait RTDL instead of resubmitting side-effecting work such
as navigation. The runner applies the same model and checks that each YAML step
matches one later RTDL plan round in order; duplicate in-flight wait rounds do
not satisfy a different step.

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
| `time_s` | Absolute wall-clock offset from the first planning round. Used to leave time for Webots, exploration, perception, or scene graph updates. |
| `delay_s` | Relative wall-clock sleep before serving this step. Prefer `time_s` for stable timelines. |
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
              jsonpath:
                - path: $.ranges
                  min_length: 1
```

Example with a later round depending on a previous leaf result:

```yaml
steps:
  - time_s: 45.0
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
              jsonpath:
                - path: $.objects
                  min_length: 1
            capture:
              object_id:
                jsonpath: $.objects[].id
                where:
                  label_not: robot
                  id_prefix: scene.object.
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
| `id` | any node | Stable scenario-local node name for humans and variable capture context. Not sent to Pilot. |
| `expect` | `do` | Runtime leaf assertion for this planned call. Not sent to Pilot. |
| `capture` | inside `expect` | Named values extracted from this leaf for later steps. Not sent to Pilot. |
| `once` | `do` | Suppress this node if the same node id has already been served in this scenario. Not sent to Pilot. |

## Leaf Assertions

Assertions are scoped to the RTDL `do` node that planned the call. The runner
first finds the next RTDL plan round whose leaf set matches the YAML step, then
matches only leaves whose `call_id` belongs to that plan round. A later or
unrelated result cannot satisfy an earlier node, and one leaf result cannot be
used to satisfy two expectations.

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
| `output.jsonpath` | Minimal JSON path checks: `exists`, `min_length`, `equals`, `prefix`. |

Failed leaves are errors unless the exact failing `do` node declares
`success: false` and its assertion passes.

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
    capture:
      nav_x: {jsonpath: $.x, transform: float}
      nav_y: {jsonpath: $.y, transform: float}
      nav_yaw: {jsonpath: $.yaw, transform: float}
```

A later node can reference those variables:

```yaml
position:
  x: {var: nav_x}
  y: {var: nav_y}
orientation:
  z: {var: nav_yaw, transform: sin_half}
  w: {var: nav_yaw, transform: cos_half}
```

Supported capture selectors:

| Key | Meaning |
| --- | --- |
| `jsonpath` | Extract a value from parsed JSON output, e.g. `$.objects[].id`. |
| `field` | Alias for a dotted JSON path. |
| `where` | Optional filters for array items: `label_not`, `label_in`, `id_prefix`, `id_contains`, or exact field equality. |
| `regex` | Extract from output text with a regular expression. |
| `group` | Regex capture group number; defaults to `1`. |
| `source` | `output` by default; use `error` to capture from failure text. |
| `transform` | `float`, `int`, `str`, `sin_half`, or `cos_half`. |

Variable references are valid only in later steps. This matches the actual
planning loop: Pilot executes the current RTDL tree, sends leaf results back to
the VLM, and only the next VLM round can plan with those results. A `sequence`
node orders work in one returned RTDL tree, but this harness does not add a
test-only dataflow language where one leaf's output is interpolated into a later
leaf in the same tree. Model result-dependent plans as `capture` in one step and
`{var: name}` in a later step.

The fake VLM resolves `{var: name}` as the original typed value and `$name`
inside strings as text interpolation.
