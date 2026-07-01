# Scenario Spec

The scenario files under `testing/scenarios/` drive Pilot through the fake VLM
while every capability call still goes to the live robonix deployment. The fake
VLM only makes planning deterministic; it does not replace scene perception,
navigation, exploration, audio, mapping, memory, or executor logic.

## Top-level fields

```yaml
name: object_navigation
task: "ci-flow: explore, choose a detected object, and navigate near it"
steps: []
expect_min_rounds: 4
expect_final_done: true
expect_leaves: []
```

| Field | Required | Meaning |
| --- | --- | --- |
| `name` | yes | Stable scenario id; also used for log/report file names. |
| `task` | yes | Exact user task text. The fake VLM selects a scenario by this string. |
| `steps` | yes | Timeline of RTDL envelopes served to Pilot. |
| `expect_min_rounds` | no | Minimum number of Pilot planning rounds. |
| `expect_final_done` | no | Defaults to `true`; fails if Pilot reports a failed final state. |
| `expect_leaves` | yes | Strict assertions over executor leaf results. |

## Timeline steps

Each step is one planned assistant turn. `time_s` is an absolute offset from the
first request for the task, so a step can intentionally leave wall-clock time for
Webots, exploration, perception, or scene graph updates before the next plan is
served.

```yaml
steps:
  - time_s: 0.0
    status: in_progress
    rtdl_description: start explore
    content: Starting exploration.
    caps:
      - cap: explore.explore_explore
        args: {area_hint: "ci local room", timeout_s: 8.0, max_speed_m_s: 0.1}
        description: run a bounded exploration pass
  - time_s: 12.0
    status: in_progress
    rtdl_description: list scene objects
    caps:
      - cap: scene.scene_list_objects
        args: {}
  - status: done
```

`cap` must be the exact provider-qualified planner capability name advertised in
Pilot's system prompt, for example `scene.scene_list_objects` or
`nav2.navigation_navigate`. Short names and substring matching are intentionally
not supported.

## Dynamic Arguments

A later step may consume a value from earlier leaf output using `from_result`.
The lookup scans prior Pilot feedback JSON and must match `contract` or
`contains`; it never falls back to unrelated fields with the same name.

```yaml
object_id:
  from_result:
    contains: objects
    field: objects[].id
    where:
      label_not: robot
      id_prefix: scene.object.
```

Supported selectors:

| Key | Meaning |
| --- | --- |
| `contract` | Optional full `robonix/...` contract id anchor if present in feedback. |
| `contains` | Required when feedback omits contract ids; matches prior result JSON/text. |
| `field` | Field or dotted path. `objects[].id` selects the first list item matching `where`. |
| `where` | Optional filters for array items: `label_not`, `label_in`, `id_prefix`, `id_contains`, or exact field equality. |
| `default` | Optional fallback if no matching result exists. |
| `transform` | Optional value transform: `float`, `int`, `str`, `sin_half`, `cos_half`. |

`sin_half` and `cos_half` are used to convert a returned yaw into a navigation
quaternion.

## Leaf Assertions

Assertions are scoped to a single executor leaf. A scenario passes only when a
leaf with the specified `contract` and `success` exists and all declared
argument/output checks pass.

```yaml
expect_leaves:
  - contract: robonix/system/scene/list_objects
    success: true
    output:
      jsonpath:
        - path: $.objects
          min_length: 1
        - path: $.objects[].id
          exists: true
          prefix: scene.object.
  - contract: robonix/service/navigation/navigate
    success: true
    output:
      json: {accepted: true}
```

Supported assertion clauses:

| Clause | Meaning |
| --- | --- |
| `args` | JSON subset that must be present in the dispatched call args. |
| `output.text_equals` | Exact text output match. |
| `output.text_regex` | Regular expression over the leaf output text. Use anchors for exactness. |
| `output.error_regex` | Regular expression over the leaf error text, for expected failed leaves. |
| `output.text_lines` | Exact output lines that must be present. |
| `output.json` | JSON subset that must be present in parsed leaf output. |
| `output.jsonpath` | Minimal JSON path checks: `exists`, `min_length`, `equals`, `prefix`. |

Failed leaves are errors unless they are explicitly declared with
`success: false` in `expect_leaves`. This is how fault-injection scenarios prove
that the injected failure happened and that Pilot recovered afterward.
