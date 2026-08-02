# Sentinel - capability-call policy

Sentinel is the robot-wide policy engine served from the Executor process for
the v0.1 runtime. It registers as its own Atlas provider, and Executor evaluates
each capability call immediately before dispatch.

Rules can match a contract wildcard, exact Keystone users or roles, local-time
windows, typed call arguments, and typed robot context. The context is fetched
by Sentinel from Soma, Vitals, and Scene; Client and Task metadata are never
trusted as robot state. The highest-priority matching rule wins. Calls are
allowed when no rule applies, while a candidate rule with missing, stale, or
wrongly typed trusted data fails closed.

Predicates use an RFC 6901 JSON Pointer and an explicit `boolean`, `string`,
`integer`, or `float` type. Boolean and string predicates support `eq`/`ne`;
numeric predicates additionally support `lt`/`le`/`gt`/`ge` and ranges with
independently inclusive bounds. No string/number/boolean coercion is performed.
For example, one deployment can define "left gripper is closed" using its
actual calibrated Soma position interval rather than a Sentinel-wide
`gripper_open` enum:

```json
{
  "path": "/soma/data/actuators/body~1arm~1left_piper~1gripper/position/value",
  "kind": "float",
  "operator": "range",
  "min": 0.0,
  "max": 0.01,
  "min_inclusive": true,
  "max_inclusive": true
}
```

The `~1` escape is the JSON Pointer representation of `/` inside a component
ID. Argument predicates use the same schema but address the capability's
`args_json` root directly.

Administrators manage the complete rule set through the Sentinel `list_rules`
and `replace_rules` contracts. Executor verifies the supplied Keystone session
before reading or replacing the robot-local policy file.

Run the policy-core tests with:

```bash
cargo test -p robonix-sentinel
```
