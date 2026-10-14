# Sentinel - capability-call policy

Sentinel provides the transport-independent rule model and policy evaluation
for capability calls. Executor and Atlas do not invoke Sentinel yet, so these
rules are not enforced by the current call path.

Rules can match a contract wildcard, exact Keystone users or roles, local-time
windows, typed call arguments, and typed robot context. The highest-priority
matching rule wins. Calls are allowed when no rule applies, while a candidate
rule with unavailable or wrongly typed trusted data fails closed.

Predicates use an RFC 6901 JSON Pointer and an explicit `boolean`, `string`,
`integer`, or `float` type. Boolean and string predicates support `eq`/`ne`;
numeric predicates additionally support `lt`/`le`/`gt`/`ge` and ranges with
independently inclusive bounds. No string, number, or boolean coercion is
performed. For example:

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

The `~1` escape represents `/` inside a JSON Pointer component. Argument
predicates use the same schema but address the capability's `args_json` root.

The `list_rules` and `replace_rules` capability contracts describe the rule
management interface. Their provider and persistence behavior will be added
with the runtime integration.

Run the policy tests from the repository root:

```bash
cargo test -p robonix-sentinel
```
