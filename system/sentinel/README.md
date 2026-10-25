# Sentinel - capability-call policy

Sentinel provides the transport-independent rule model and policy evaluation
for capability calls. Executor evaluates the active rules immediately before
dispatching each RTDL `do` node.

Rules can match a contract wildcard, exact Keystone users or roles, local-time
windows, typed call arguments, and typed robot context. The highest-priority
matching rule wins. Calls are allowed when no rule applies, while a candidate
rule with unavailable or wrongly typed trusted data fails closed.

See [the Webots demo](../../examples/webots/SENTINEL_DEMO.md) for policy setup.

Executor serves the `list_rules` and `replace_rules` capability contracts from
the Sentinel management endpoint. Replacements are validated and persisted
before they become active.

Run the policy tests from the repository root:

```bash
cargo test -p robonix-sentinel
```

For the local Keystone-to-Executor test, including Liaison, Pilot, and `rbnx`,
build the binaries and run:

```bash
scripts/test_sentinel_keystone.sh
```

The script needs Python with `grpcio` and `grpcio-tools`.
