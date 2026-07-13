# Sentinel - capability-call safety policy

Sentinel decides whether a requested capability call is allowed under the
active rule set before Executor dispatches side-effecting work.

## Current implementation

The `robonix-sentinel` crate provides the first transport-independent policy
core. It supports:

- allow and deny rules with deterministic priority selection;
- `*` wildcards in contract patterns;
- local weekday and time windows, including windows that cross midnight;
- simple numeric JSON argument conditions such as `linear_x > 0.5`;
- exact user ID or role matching;
- validation and atomic replacement of the active rule set;
- default allow when no rule matches.

Equal-priority rules preserve the order supplied to `set_rules`; the first
matching rule wins.

## Integration boundary

This crate does not yet enforce runtime calls. Executor must call it before
dispatch, while Chronos and Keystone must supply canonical time and user roles.
The gRPC contracts, Atlas registration, persistent rule source, and management
API remain follow-up integration work.

Run the focused tests from the repository root:

```bash
cargo test -p robonix-sentinel
```
