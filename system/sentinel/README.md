# Sentinel — safety supervision

One of the 12 Robonix system components. Intercepts capability calls
from [executor](../executor/) and enforces safety rules: rate limits,
allow/deny lists, deny-windows, hard-stops.

**Status — v0.1 minimal, currently embedded in executor.**

In v0.1 sentinel is not a separate process. The dispatch path in
[`system/executor/src/dispatch/sentinel.rs`](../executor/src/dispatch/sentinel.rs)
loads `sentinel.yaml` rules at boot and gates each capability call
through them. Demo coverage:

- rule-driven deny / allow for capability calls,
- `deny_window` semantics for time-bounded blocks,
- denied-call telemetry surfaces in chat / TUI.

When Sentinel is extracted into its own process it will:

- run alongside executor as a separate component with its own gRPC
  surface (consumers ask "may I call X under identity Y given the
  current scene") rather than as an executor-internal module,
- subscribe to [scene](../scene/) state, [vitals](../vitals/) health,
  and [keystone](../keystone/) identity so rules can be context-aware
  rather than purely static,
- get its own log channel to [scribe](../scribe/).

See the whitepaper §3 *L2 执行安全层*.
