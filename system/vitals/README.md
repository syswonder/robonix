# Vitals — health monitoring, heartbeat

One of the 12 Robonix system components. Aggregates liveness and
health signals from every running component.

**Status — v0.1 partial.** The atlas heartbeat loop today covers the
"is this provider still alive" question (see
`system/atlas/src/service.rs` — `HeartbeatRequest` + the eviction
timer). Capability-health and body-health (battery, thermals, joint
fault states) are not centralised.

When Vitals is split out it will:

- collect process-level liveness from atlas (today's heartbeat) plus
  capability-level health from per-component self-reports,
- ingest body / hardware health from [soma](../soma/) (battery,
  joint temperatures, motor faults, …),
- expose a single `vitals/state` capability that pilot / liaison /
  sentinel can query for "can we accept new tasks right now".
