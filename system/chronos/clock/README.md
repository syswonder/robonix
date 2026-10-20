# Chronos clock

In-process Linux wall and steady clock readings with uncertainty and clock generations. No running service, Atlas or logging dependency is required.

Use `reference_time()`, `steady_time()`, `wall_time()` or `snapshot()` from `robonix_chronos_clock`. Reads return errors instead of fabricated timestamps. Steady timestamps are local to the host.

For caller-supplied simulation time, select `ReferenceSourceKind::RosTime` and inject `VirtualReferenceTime` readings with `update_reference_time()`. Zero is valid; paused readings are retained without extrapolation. Backward time requires a new generation. This library does not subscribe to ROS.

Injected readings respect `max_uncertainty_ns`. An unavailable reading does not erase the last available timestamp used to reject backward time in the same generation. Generation changes remain observable after a failed snapshot read.

Run `cargo test -p robonix-chronos-clock`.
