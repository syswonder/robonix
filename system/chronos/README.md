# Chronos

`Chronos::new(ChronosConfig)` composes the clock library and device models. Capture ingress snapshots, submit observations and query conversions and coherent quality/model snapshots. Reference and device generations invalidate stale observation tokens.

Fitted rates outside the signed coefficient range are rejected, including a floating-point rate that rounds to the exclusive upper boundary.

Refreshing a new generation preserves retained models from earlier generations; their freshness cannot be computed from the new observations.

A strictly newer reference generation clears `Fault` to `Reset` and replaces its diagnostic reason. A retained model becomes `Unsynced` with its old generation identity until refitting. Equal-generation updates are idempotent, and reference availability recovery preserves `Fault`.

Outlier checks include the configured drift allowance over steady time since the last accepted observation, so permitted rate changes and observations within holdover uncertainty can update the model. Larger residuals remain rejected.

`ChronosConfig::from_yaml()` parses and validates explicit direct/affine domain configuration, timestamp units, quality limits and observation policy. Configuration describes collection but does not start it. Linux clock types are re-exported from `robonix-chronos-clock`.

`QualityRuntime::start(chronos)` runs periodic quality evaluation on an owned worker. Read failures retain historical diagnostics and mark data unavailable; recovery clears the failure. Call `stop()` to join the worker. The library does not require Atlas, gRPC or ROS.

Run `cargo test -p robonix-chronos`.
