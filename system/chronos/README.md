# Chronos

`robonix-chronos` provides `ChronosCore`: a deterministic device-to-reference clock model library. Register a clock domain and policy, submit paired `ClockObservation` values, then query status, model coefficients or conversions. It does not read host clocks or run a process.

The core bounds its observation window, checks integer conversion overflow, tracks reference/device generations, and handles wrap, outliers, holdover and reset. Model updates support step and bounded slew.

Fitted rates outside the signed coefficient range are rejected, including a floating-point rate that rounds to the exclusive upper boundary.

Refreshing a new generation preserves retained models from earlier generations; their freshness cannot be computed from the new observations.

A strictly newer reference generation clears `Fault` to `Reset` and replaces its diagnostic reason. A retained model becomes `Unsynced` with its old generation identity until refitting. Equal-generation updates are idempotent, and reference availability recovery preserves `Fault`.

Outlier checks include the configured drift allowance over steady time since the last accepted observation, so permitted rate changes and observations within holdover uncertainty can update the model. Larger residuals remain rejected.

Run `cargo test -p robonix-chronos --test core`.
