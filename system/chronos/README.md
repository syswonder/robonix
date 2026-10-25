# Chronos

`robonix-chronos` provides `ChronosCore`: a deterministic device-to-reference clock model library. Register a clock domain and policy, submit paired `ClockObservation` values, then query status, model coefficients or conversions. It does not read host clocks or run a process.

The core bounds its observation window, checks integer conversion overflow, tracks reference/device generations, and handles wrap, outliers, holdover and reset. Model updates support step and bounded slew.

Refreshing a new generation preserves retained models from earlier generations; their freshness cannot be computed from the new observations.

Outlier checks include the configured drift allowance over steady time since the last accepted observation, so permitted rate changes and observations within holdover uncertainty can update the model. Larger residuals remain rejected.

Run `cargo test -p robonix-chronos --test core`.
