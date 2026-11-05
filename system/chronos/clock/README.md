# Chronos clock

In-process Linux wall and steady clock readings with uncertainty and clock generations. No running service, Atlas or logging dependency is required.

Use `reference_time()`, `steady_time()`, `wall_time()` or `snapshot()` from `robonix_chronos_clock`. Reads return errors instead of fabricated timestamps. Steady timestamps are local to the host.

For caller-supplied simulation time, call `configure_reference()` with `ReferenceSourceKind::RosTime` before starting consumers, then inject `VirtualReferenceTime` readings with `update_reference_time()`. Zero is valid; paused timestamps are retained without extrapolation. Cached readings become unavailable after `max_status_age_ns` without another injection. This library does not subscribe to ROS.

Injection applies the process-wide `max_uncertainty_ns` limit before updating the last available timestamp used to reject backward time in the same generation. Unavailable readings preserve that timestamp. Older generations are rejected, and backward time requires a newer generation. The first actual reading establishes the generation without reporting a change. Generation changes remain observable after a failed snapshot read.

Linux reference sampling uses `CLOCK_BOOTTIME` to bound uncertainty and detect wall-clock steps, so suspend alone does not change generation. Snapshot steady time uses the enclosing `CLOCK_MONOTONIC` midpoint; `steady_time()` also uses MONOTONIC, which excludes suspend. Freshness is evaluated at sampling completion. Poisoned reference ownership is reported separately from OS clock-read errors.

`wall_timestamp_ns()` returns a valid timestamp even when sampling uncertainty exceeds the quality limit. It writes a warning directly to stderr once until a healthy read occurs; no logger is required. `wall_time()` retains quality metadata for callers to inspect. Actual clock-read failures and out-of-range timestamps still return errors. `reference_timestamp_ns()` continues to reject unavailable reference time.

Run `cargo test --locked -p robonix-chronos-clock -- --nocapture` and `cargo clippy -p robonix-chronos-clock --all-targets -- -D warnings`. Linux integration tests require `cc` for the isolated clock-failure fixture. macOS runs the deterministic unit tests; runtime clock reads require Linux.

## Python

The Python package `robonix-chronos-clock` exposes the Rust implementation as
`robonix_chronos_clock` inside the Python process. It has no Python runtime
dependencies. Rust callers can leave the optional `python` Cargo feature disabled.

```python
import robonix_chronos_clock as clock

reading = clock.reference_time()
snapshot = clock.snapshot()
steady_ns = clock.steady_time().timestamp_ns
reference_ns = clock.reference_timestamp_ns()
wall_ns = clock.wall_timestamp_ns()
```

`VirtualReferenceTime`, `VirtualSteadyTime`, and `VirtualClockSnapshot` expose
read-only fields matching their Rust types. Timestamps remain integer nanoseconds.
`reference_time()` and `snapshot()` retain unavailable quality as metadata;
`reference_timestamp_ns()` rejects unavailable or negative time. Read failures
raise `OSError`; an invalid source name raises `ValueError`. Linux reference time
remains unverified, and wall reads retain the warning behavior described above.

On Linux, `clock.configure_reference("ros_time")` selects injected reference
time with the default quality limits and discards previous cached readings.
Call `clock.update_reference_time(timestamp_ns, generation, available,
uncertainty_ns=1, verified=False, reason="")` to install received readings.
The caller owns the ROS subscription and the supplied quality metadata.
These operations affect only the current process; they do not synchronize
generation counters or caches across processes. Wall and steady reads remain
independent of the selected reference source.

Build a wheel from the repository root:

```bash
uvx maturin build --release --manifest-path system/chronos/clock/Cargo.toml \
  --out /tmp/chronos-wheels
```

The wheel uses CPython's 3.10+ stable ABI on standard GIL-enabled CPython.
Build separately for each platform and architecture; source builds require
Rust and maturin. Install the wheel into the Python environment that will
read time, then run `python -m unittest discover -s system/chronos/clock/tests -p test_python.py`.
The failure tests require Linux and `cc` and never change host time.
