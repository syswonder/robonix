# Scribe - structured logging facade

Scribe is the common logging library used by Robonix system components. It is
implemented today as the Rust crate `robonix-scribe`, not as a standalone
service process.

## Current status

Scribe provides a process-local logging facade:

- `robonix_scribe::init(tag)` sets the default component tag once at startup.
- `robonix_scribe::init_from_config(tag, config_json)` also reads the
  component launch config's top-level `log` field and applies it to the file
  sink level.
- `info!`, `warn!`, `error!`, and `debug!` macros write structured records.
- Each record goes to stderr and to a per-tag JSON-lines file.

Rust Scribe reads wall timestamps through the in-process Chronos clock library;
logging does not require a running Chronos service. Unverified Linux time is
accepted. Simulation pauses and resets do not change log dates. A successful
reading with unavailable wall-clock quality retains its
timestamp and adds the quality reason to `msg`. If the read fails or its value
cannot fit the unsigned timestamp, Scribe writes the original level, tag,
message, and error directly to stderr, even for a file-only level, and skips
the structured file record. It neither fills in zero nor retries through a
different clock. Fully filtered messages still produce no output.

`LogRecord::now(...)` now returns `std::io::Result<LogRecord>`; callers that
construct records directly must handle errors. The logging functions and
macros retain their existing signatures. Clock reads currently require Linux;
unsupported platforms follow the same explicit error path.

There is no central Scribe daemon, query API, replay service, retention policy,
or Sentinel audit integration on `dev` yet.

## Output

The file sink writes under `$SCRIBE_LOG_DIR`, or `./logs` when the environment
variable is unset. Components launched by `rbnx boot` normally receive a log
directory from the boot environment, so their Scribe output is collected with
the rest of the component logs.

Each JSON-lines record carries:

- `ts`: local-time timestamp with nanosecond precision,
- `level`: `debug`, `info`, `warn`, or `error`,
- `tag`: component or provider identifier,
- `msg`: free-form message text.

## Usage

```rust
fn main() {
    robonix_scribe::init("executor");
    robonix_scribe::info!("executor starting");
}
```

For binaries launched with a manifest config JSON, prefer:

```rust
robonix_scribe::init_from_config("executor", config_json.as_deref());
```

This keeps the manifest-level `log` setting consistent with what reaches the
on-disk Scribe files.

## Intended future role

The longer-term Scribe component can still grow into the durable system journal
described by the architecture docs: Atlas lifecycle events, Executor RTDL
events, Sentinel rule decisions, component health, query, and replay. That is
not the current implementation, so new docs should not describe Scribe as an
available replay/audit service yet.

Run `cargo test -p robonix-scribe --test clock_failures` on Linux with `cc`
for isolated clock-failure tests of stderr and structured log output.
