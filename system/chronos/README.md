# Chronos

The Linux `robonix-chronos` daemon publishes cached reference-time and clock-quality snapshots over gRPC. It registers with Atlas and uses the shared Driver lifecycle. Clients discover the contracts, connect with generated stubs and send an empty request to start a server stream. Each stream starts with the latest cached snapshot; slow readers receive current values instead of an unbounded history.

Fitted rates outside the signed coefficient range are rejected, including a floating-point rate that rounds to the exclusive upper boundary.

Refreshing a new generation preserves retained models from earlier generations; their freshness cannot be computed from the new observations.

A strictly newer reference generation clears `Fault` to `Reset` and replaces its diagnostic reason. A retained model becomes `Unsynced` with its old generation identity until refitting. Equal-generation updates are idempotent, and reference availability recovery preserves `Fault`.

Outlier checks include the configured drift allowance over steady time since the last accepted observation, so permitted rate changes and observations within holdover uncertainty can update the model. Larger residuals remain rejected.

`ChronosConfig::from_yaml()` parses and validates explicit direct/affine domain configuration, timestamp units, quality limits and observation policy. Configuration describes collection but does not start it. Linux clock types are re-exported from `robonix-chronos-clock`.

`QualityRuntime::start(chronos)` runs periodic quality evaluation on an owned worker. Read failures retain historical diagnostics and mark data unavailable; recovery clears the failure. Call `stop()` to join the worker. The library does not require Atlas, gRPC or ROS.

Start Atlas, then run `cargo run -p robonix-chronos -- --atlas 127.0.0.1:50051 --listen 127.0.0.1:50094`. `--config` loads Chronos YAML and `--config-json` supplies inline configuration. `--provider-id`, `--log`, and `ROBONIX_ATLAS` select identity, logging and discovery. Linux reference is supported; ROS input and automatic collection are explicitly rejected at activation. Device models remain available through the independent in-process library.

INIT constructs the runtime, ACTIVATE starts publication, DEACTIVATE ends existing streams, and SHUTDOWN joins workers and closes the server. Shutdown and communication budgets use real time. Snapshot quality retains read failures instead of inventing timestamps.

Pilot discovers `robonix/system/chronos/get_wall_time` through Atlas. Executor calls it over MCP at `/mcp` on the same listener as gRPC. The capability takes no arguments and returns `datetime` in RFC 3339. Chronos uses the host time zone unless `TZ` is set in its environment. The call returns an error if Chronos is inactive or cannot read the time or resolve its time zone.

Run `cargo test -p robonix-chronos`; `grpc_process` launches a separate daemon and exercises discovery, repeated stream delivery and lifecycle shutdown. Rebuild stubs from the Chronos contracts and message IDL before connecting.
