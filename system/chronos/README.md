# Chronos

Chronos is Robonix's read-only source of clock quality and conversion models.
Use of its reports and conversions remains optional for consumers. The current
crate contains the privilege-free clock-domain core: it
validates paired observations, estimates versioned affine clock models, tracks
uncertainty and synchronization state, and converts device timestamps with
checked integer arithmetic.

The core is an in-process Rust library, with a Linux `robonix-chronos` gRPC
executable for deployments. Its first reference source
reads Linux `CLOCK_REALTIME` without modifying host time and brackets every read
with `CLOCK_MONOTONIC`. A `ros_time` reference subscribes to ROS 2 `/clock`;
PTP supervision is not implemented.

## Simulation reference

Select the reference source in the deployment manifest:

```yaml
system:
  chronos:
    core:
      reference:
        kind: ros_time
```

The default reader runs `python3 -u` and requires `rclpy` and
`rosgraph_msgs` in a sourced ROS environment. `reference_reader_command` under
`system.chronos` can select another interpreter, for example
`["/path/to/ros/python", "-u", "{script}"]`. `{script}` must be accessible to
that interpreter; a container needs the script directory mounted at the same
path. The same key is accepted under `chronos` in a standalone YAML config.
Keep the reader's stdin connected to its owner; container commands use
`docker exec -i`. Closing stdin stops the reader, including while paused.

Chronos reports unavailable reference time until the first `/clock` message.
Zero is valid. Time remains fixed during pauses and follows received clock
steps without extrapolation. A backward step starts a new generation;
reader reactivation, detected publisher loss followed by recovery, or daemon restart also
starts a new generation. The reader checks publisher presence separately from
timestamp progress and sends real health checkpoints. Missing publishers make
the reference unavailable; five real seconds without reader output fails the
reader. A present publisher that stops publishing is indistinguishable from a
legitimate pause. A same or forward world reset is also unobservable unless
detected publisher disappearance, reader restart or another detectable boundary occurs.
The latest reference reading is published through `robonix/system/chronos/reference_time`,
using the existing `ReferenceQuality` message. The reference publication
interval is at most 100 real milliseconds, independent of a slower quality
refresh setting. Motion duration checks still have sampling and transport
latency; the stream never extrapolates paused time. Publication and process health
continue on real time.
Atlas heartbeat eviction and communication deadlines use real steady time.
Simulated reference currently supports direct domains; affine
device-clock fitting against `ros_time` is rejected during configuration.
Restart the deployment to change the reference source.

## Run the gRPC component

Install with `make build-chronos` or build with `cargo build -p robonix-chronos`.
With Atlas running, start `robonix-chronos` for Linux-reference-only quality,
or pass `--config <yaml>` using the `chronos.core` format shown below.
It registers the shared `robonix/lifecycle/driver` and the four Chronos contracts,
then invokes INIT and ACTIVATE through its own Driver endpoint before reporting
readiness. ROS dependencies are required for `ros_time` or automatic ROS observation collection.
Atlas gRPC metadata references each source contract path, matching the other
Rust system components; the service and method routes come from codegen.

`rbnx boot` can launch it from this deployment block:

```yaml
system:
  atlas:
    listen: 127.0.0.1:50051
  chronos:
    listen: 127.0.0.1:50094
    provider_id: chronos
    core:
      reference:
        kind: linux_system
```

Options are `--config`, `--config-json`, `--atlas`, `--listen`, `--provider-id`,
and `--log`. `ROBONIX_ATLAS` supplies the Atlas endpoint; otherwise it defaults
to `127.0.0.1:50051`. CLI/env settings override manifest process settings;
manifest `core` overrides YAML `chronos.core`. Scribe uses `SCRIBE_LOG_DIR` and
`SCRIBE_FILE_LEVEL`, with `--log` or manifest `log` overriding the file level.
Use an explicit reachable host IP in `listen` for remote consumers; unspecified
bind addresses are advertised as loopback. Chronos is opt-in in deployments.

Consumers discover each contract with Atlas using gRPC, connect to the returned
endpoint, and call its generated server-stream method with an empty request.
Each connection starts with the latest cached snapshot. Quality is published
at the configured refresh cadence; models are published when domain/model
identity or quality changes and at least once per ten seconds, subject to that
cadence. Slow consumers receive the latest value instead of an unbounded backlog.
Clients reconnect and replace their cache after connection loss. These streams
are snapshots, not a complete event history, and time freshness remains a
consumer decision. Steady timestamps are local to the Chronos host, so remote
clients must not subtract them from their own monotonic clock; measure stream
age locally. An initial reference failure has `report_available=false`;
the model stream waits for the first real report without inventing a timestamp.

INIT starts the sole quality runtime and observation worker. ACTIVATE enables
observation input and publication; DEACTIVATE
ends streams while retaining the runtime and model identities. Reactivation
publishes current snapshots. SHUTDOWN and SIGINT/SIGTERM stop publication and
observation and quality workers and close the server. Changing core configuration
after INIT requires restarting the process. Configured device domains without
observations report initialization and no model. The executable accepts explicit
gRPC observations. Optional automatic ROS collection is described below.

At most eight Driver transactions can be outstanding, including work whose
caller has disconnected. Excess requests return `ok=false` with a retry message;
admitted transactions finish before releasing their slot. Domain IDs contain
1..128 UTF-8 bytes without surrounding whitespace; invalid IDs fail configuration
loading before collection starts. Helper stdout payloads are limited to 4096 bytes
while reading, including startup output. Oversized output stops and reaps the helper.

### Collect device observations

`chronos.collect_observations` is the automatic collection master switch and
`chronos.collect_ros_observations` explicitly enables ROS receipt. Both default
to false; deployment keys are under `system.chronos`. With the master switch on,
configured `query.adapter` domains use hardware queries. Domains without a query
adapter are subscribed only when the ROS switch is also true. Existing ROS-only
configurations must add the ROS switch; the master switch alone starts no ROS
helper. A query domain never falls back to ROS, even when ROS collection is enabled.
The public observation RPC remains available independently of these switches.

Prefer the driver's existing hardware clock interface with a configured query
adapter for device-uptime fitting. See the query configuration below. To collect
existing ROS sensor streams instead, explicitly configure:

```yaml
chronos:
  collect_observations: true
  collect_ros_observations: true
  core:
    # Existing reference and domain configuration.
```

ACTIVATE starts background discovery without waiting for devices, so `rbnx boot`
can bring Chronos up before Soma starts the drivers. Chronos serves reference
quality and model snapshots immediately; absent devices have no fitted model.
Each selected domain starts its own embedded Python helper when its source is
ready, without waiting for other domains. ROS discovery waits for the configured
ROS capability and ACTIVE driver. Waiting is logged at most once per ten seconds.
DEACTIVATE cancels discovery, stops and reaps every helper, and releases Atlas
connections; reactivation starts discovery again.
ROS subscriptions require no driver code or additional driver RPC. The installed
binary contains the helper, so it does not require the source checkout at runtime.
For ROS subscriptions, the parent environment must provide `python3`, `rclpy`, `rosidl_runtime_py`, and
the configured ROS message packages; source the matching ROS setup before boot.
With the default `rmw_fastrtps_cpp` middleware, the helper selects UDPv4 with
4 MiB send/receive socket buffer requests to improve large-image delivery.
The kernel may cap these requests. This setting applies only to the helper;
it does not alter the parent or sensor drivers. An inherited
`FASTRTPS_DEFAULT_PROFILES_FILE`, `FASTDDS_DEFAULT_PROFILES_FILE`, or
`FASTDDS_BUILTIN_TRANSPORTS` takes precedence; other middleware is unchanged.
Chronos owns the temporary XML file and removes it after reaping the helper,
including when the helper is forcibly terminated. This does not load ROS automatically or remove
capture-to-receive latency from fitted models.

Invalid static collection settings fail activation explicitly. Missing runtime
dependencies or subscription startup failures are reported by the background
worker and terminate Chronos; device absence alone does not fail startup.
An unexpected helper exit is logged and terminates Chronos after stopping all
other helpers and pending discovery, rather than leaving collection silently stopped.

For each ROS-collected affine domain, configure exactly one existing `topic_out` capability
using its existing `provider_id`, `contract_id`, `transport: ros2`, and
`timestamp_field`. Chronos connects through Atlas, checks a declared time domain
when present, and gets the message type from the contract. Direct domains are
not collected. Drivers must already publish device timestamps on the chosen
stream; subscribing cannot turn game time or reference time into device uptime.

Optional dotted paths `session_id_field`, `device_generation_field`, and
`sequence_id_field` extract existing capture identity fields. Configure all three
together or omit all three, so a helper restart cannot restart a local counter
inside an unchanged native identity. Timestamp fields
accept integer nanoseconds or ROS Time (`sec`, `nanosec`); no float conversion
is used. Without identity fields, the helper uses a unique collection session,
generation 1, and a local sequence, suppressing equal timestamps. A new helper
session resets the fit after reactivation. This cannot reliably detect every
physical device restart. Unexplained timestamp regression stops that domain
with an error; restart collection only after checking the source. Automatic
collection currently rejects configured wrapping counters. A domain failure ends its helper and terminates Chronos after cleanup.

Subscriptions use depth one and retain at most one latest sample per domain.
At ROS callback entry, before extraction or locking, the helper brackets a host
realtime read with monotonic reads and forwards that pair with its read
uncertainty. The owned helper shares the host's Linux clocks with Chronos.
Each helper forwards one observation at a time to the existing shared bounded
observation worker. Samples older than 100ms are discarded at both helper and parent;
worker replies have a 100ms deadline, and overload drops the sample instead of
replaying it. Future clock pairs, excessive read uncertainty, inconsistent
reference readings, and samples predating a detected reference-generation change
are rejected. Core policy
rejections are logged at debug level; transport drops and domain failures are
reported. Do not run another observation producer for the same domain.
Fitting keeps the callback clock pair; extraction, helper queues, IPC and fitting
queues do not move the paired reference time. Sensor transport, ROS deserialization
and scheduling before the callback still contribute to the pairing, so this
remains a best-effort arrival model, not a guarantee of capture-time accuracy.
The public observation RPC is unchanged and samples reference time at RPC ingress;
it does not accept caller-supplied host times.

Run dependency-free extraction tests with
`python3 -m unittest discover -s system/chronos/tests -p test_collector.py`.
Run the real ROS/Atlas/gRPC process checks, including startup rollback and helper
failure, from a sourced ROS environment with
`cargo test -p robonix-chronos --test collector_process -- --ignored --test-threads=1`.
After `cargo build --workspace`, verify actual `rbnx boot/shutdown` and late
device activation with `python system/chronos/tests/rbnx_boot_smoke.py` from
the repository root. This opt-in smoke needs a sourced ROS environment and
`grpcio`, `grpcio-tools`, and `PyYAML` in the invoking Python interpreter;
it launches isolated Atlas/Chronos and a synthetic stationary camera, creates
artifacts under `/tmp`, and never starts Minecraft.

`examples/minecraft-camera.yaml` is a first test configuration for the existing
Minecraft camera capability. It requires that capability to already carry
host-monotonic device uptime (`primary_timestamp_mode: device`); the separate
raw camera topic is not registered as this Atlas capability. The example does
not change the deployment's game/reference timestamp settings or start the game.
With the source running, use `robonix-chronos --config
system/chronos/examples/minecraft-camera.yaml`. Its uncertainty and fit limits
are generous software-test settings, not measured capture accuracy.
For one-command deployment, place that file's `chronos` mapping under
`system.chronos` in the robot manifest and run `rbnx boot -f <manifest>` from
the same ROS environment. Chronos may start before the camera registers.

### Submit device observations

Discover `robonix/system/chronos/observe_device_timestamp` through Atlas and call
the generated `ObserveDeviceTimestamp` RPC on the same server endpoint. Its
`chronos/srv/ObserveDeviceTimestamp.srv` request contains `domain_id`, `session_id`,
`device_generation`, `sequence_id`, and integer `device_timestamp_ns`. Send only
raw device time: reference truth, injection parameters and sender monotonic time
are not inputs. The domain must be configured with `affine_model`; existing
ROS capability bindings become input subscriptions only when both collection
switches are enabled.

Chronos samples its own reference after decoding/admission, before any async wait
or bounded serial fitting queue. If the lifecycle or core owner is busy, it rejects
the request rather than pairing with a later reference. At most 64 observations
can be outstanding. `accepted=true`
means the core accepted the observation; policy rejections return `accepted=false`
with `reason`. Malformed requests/unknown domains return `INVALID_ARGUMENT`,
duplicates/stale identities return `FAILED_PRECONDITION`, overload returns
`RESOURCE_EXHAUSTED`, and inactive/stopped service returns `UNAVAILABLE`. Retry
overload with a fresh observation rather than replaying an old capture. A lost
RPC response leaves acceptance uncertain; duplicate sequence retries fail
explicitly instead of fitting twice. Arrival pairing includes upstream network,
encoding and capture latency. Unknown systematic delay cannot be distinguished
from clock offset; configure observation uncertainty conservatively. The RPC
does not promise capture-time accuracy or external synchronization.

Use one producer per configured domain, a stable session ID per device session,
positive non-decreasing device generations and increasing sequences within an
identity. IDs are limited to 128 bytes. A new session or higher generation resets
the domain once; its boundary sample returns `accepted=false` because reset
invalidates the entrance token. The next fresh sample starts the new fit, without
silently replacing the original receive time by a later clock read. Retired
sessions cannot reclaim the domain; history is bounded to 64 retired sessions
per domain, after which restart is required. Source identities survive
DEACTIVATE/ACTIVATE, and queued inputs from old activations are cancelled.
DEACTIVATE waits for already-started inputs before returning.
Consumers must compare the current domain generation with retained model
identity, since old coefficients remain available during reset and degradation.

## Use

For basic in-process time readings, use the dependency-independent
[`robonix-chronos-clock`](clock/README.md) library. It owns `ReferenceConfig`,
`LinuxSystemReference`, and `VirtualClock`, requires no running service, and
does not depend on Scribe, Atlas, Codegen, or gRPC. This package re-exports the
existing clock types and configuration error, preserving consumer imports.
Device models, continuous quality evaluation, and service lifecycle remain here.

Add `robonix-chronos` as a Rust dependency, parse a `ChronosConfig`, and create
the in-process `Chronos` API. It owns the virtual clock and pure clock-domain
core, registers configured affine domains, and keeps reference-generation and
freshness updates ordered. Call `refresh` for a coherent
`ClockQualityReport`. For a configured device-uptime domain, take a `snapshot`
at message ingress and pass it to `observe_device_timestamp_at`; the convenience
`observe_device_timestamp` method is suitable when it is itself called at
ingress. The returned observation snapshot is opaque, bound to the issuing
`Chronos` instance, and rejected after the reference generation changes. A
queued snapshot pairs one observation but cannot roll global quality backward.
The configured sampling interval rejects observations that arrive too soon
with an explicit error. Call `model` to obtain owned coefficients or `convert`
for a low-frequency best-effort conversion. Both refresh and apply reference
quality before returning. Direct domains deliberately expose reference
evidence but no conversion model.

`direct` assumes the driver already supplies timestamps that consumers can use
in the declared time scale. Driver-side timestamp uncertainty is unknown;
additional direct-domain quality management is not implemented. Chronos does
not measure or manage the driver's alignment error, drift, or timestamp
freshness. Quality snapshots expose reference-clock evidence and add the
configured numeric timestamp uncertainty to reference uncertainty. That number
is a configuration assumption, not a measured or verified direct-domain bound.

Wrap the configured `Chronos` in `QualityRuntime::start` to run continuous
in-process evaluation. The runtime evaluates once before returning, refreshes
on the configured cadence, and exposes `latest_quality_status` without a clock
read. It delegates the observation, model, conversion, reset, and virtual-clock
APIs through one serialized owner and stops promptly when `stop` is called or
the value is dropped.

`quality_snapshot` and `models_snapshot` expose transport-neutral publication
structures. The quality snapshot separates direct and affine domains and keeps
an initial refresh failure representable without fabricating reference time.
The aggregate model snapshot includes current domain identity plus optional
retained coefficients with their own generation and version, so consumers can
detect parameters that predate a device reset or reference-clock step. Matching
ROS-style IDL message definitions live under `capabilities/lib/chronos/msg`.
The contracts under `capabilities/system/chronos` define three output streams:

| Contract | IDL message |
| --- | --- |
| `robonix/system/chronos/reference_time` | `chronos/msg/ReferenceQuality.msg` |
| `robonix/system/chronos/quality` | `chronos/msg/ClockQualitySnapshot.msg` |
| `robonix/system/chronos/conversion_models` | `chronos/msg/ClockModelsSnapshot.msg` |

All use `topic_out`, which codegen maps to a gRPC server stream with an empty
request and snapshot messages as responses. The executable serves them over
gRPC; these definitions do not create ROS topics.

`QualityRuntimeStatus` distinguishes a current report from retained evidence.
On any refresh failure it preserves the latest report and conversion
coefficients, marks their current quality unavailable and unbounded, and
records the error text, consecutive failure count, first and latest observed
steady times, and last successful refresh time. `model` and `convert` continue
with the retained quality-adjusted model. Warning logs summarize refresh
failure, reference unavailability, and every `UNSYNCED` or `FAULT` domain; an
unchanged warning is rate-limited with a monotonic clock while a changed
condition is reported immediately.

An unexpected quality-worker panic makes status queries and delegated
operations return an error instead of continuing through potentially
inconsistent mutable state. It is also reported by `stop`; dropping the runtime
logs that failure.

Degraded state is quality metadata rather than a conversion gate: when retained
coefficients still define an unambiguous value, conversion returns a
best-effort estimate with its state, uncertainty, domain generation, reference
generation, model version, validity flag, and last observation time. Consumers
decide whether to use it. Only a missing model, ambiguous wrapping counter,
invalid raw value, or arithmetic overflow prevents calculation. Non-wrapping
clocks may be extrapolated beyond the fitted validity interval; such results
set the validity flag to false and uncertainty to `u64::MAX`.

An unavailable reference does not discard retained coefficients. Affine models
become `UNSYNCED` with unbounded uncertainty, and new observations are rejected
until reference sampling recovers. Conversion remains best effort and carries
that degraded state for the consumer to evaluate.

Call `Chronos::reset_device_domain` when a device restart is confirmed; this
creates a new generation and permits a restarted sequence counter. A declared
counter wrap is extended automatically and is not guessed to be a restart.

The reference source is selected by validated YAML. This example uses
`linux_system`; see Simulation reference above for `ros_time`:

```yaml
chronos:
  core:
    reference:
      kind: linux_system
      max_uncertainty_ns: 1000000
      max_status_age_ns: 2000000000
      max_clock_step_ns: 100000000
    quality:
      # Refresh the in-process quality report once per second.
      refresh_interval_ns: 1000000000
      # Repeat a continuing degraded-quality warning at most every ten seconds.
      warning_interval_ns: 10000000000
    domains:
      front_camera_clock:
        capabilities:
          - provider_id: front_camera
            contract_id: robonix/primitive/camera/rgb
            transport: ros2
            timestamp_field: header.stamp
        timestamp:
          event: acquisition
          hardware: true
          time_scale: ros_time
          resolution_ns: 1
          uncertainty_ns: 1000000
        strategy:
          kind: direct
          aligned_to: linux_system
      lidar_uptime_clock:
        capabilities:
          - provider_id: front_lidar
            contract_id: robonix/primitive/lidar/lidar
            transport: ros2
            timestamp_field: header.stamp
        timestamp:
          event: acquisition
          hardware: true
          time_scale: device_uptime
          resolution_ns: 1000
          wrap_period_ns: null
          uncertainty_ns: 1000
        strategy:
          kind: affine_model
          observation:
            reference_at: host_receive_time
            sampling_interval_ns: 500000000
            fixed_delay_ns: 0
            uncertainty_ns: 5000000
          policy:
            minimum_observations: 8
            observation_window: 32
            max_drift_ppm: 200
            max_residual_ns: 10000000
            tracking_timeout_ns: 2000000000
            holdover_duration_ns: 5000000000
            max_uncertainty_ns: 20000000
            model_update: {kind: step}
```

Within `affine_model`, observations can come from ROS receipt or a configured
hardware query adapter. Omitting `query` retains `reference_at: host_receive_time`; automatic ROS
callbacks additionally require `collect_ros_observations: true`. Queries require
`collect_observations: true` but do not require the ROS switch. To register a query adapter for a domain:

```yaml
# Under that domain's strategy.observation:
reference_at: host_query_midpoint
sampling_interval_ns: 500000000
uncertainty_ns: 1000000
query:
  adapter: my_camera.clock:create
  tick_ns: 1000
  timeout_ns: 50000000
  options:
    serial: "camera-1"
```

For a u32 microsecond counter, also declare `timestamp.wrap_period_ns:
4294967296000` and `resolution_ns: 1000`. The data acquisition timestamp and
queried counter must belong to the same device clock and use the same raw
nanosecond representation. Query adapters multiply ticks by `tick_ns`; they
never unwrap counters. Chronos expands declared wraps. Sampling interval plus
query timeout must stay below half the wrap period, allowing for maximum drift.
An ambiguous gap or regression requires confirmed restart evidence or a domain
reset; do not infer a reboot from the counter alone.

Adapters are ordinary Python modules installed in the collector's Python
environment or located on `PYTHONPATH`. Keep vendor code in the corresponding
driver package, with a separate module and tests for each hardware family. The
configured `module:factory` receives a dictionary of string options and returns
an object implementing `read()` and `close()`. Only selected modules and their
SDK dependencies are imported. `read()` returns `chronos_query.QuerySample`:
raw unsigned ticks, reference/steady times before and after the actual query,
positive device generation, and additional uncertainty in nanoseconds. The
embedded collector supplies the dependency-free `chronos_query` module;
`capture_query(read_ticks, device_generation=1, uncertainty_ns=1)` brackets a
local operation. A confirmed reboot must increase `device_generation`.

For drivers holding an exclusive device handle, keep the hardware query inside
that driver. Its collector adapter can call the driver's existing interface and
construct `QuerySample` from the actual query bracket on the same Chronos host.
Remote host clock values cannot be passed as local clock evidence; bracket the
exchange locally or explicitly translate them. This adds no hardware-specific
RPC or Atlas contract. Registration here is the per-domain configuration, not
automatic detection of arbitrary vendor APIs.

For standalone SDK access, a query domain may omit `capabilities` or use an
empty list; its adapter owns device connection and startup errors. For queries
through a driver, bind an existing capability with its `provider_id`,
`contract_id`, and `transport: grpc` or `ros2`. Query discovery waits for all
configured bindings to be declared and their drivers to become ACTIVE;
`timestamp_field` may be omitted because query mode does not extract messages.
For example, a driver adapter may wait on the existing `robonix/lifecycle/driver`
gRPC capability. The adapter still owns calling the driver's actual time
interface; this binding does not create a new RPC. Query bindings need not be
`topic_out`, and query domains open no ROS subscriptions or Atlas consumer edges.
They work without rclpy. Samples use the query midpoint, include half the bracket duration and
additional error in uncertainty, and retain the existing bounded IPC and model
worker. Query mode rejects nonzero `fixed_delay_ns` and the timestamp-only
`ObserveDeviceTimestamp` RPC, which cannot carry query clock evidence.

Timeouts must be between 1 and 50000000 ns. A read error stops that domain and is
logged; there is no silent ROS fallback. A blocked read stops that domain's helper; the daemon then cancels all collection
and reaps every helper to release device resources.
A thread cannot safely cancel a vendor SDK call. Lifecycle deactivation and
shutdown also reap the helper.

See [the adapter template](examples/query_adapters/template.py) and
[the RealSense example](examples/query_adapters/realsense.py). The latter uses
`get_device_time_ms()` with an explicit serial and converts milliseconds to raw
microsecond ticks; its vendor import is optional. It is a standalone query
example, not validated against attached hardware. The current RealSense ROS
wrapper publishes host-mapped acquisition time with `global_time`; use `direct`
for those headers. An affine query deployment needs raw device acquisition
timestamps from the driver, not those host-mapped headers. If sharing control
access with the existing driver is unsupported, adapt through that driver's
owned handle instead of opening another connection.

Validate extraction/query behavior with
`python3 -m unittest discover -s system/chronos/tests -p test_collector.py`.
`cargo test -p robonix-chronos --test collector_process
hardware_query_collection_is_owned_by_chronos` runs an isolated query-only
process test with a simulated u32 counter, no data capability declarations, and
lifecycle cleanup; the ignored
ROS process tests still require a sourced ROS environment.

`policy.model_update` defaults to `{kind: step}`: each accepted fit is published
immediately, including backward corrections. This replaces the former behavior
that retained forward bias to prevent a backward correction.

To limit offset corrections at model updates, use:

```yaml
model_update:
  kind: slew
  max_correction_ppm: 1000
```

The rate must be between 1 and 1000000 ppm. At 1000 ppm, each elapsed steady
second permits up to 1 ms of positive or negative offset correction at the
latest sample. Fitted clock rates still update normally; this does not guarantee
monotonic results across arbitrary conversions. Correction advances only on
accepted observations, and the remaining offset is added to model uncertainty.
Device or reference resets start a new fit without smoothing across generations.
Neither mode changes the operating system clock. Arrival-delay bias still needs
to be addressed at the source or through delay compensation.

Parse this with `ChronosConfig::from_yaml` and pass it to `Chronos::new`.
`Chronos::refresh` samples reference and steady time together, propagates any
reference discontinuity into the models, refreshes domain freshness, and stores
one quality report. `Chronos::reference_time` provides the sampled virtual
reference value, while `Chronos::steady_time` is available independently for
durations and deadlines. Steady time is never a business timestamp and Chronos
does not hide reference-clock steps behind it.

Start and query continuous evaluation with:

```rust,ignore
let chronos = Chronos::new(ChronosConfig::from_yaml(yaml)?)?;
let runtime = QualityRuntime::start(chronos)?;
let status = runtime.latest_quality_status()?;

// Observation and conversion methods remain available through `runtime`.
runtime.stop()?;
```

A reference step or regression restarts fitting for every registered affine
domain while retaining the last model with its old reference-generation
identity. Linux system time is reported as available when fresh and within its
uncertainty limit, but remains unverified until a later time-service observer
confirms its source.

Domain configuration is explicit. `direct` declares that consumers may keep
the original timestamp while Chronos reports its configured relationship;
`affine_model` describes a device-uptime domain that requires paired
observations. The lower-level `ChronosCore` and `VirtualClock` APIs remain
available for embedding that needs to manage sampling and registration itself.
The library does not subscribe to data itself; the executable's optional helper
queries configured hardware adapters and optionally subscribes to ROS 2
capabilities. The runtime keeps the
latest report in process and logs warnings. The executable distributes these
snapshots over gRPC; ROS 2 and Vitals publication are not implemented.

Chronos does not sit between a driver and its consumers. Drivers continue to
publish normally, consumers may ignore Chronos, and later local adapters can
cache models for conversion without synchronously depending on the core.

Run the core tests from the repository root:

```bash
cargo test -p robonix-chronos --all-targets
```

## TODO

- Investigate whether further `direct` quality management is needed, including
  driver-provided evidence and an explicit representation of unknown timestamp
  uncertainty, then implement it if justified.
