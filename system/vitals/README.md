# robonix-vitals — health monitoring

Monitors robot power state and component health (temperature, voltage), normalises
readings against per-robot-model threshold tables, and exposes the result via two
gRPC capabilities:

| contract | mode | transport | port | consumers |
|-|-|-|-|-|
| `robonix/service/vitals/get` | rpc | gRPC | 50091 | pilot, sentinel, liaison |
| `robonix/service/vitals/stream` | server\_stream | gRPC | 50091 | sentinel, liaison |

Vitals **reports only** — it does not decide what to do on WARN/ERROR. Safety
actions are the responsibility of `sentinel`.

## Architecture

```
sysfs / hwmon
      │
      v
health_primitive (50092)               ← hardware abstraction
  robonix/primitive/health/state        ← unified gRPC contract
      │
      │  Atlas discovery
      v
vitals (50091)                         ← normalise + threshold + report
  robonix/service/vitals/get
  robonix/service/vitals/stream
      │
      v
pilot / sentinel / liaison
```

Vitals does **not** read sysfs directly. It discovers a `health_primitive` provider
through Atlas (`connect_to_capability`), then calls `GetHealthState` on the unified
`robonix/primitive/health/state` contract. The primitive absorbs hardware differences;
vitals only sees unified fields.

## Quick start

```bash
# Terminal 1 — start the health primitive
robonix-health-primitive --log info
# Or
cargo run --release -p robonix-health-primitive -- --log info

# Terminal 2 — start vitals (discovers the primitive via Atlas)
robonix-vitals --log info
# Or
cargo run --release -p robonix-vitals -- --log info
```

Typical output:

```
[vitals] connected to health primitive via Atlas
[vitals] 19.8V | cpu:OK(35) tj:OK(36) soc012:OK(36) soc345:OK(35) nvme:OK(38) battery:OK(-1)
```

## Data flow

```
health_primitive:GetHealthState  →  GrpcCollector.collect()
                                         │
                                    (PowerState, Vec<RawReading>)
                                         │
                                   normalize.rs (threshold check)
                                         │
                                    VitalsSnapshot (cached)
                                         │
                              ┌──────────┴──────────┐
                              v                     v
                       GetVitals (rpc)     StreamVitals (server_stream)
                       returns cache       pushes on state transitions
```

- **collect.rs** (`GrpcCollector`) calls the health primitive via gRPC and
  converts `HealthState` → `(PowerState, Vec<RawReading>)`.
- **normalize.rs** compares raw readings against the YAML threshold table and
  produces `ComponentHealth` entries with OK / WARN / ERROR status.
- **service.rs** caches the latest `VitalsSnapshot`, serves `GetVitals`, and
  broadcasts on `StreamVitals` only when a component health state transitions
  (OK→WARN, WARN→ERROR, etc.). Steady-state temperature drift does **not**
  trigger a push.

## CLI reference

```
robonix-vitals [FLAGS] [OPTIONS]
```

| Flag / Option | Env | Default | Description |
|-|-|-|-|
| `--atlas` | `ROBONIX_ATLAS_ENDPOINT` | `127.0.0.1:50051` | Atlas control-plane endpoint |
| `--listen` | `ROBONIX_VITALS_LISTEN` | `127.0.0.1:50091` | gRPC listen address |
| `--id` | `ROBONIX_VITALS_PROVIDER_ID` | `vitals` | Provider id registered with Atlas |
| `--collect-interval-ms` | `ROBONIX_VITALS_COLLECT_INTERVAL_MS` | `1000` | Sensor polling interval (ms) |
| `--thresholds-path` | `ROBONIX_VITALS_THRESHOLDS_PATH` | `<crate>/thresholds/jetson_agx_orin.yaml` | YAML threshold file |
| `--config` | `ROBONIX_CONFIG_PATH` | — | Optional YAML config file (CLI/env override) |
| `--log` | `RUST_LOG` | `robonix_vitals=info` | env\_logger filter |

## Threshold file format

```yaml
robot_model: "jetson_agx_orin"
components:
  - name: "cpu"
    warn_above_c: 80.0                # °C — trigger WARN  (optional)
    error_above_c: 90.0               # °C — trigger ERROR (optional)
    # Battery-style rules (optional):
    # warn_below_percent: 20.0
    # error_below_percent: 5.0
    # warn_below_voltage: 11.0
    # error_below_voltage: 10.0
```

Component `name` must match the `name` field in the health primitive's
`SensorReading`. Thermal zone names are stripped of the `-thermal` suffix
by the primitive (e.g. `cpu-thermal` → `cpu`).

## Source layout

```
system/vitals/
  Cargo.toml
  build.rs
  thresholds/
    jetson_agx_orin.yaml
  src/
    main.rs          # Atlas registration, discovery, gRPC serve, collect loop
    config.rs        # VitalsConfig
    service.rs       # VitalsServiceImpl: GetVitals + StreamVitals + state-change detection
    collect.rs       # GrpcCollector: gRPC client for health primitive
    normalize.rs     # evaluate() + load_thresholds() + unit tests
    pb.rs            # include!(contract_proto_modules.rs)
```

## Verifying

```bash
cargo build -p robonix-vitals
cargo clippy -p robonix-vitals -- -D warnings
cargo test -p robonix-vitals

# Must have health_primitive running first
grpcurl -plaintext -d '{}' 127.0.0.1:50091 \
  robonix.contracts.RobonixServiceVitalsGet/GetVitals
```
