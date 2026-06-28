# robonix-vitals — health monitoring

Monitors robot platform health (temperature, voltage) and body health (joint
motors), normalises readings against threshold tables, and exposes the result
via two gRPC capabilities:

| contract | mode | transport | port | consumers |
|-|-|-|-|-|
| `robonix/service/vitals/get` | rpc | gRPC | 50091 | pilot, sentinel, liaison |
| `robonix/service/vitals/stream` | server\_stream | gRPC | 50091 | sentinel, liaison |

Vitals **reports only** — it does not decide what to do on WARN/ERROR. Safety
actions are the responsibility of `sentinel`.

## Architecture

```
sysfs / hwmon / thermal          body SDK (Piper/Go2/...)
      │                                │
      │ SysfsCollector                 │ BodyCollector (Python subprocess)
      ▼                                ▼
                    vitals
              (aggregate + threshold)
                      │
          GetVitals / StreamVitals
                      │
                      ▼
                    pilot
```

In v0.1 vitals reads hardware directly. When **Soma** is ready, both collectors
become gRPC clients consuming Soma's unified interfaces — the data structures
stay the same.

## Quick start

```bash
# Board-only (Jetson):
robonix-vitals --log info

# Board + body (Piper arm via roboarm venv):
ROBONIX_VITALS_BODY_PYTHON=/path/to/roboarm/.venv/bin/python3 \
robonix-vitals --log info --body-type arm --body-model piper
```

Typical output:

```
[vitals] sysfs collector ready
[vitals] body collector ready (arm/piper)
[vitals] 19.8V | cpu:OK(35) tj:OK(36) nvme:OK(38) | arm[piper]:NORMAL
```

## Data flow

```
SysfsCollector.collect()  →  PowerState + ComponentHealth[]  (board)
BodyCollector.collect()   →  BodyHealth                      (joints, optional)
                                         │
                                   normalize.rs (threshold check)
                                         │
                                    VitalsSnapshot
                                         │
                              ┌──────────┴──────────┐
                              v                     v
                       GetVitals (rpc)     StreamVitals (server_stream)
```

- **collect.rs** (`SysfsCollector`) reads thermal zones, NVMe, and system voltage
  from sysfs/hwmon.
- **body.rs** (`BodyCollector`) spawns a Python subprocess that calls the body SDK
  (e.g. `piper_body.py` → `piper_sdk`). Communicates via stdin/stdout JSON.
- **normalize.rs** compares readings against the YAML threshold table.
- **service.rs** caches the latest `VitalsSnapshot`, serves `GetVitals`, and
  broadcasts on `StreamVitals` only on state transitions.

## CLI reference

| Flag | Env | Default | Description |
|-|-|-|-|
| `--atlas` | `ROBONIX_ATLAS_ENDPOINT` | `127.0.0.1:50051` | Atlas endpoint |
| `--listen` | `ROBONIX_VITALS_LISTEN` | `127.0.0.1:50091` | gRPC listen address |
| `--id` | `ROBONIX_VITALS_PROVIDER_ID` | `vitals` | Provider id |
| `--collect-interval-ms` | `ROBONIX_VITALS_COLLECT_INTERVAL_MS` | `1000` | Poll interval (ms) |
| `--thresholds-path` | `ROBONIX_VITALS_THRESHOLDS_PATH` | `<crate>/thresholds/jetson_agx_orin.yaml` | YAML threshold file |
| `--body-type` | `ROBONIX_VITALS_BODY_TYPE` | — | Body type: `arm`, `dog`, etc. |
| `--body-model` | `ROBONIX_VITALS_BODY_MODEL` | — | Body model: `piper`, `koch`, `go2` |
| `--config` | `ROBONIX_CONFIG_PATH` | — | Optional YAML config file |
| `--log` | `RUST_LOG` | `robonix_vitals=info` | Log filter |

`ROBONIX_VITALS_BODY_PYTHON` env var overrides the Python binary for body
scripts (default: `python3`).

## Threshold file format

```yaml
robot_model: "jetson_agx_orin"
components:
  - name: "cpu"
    warn_above_c: 80.0
    error_above_c: 90.0
    # Battery-style rules (optional):
    # warn_below_percent: 20.0
    # error_below_percent: 5.0
    # warn_below_voltage: 11.0
    # error_below_voltage: 10.0
```

## Source layout

```
system/vitals/
  Cargo.toml
  build.rs
  thresholds/
    jetson_agx_orin.yaml
  scripts/
    piper_body.py        # Piper SDK → JSON bridge (Python)
  src/
    main.rs              # Atlas registration, gRPC serve, collect loop
    config.rs            # VitalsConfig
    service.rs           # VitalsServiceImpl: GetVitals + StreamVitals + change detection
    collect.rs           # SysfsCollector: sysfs/hwmon/thermal
    body.rs              # BodyCollector: Python subprocess → BodyHealth
    normalize.rs         # evaluate() + load_thresholds() + unit tests
    pb.rs                # include!(contract_proto_modules.rs)
```

## Verifying

```bash
cargo build -p robonix-vitals
cargo clippy -p robonix-vitals -- -D warnings
cargo test -p robonix-vitals

grpcurl -plaintext -d '{}' 127.0.0.1:50091 \
  robonix.contracts.RobonixServiceVitalsGet/GetVitals
```
