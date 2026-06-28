# robonix-health-primitive — raw hardware sensor data

Wraps robot sysfs / hwmon sensors behind unified gRPC contracts so that
downstream consumers (vitals, sentinel, …) never touch hardware-specific
paths or protocols.

## Contracts

| contract | mode | transport | port | consumers |
|-|-|-|-|-|
| `robonix/primitive/health/state` | rpc | gRPC | 50092 | vitals, sentinel |
| `robonix/primitive/health/stream` | server\_stream | gRPC | 50092 | sentinel |

## Quick start

```bash
robonix-health-primitive --log info
# Or
cargo run --release -p robonix-health-primitive -- --log info
```

## Data sources

| sensor | sysfs path | unit | normalised to |
|-|-|-|-|
| thermal zones | `/sys/class/thermal/<zone>/temp` | milli-°C | °C |
| NVMe temp | `/sys/class/hwmon/<nvme>/temp1_input` | milli-°C | °C |
| system voltage | `/sys/class/hwmon/<ina3221>/in1_input` | mV | V |

Thermal zone names are stripped of the `-thermal` suffix (e.g. `cpu-thermal` → `cpu`)
so they match vitals threshold rule names.

## CLI reference

| Flag / Option | Env | Default | Description |
|-|-|-|-|
| `--atlas` | `ROBONIX_ATLAS_ENDPOINT` | `127.0.0.1:50051` | Atlas endpoint |
| `--listen` | `ROBONIX_HEALTH_LISTEN` | `127.0.0.1:50092` | gRPC listen address |
| `--id` | `ROBONIX_HEALTH_PROVIDER_ID` | `health_primitive` | Provider id |
| `--collect-interval-ms` | `ROBONIX_HEALTH_COLLECT_INTERVAL_MS` | `1000` | Poll interval |
| `--log` | `RUST_LOG` | `robonix_health_primitive=info` | Log filter |

## Source layout

```
system/health_primitive/
  Cargo.toml
  build.rs
  src/
    main.rs       # Atlas registration, gRPC serve, collect loop
    config.rs     # HealthConfig
    service.rs    # Collector + HealthPrimitiveService (gRPC handlers)
    pb.rs         # include!(contract_proto_modules.rs)
```

Contracts and IDL:

```
capabilities/lib/health/msg/SensorReading.msg
capabilities/lib/health/msg/HealthState.msg
capabilities/lib/health/srv/GetHealthState.srv
capabilities/lib/health/srv/StreamHealthState.srv
capabilities/primitive/health/state.v1.toml
capabilities/primitive/health/stream.v1.toml
```
