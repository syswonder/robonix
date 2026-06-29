# robonix-vitals

Monitors robot onboard health (CPU/GPU/NVMe temperature, voltage) and body (joint motors), evaluates thresholds, and reports via gRPC. Supports Piper (CAN bus) and Koch (Dynamixel serial) arms via hardware bridge subprocesses.

Vitals consumes the unified health stream from Soma (real or mock). Without Soma, Vitals exits with an error.

## Running

```bash
# Auto-discover Soma via Atlas
robonix-vitals --log info

# Specify Soma endpoint directly
robonix-vitals --soma-endpoint 127.0.0.1:50092 --log info
```

## Mock testing (no hardware needed)

Start mock Soma + Vitals across three terminals to run the full health pipeline:

```bash
# Terminal 1: Atlas
cargo run --release -p robonix-atlas -- --log info

# Terminal 2: mock Soma (scenarios: normal / ramp / fault / toggle / mixed)
cargo run --release -p robonix-vitals -- --log info \
  --mock-soma \
  --mock-soma-scenario ramp \
  --mock-soma-interval-ms 1000

# Terminal 3: Vitals consuming the Soma health stream
cargo run --release -p robonix-vitals -- --log info
```

### Mock Soma with real Piper hardware

When `--mock-soma-piper-can` is set, mock Soma spawns a `piper_bridge.py` subprocess to read real Piper joint data and merge it into synthetic snapshots:

```bash
target/debug/robonix-vitals --log info \
  --mock-soma \
  --mock-soma-piper-can can0 \
  --mock-soma-piper-python /path/to/roboarm/.venv/bin/python3 \
  --mock-soma-interval-ms 1000
```

Without `--mock-soma-piper-can`, fully synthetic Piper data is used.

### Mock Soma with real Koch hardware

When `--mock-soma-koch-port` is set, mock Soma spawns a `koch_bridge.py` subprocess to read real Koch (Dynamixel) joint data via the serial port:

```bash
target/debug/robonix-vitals --log info \
  --mock-soma \
  --mock-soma-koch-port /dev/ttyUSB0 \
  --mock-soma-koch-python /path/to/roboarm/.venv/bin/python3 \
  --mock-soma-interval-ms 1000
```

Piper and Koch bridges can be used together for dual-arm setups:

```bash
target/debug/robonix-vitals --log info \
  --mock-soma \
  --mock-soma-piper-can can0 \
  --mock-soma-koch-port /dev/ttyUSB0 \
  --mock-soma-interval-ms 1000
```

## CLI flags

| Flag | Env var | Default |
|------|---------|---------|
| `--atlas` | `ROBONIX_ATLAS_ENDPOINT` | `127.0.0.1:50051` |
| `--listen` | `ROBONIX_VITALS_LISTEN` | `127.0.0.1:50091` |
| `--id` | `ROBONIX_VITALS_PROVIDER_ID` | `vitals` |
| `--thresholds-path` | `ROBONIX_VITALS_THRESHOLDS_PATH` | `thresholds/example_thresholds.yaml` |
| `--soma-endpoint` | `ROBONIX_SOMA_ENDPOINT` | — |
| `--mock-soma` | `ROBONIX_VITALS_MOCK_SOMA` | `false` |
| `--mock-soma-listen` | `ROBONIX_VITALS_MOCK_SOMA_LISTEN` | `127.0.0.1:50092` |
| `--mock-soma-scenario` | `ROBONIX_VITALS_MOCK_SOMA_SCENARIO` | `normal` |
| `--mock-soma-interval-ms` | `ROBONIX_VITALS_MOCK_SOMA_INTERVAL_MS` | `10000` |
| `--mock-soma-piper-can` | `ROBONIX_VITALS_MOCK_SOMA_PIPER_CAN` | — (empty = synthetic data) |
| `--mock-soma-piper-python` | `ROBONIX_VITALS_MOCK_SOMA_PIPER_PYTHON` | `python3` |
| `--mock-soma-piper-script` | `ROBONIX_VITALS_MOCK_SOMA_PIPER_SCRIPT` | `<crate>/scripts/piper_bridge.py` |
| `--mock-soma-koch-port` | `ROBONIX_VITALS_MOCK_SOMA_KOCH_PORT` | — (empty = synthetic data) |
| `--mock-soma-koch-python` | `ROBONIX_VITALS_MOCK_SOMA_KOCH_PYTHON` | `python3` |
| `--mock-soma-koch-script` | `ROBONIX_VITALS_MOCK_SOMA_KOCH_SCRIPT` | `<crate>/scripts/koch_bridge.py` |
| `--config` | `ROBONIX_CONFIG_PATH` | — |
| `--log` | `RUST_LOG` | `robonix_vitals=info` |

## Threshold format

Soma selector format:

```yaml
rules:
  - id: "joint_motor_temp"
    selector:
      kind: "JOINT"
      signal: "motor_temp"
    warn_above: 60.0
    error_above: 75.0
    unit: "degC"
```

## Testing

### Unit tests

```bash
cargo test -p robonix-vitals
```

### gRPC verification

```bash
grpcurl -plaintext -d '{}' 127.0.0.1:50091 \
  robonix.contracts.RobonixServiceVitalsGet/GetVitals
```
