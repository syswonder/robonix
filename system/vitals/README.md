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

### Mock Soma with real hardware

Use `--mock-soma-arm` to select the hardware backend (default: `synthetic`):

```bash
# Piper arm via CAN bus
robonix-vitals --log info \
  --mock-soma \
  --mock-soma-arm piper \
  --mock-soma-piper-can can0 \
  --mock-soma-bridge-python /path/to/roboarm/.venv/bin/python3 \
  --mock-soma-interval-ms 1000

# Koch arm via Dynamixel serial
robonix-vitals --log info \
  --mock-soma \
  --mock-soma-arm koch \
  --mock-soma-koch-port /dev/ttyUSB0 \
  --mock-soma-bridge-python /path/to/roboarm/.venv/bin/python3 \
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
| `--mock-soma-arm` | `ROBONIX_VITALS_MOCK_SOMA_ARM` | `synthetic` |
| `--mock-soma-piper-can` | `ROBONIX_VITALS_MOCK_SOMA_PIPER_CAN` | `can0` (when arm=piper) |
| `--mock-soma-koch-port` | `ROBONIX_VITALS_MOCK_SOMA_KOCH_PORT` | `/dev/ttyUSB0` (when arm=koch) |
| `--mock-soma-bridge-python` | `ROBONIX_VITALS_MOCK_SOMA_BRIDGE_PYTHON` | `python3` |
| `--mock-soma-piper-script` | `ROBONIX_VITALS_MOCK_SOMA_PIPER_SCRIPT` | `<crate>/scripts/piper_bridge.py` |
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
