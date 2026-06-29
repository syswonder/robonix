# Vitals Soma Mock Demo

Vitals' `--mock-soma` starts a full mock Soma gRPC server in-process, streaming
`SomaHealthSnapshot`. Vitals consumes its StreamHealth via loopback gRPC —
**the processing pipeline is identical to production mode**. No real hardware
or real Soma needed for development and debugging.

Optionally, with `--mock-soma-piper-can` (CAN port) or `--mock-soma-koch-port`
(serial port), mock Soma spawns `piper_bridge.py` / `koch_bridge.py` subprocesses
to read real Piper / Koch arm joint data and replace synthetic data.

## Deployment topology

```
┌─────────────────────────────────────────────┐
│ mock_soma.rs (in Vitals process)             │
│   5 scenarios producing deterministic mock   │
│   data                                       │
│                                              │
│                                              │
│   Optional: piper_bridge.py subprocess       │
│   → real Piper CAN data merged into snapshot │
│   Optional: koch_bridge.py subprocess        │
│   → real Koch Dynamixel data merged in       │
│   gRPC server on :50092                      │
└──────────────┬──────────────────────────────┘
               │ StreamHealth (gRPC, loopback)
               v
┌─────────────────────────────────────────────┐
│ Vitals (same process)                       │
│   soma_ingest.rs: normalize + threshold eval │
│   service.rs: GetVitals + StreamVitals       │
│   gRPC server on :50091                      │
└──────────────────────────────────────────────┘
```

## Quick start

Three terminals to run the full health pipeline:

```bash
# Terminal 1: Atlas
target/debug/robonix-atlas \
  --listen 127.0.0.1:50251 \
  --capabilities /path/to/robonix/capabilities

# Terminal 2: mock Soma (scenarios: normal / ramp / fault / toggle / mixed)
target/debug/robonix-vitals \
  --mock-soma \
  --atlas 127.0.0.1:50251 \
  --mock-soma-listen 127.0.0.1:50292 \
  --mock-soma-scenario mixed \
  --mock-soma-interval-ms 10000

# Terminal 3: Vitals consuming the Soma health stream
target/debug/robonix-vitals \
  --atlas 127.0.0.1:50251 \
  --listen 127.0.0.1:50291 \
  --thresholds-path system/vitals/thresholds/example_thresholds.yaml \
  --log info
```

## Optional: real hardware bridges

### Piper arm (CAN bus)

```bash
target/debug/robonix-vitals \
  --mock-soma \
  --atlas 127.0.0.1:50251 \
  --mock-soma-piper-can can0 \
  --mock-soma-piper-python /path/to/roboarm/.venv/bin/python3 \
  --mock-soma-interval-ms 500
```

When `--mock-soma-piper-can` is set:
- Spawns `piper_bridge.py` subprocess, reads real joint data via piper_sdk
- Replaces matching ActuatorState by `joint_1..6` name on `body/arm_right`
- `PiperData.state` → SafetyState.aggregate_state
- Non-zero error_code → appended FaultState
- Without it, fully synthetic Piper data is used

### Koch arm (Dynamixel serial)

```bash
target/debug/robonix-vitals \
  --mock-soma \
  --atlas 127.0.0.1:50251 \
  --mock-soma-koch-port /dev/ttyUSB0 \
  --mock-soma-koch-python /path/to/roboarm/.venv/bin/python3 \
  --mock-soma-interval-ms 500
```

When `--mock-soma-koch-port` is set:
- Spawns `koch_bridge.py` subprocess, reads real joint data via dynamixel_sdk
- Replaces matching ActuatorState by `joint_1..6` name on `body/arm_left`
- `KochData.state` → SafetyState.aggregate_state (NORMAL=0, FAULT=1)
- Non-zero Hardware_Error_Status → appended FaultState
- Without it, fully synthetic Koch data is used

Both bridges can be used simultaneously for dual-arm setups.

## Mock scenarios

| Scenario | Behavior |
|----------|----------|
| `normal` | All joint temps 36-41°C, communication OK, no faults |
| `ramp` | joint_1 temp ramps linearly from 40°C to 88°C every 30s, crossing WARN(60°C)→ERROR(75°C) |
| `fault` | joint_3 injected with overcurrent fault on 4 of every 8 cycles |
| `toggle` | joint_6 torque_enabled toggles on 4 of every 8 cycles |
| `mixed` | ramp + fault + toggle simultaneously |

## Simulated robot

The mock snapshot represents a small robot with a Jetson compute platform, Piper arm (right), Koch arm (left), and battery:

```
body
  computer_jetson
    cpu
    gpu
  arm_right (Piper, CAN)
    joint_1 .. joint_6
  arm_left (Koch, Dynamixel)
    joint_1 .. joint_6
  battery_main
```

Each joint publishes actuator data (position/velocity/effort/motor_temp/driver_temp etc.),
the battery publishes soc/voltage/current/temperature, and Jetson publishes cpu/gpu temps and fan_rpm.

## Expected log output

Using the `mixed` scenario:

```
# Temperature threshold changes
body/arm_right/joint_1/motor_temp health: OK -> WARN
body/arm_right/joint_1/motor_temp health: WARN -> ERROR

# Explicit faults
ALERT: body/arm_right/joint_3/fault/overcurrent - mock joint_3 overcurrent

# Enable state changes
body/arm_right/joint_6 enabled: true -> false

# Body state changes
arm_right/piper body state: NORMAL -> FAULT

# Koch arm body
arm_left/koch body state: NORMAL (first reading)

# Snapshot summary
24.2V | body/arm_right/joint_1/motor_temp:OK(40) ...
```

## Check registrations

```bash
ROBONIX_ATLAS=127.0.0.1:50251 target/debug/rbnx caps -v
```

Expected output:

```
mock-soma [ACTIVE] robonix/system/soma
vitals [ACTIVE] robonix/service/vitals
```

## gRPC verification

```bash
# Vitals side
grpcurl -plaintext -d '{}' 127.0.0.1:50091 \
  robonix.contracts.RobonixServiceVitalsGet/GetVitals

# Mock Soma side
grpcurl -plaintext -d '{}' 127.0.0.1:50092 \
  robonix.contracts.RobonixSystemSomaGetHealth/GetHealth
```

## Threshold file

Uses `thresholds/example_thresholds.yaml` (Soma selector format):

```yaml
rules:
  - id: "joint_motor_temp"
    selector: { kind: "JOINT", signal: "motor_temp" }
    warn_above: 60.0
    error_above: 75.0
    unit: "degC"
  - id: "battery_soc"
    selector: { kind: "BATTERY", signal: "soc_percent" }
    warn_below: 20.0
    error_below: 8.0
    unit: "percent"
  # ...
```

`communication_ok = false`, `vendor_error_code != 0`, and active `fault_id` are
not evaluated through YAML thresholds — they are converted to WARN/ERROR directly
in `soma_ingest.rs`.

## CLI flags

| Flag | Env var | Default |
|------|---------|---------|
| `--mock-soma` | `ROBONIX_VITALS_MOCK_SOMA` | `false` |
| `--mock-soma-listen` | `ROBONIX_VITALS_MOCK_SOMA_LISTEN` | `127.0.0.1:50092` |
| `--mock-soma-scenario` | `ROBONIX_VITALS_MOCK_SOMA_SCENARIO` | `normal` |
| `--mock-soma-interval-ms` | `ROBONIX_VITALS_MOCK_SOMA_INTERVAL_MS` | `10000` |
| `--mock-soma-piper-can` | `ROBONIX_VITALS_MOCK_SOMA_PIPER_CAN` | — |
| `--mock-soma-piper-python` | `ROBONIX_VITALS_MOCK_SOMA_PIPER_PYTHON` | `python3` |
| `--mock-soma-piper-script` | `ROBONIX_VITALS_MOCK_SOMA_PIPER_SCRIPT` | `<crate>/scripts/piper_bridge.py` |
| `--mock-soma-koch-port` | `ROBONIX_VITALS_MOCK_SOMA_KOCH_PORT` | — |
| `--mock-soma-koch-python` | `ROBONIX_VITALS_MOCK_SOMA_KOCH_PYTHON` | `python3` |
| `--mock-soma-koch-script` | `ROBONIX_VITALS_MOCK_SOMA_KOCH_SCRIPT` | `<crate>/scripts/koch_bridge.py` |

## Implementation files

```
system/vitals/src/mock_soma.rs     # MockSomaService + PiperBridge + KochBridge + generate_snapshot
system/vitals/src/subprocess.rs    # SubprocessHandle (bridge subprocess management)
system/vitals/scripts/piper_bridge.py    # PiperCollector + stdin/stdout JSON wrapper
system/vitals/scripts/koch_bridge.py     # KochCollector + stdin/stdout JSON wrapper
```
