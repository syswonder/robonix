# Vitals Soma Mock Demo

This demo uses the Vitals binary in two roles:

- `mock Soma`: simulates the future Soma health provider.
- `Vitals`: discovers Soma through Atlas, subscribes to Soma health snapshots, and converts them into Vitals health output.

It is useful when no real robot hardware or real Soma implementation is available.

## What This Demo Verifies

The demo verifies this path:

```text
mock_soma.rs
  generates SomaHealthSnapshot once per second
        |
        v
Atlas
  advertises robonix/system/soma/health
        |
        v
Vitals
  opens Soma health stream
        |
        v
soma_ingest.rs
  converts SomaHealthSnapshot into VitalsSnapshot
        |
        v
service.rs
  logs health changes, alerts, body state, and snapshot summaries
```

## Build

Run from the repository root:

```bash
cd /path/to/robonix

cargo build -p robonix-atlas -p robonix-vitals -p robonix-cli
```

## Run

Open three terminals.

### Terminal 1: Atlas

```bash
cd /path/to/robonix

target/debug/robonix-atlas \
  --listen 127.0.0.1:50251 \
  --capabilities /path/to/robonix/capabilities \
  --log robonix_atlas=info
```

Atlas is the service registry. Both mock Soma and Vitals register themselves here.

### Terminal 2: Mock Soma

```bash
cd /path/to/robonix

target/debug/robonix-vitals \
  --mock-soma \
  --atlas 127.0.0.1:50251 \
  --mock-soma-listen 127.0.0.1:50292 \
  --mock-soma-scenario mixed \
  --mock-soma-interval-ms 10000 \
  --log robonix_vitals=info
```

Expected startup output:

```text
mock Soma ready on 127.0.0.1:50292 scenario=mixed interval_ms=10000
```

This process registers:

```text
robonix/system/soma/get_health
robonix/system/soma/health
```

### Terminal 3: Vitals

```bash
cd /path/to/robonix

target/debug/robonix-vitals \
  --atlas 127.0.0.1:50251 \
  --listen 127.0.0.1:50291 \
  --thresholds-path system/vitals/thresholds/soma_mock.yaml \
  --log robonix_vitals=info
```

Expected startup output:

```text
robonix-vitals ready on 127.0.0.1:50291 (Soma input)
```

The third terminal should keep printing logs when health states change. That is expected.

## Simulated Robot

The mock snapshot represents a small robot body with a Jetson computer, one Piper arm, and one main battery:

```text
body
  computer_jetson
    cpu
    gpu
  arm_right
    joint_1
    joint_2
    joint_3
    joint_4
    joint_5
    joint_6
  battery_main
```

Each joint publishes actuator data:

```text
position
velocity
effort
current
voltage
motor_temp
driver_temp
torque_enabled
communication_ok
vendor_error_code
status_flags
```

The battery publishes:

```text
soc_percent = 76
soh_percent = 96
voltage = 24.2V
current = -3.1A
temperature = 32C
remaining_s = 7200
cycle_count = 142
```

The Jetson publishes:

```text
cpu temperature = 43C
gpu temperature = 45C
fan_rpm = 1800
```

## Mock Scenarios

Use `--mock-soma-scenario <scenario>` to select a scenario.
Use `--mock-soma-interval-ms <ms>` to control how often the stream publishes a new snapshot. The default is `10000`, which is easier to follow in demos.

```text
normal   all data stays mostly healthy
ramp     joint_1 temperature rises and loops
fault    joint_3 periodically reports communication and overcurrent faults
toggle   joint_6 periodically becomes disabled
mixed    enables ramp + fault + toggle
```

For demos, `mixed` is usually the most useful because it exercises temperature thresholds, explicit faults, and enable-state changes together.

## Expected Observations

With `mixed`, Vitals should report several kinds of events.

Temperature threshold changes:

```text
body/arm_right/joint_1/motor_temp health: OK -> WARN
body/arm_right/joint_1/motor_temp health: WARN -> ERROR
body/arm_right/joint_1/driver_temp health: WARN -> ERROR
```

Explicit actuator faults:

```text
ALERT: body/arm_right/joint_3/communication - body/arm_right/joint_3 communication is not OK
ALERT: body/arm_right/joint_3/vendor_error - body/arm_right/joint_3 vendor_error_code=0x4
```

Fault messages:

```text
ALERT: body/arm_right/joint_3/fault/overcurrent - mock joint_3 overcurrent
ALERT: body/arm_right/joint_1/fault/motor_overheat - mock joint_1 temperature is high
```

Joint enable changes:

```text
body/arm_right/joint_6 enabled: true -> false
ALERT: body/arm_right/joint_6 - disabled
```

Body state changes:

```text
arm_right/piper body state: NORMAL -> FAULT
ALERT: body arm_right/piper state=FAULT (active faults: overcurrent)
```

Snapshot summaries:

```text
24.2V | body/arm_right/joint_1/motor_temp:OK(40) body/arm_right/joint_1/driver_temp:OK(43) ...
```

The summary line is generated from `VitalsSnapshot.components`, not directly from raw Soma data. It shows the health result after Vitals has applied thresholds and explicit fault rules.

## Thresholds

The demo threshold file is:

```text
system/vitals/thresholds/soma_mock.yaml
```

It currently checks:

```text
joint motor temperature: WARN >= 60C, ERROR >= 75C
joint driver temperature: WARN >= 70C, ERROR >= 85C
battery state of charge: WARN <= 20%, ERROR <= 8%
battery voltage: WARN <= 22V, ERROR <= 19V
board temperature: WARN >= 80C, ERROR >= 90C
```

Fields such as `communication_ok = false`, `vendor_error_code != 0`, and active `fault_id` do not go through YAML thresholds. They are explicit hardware states and are converted into WARN or ERROR directly.

## Inspect Registration

In a fourth terminal:

```bash
cd /path/to/robonix

ROBONIX_ATLAS=127.0.0.1:50251 target/debug/rbnx caps -v
```

Expected providers:

```text
mock-soma [ACTIVE] robonix/system/soma
vitals [ACTIVE] robonix/service/vitals
```

Inspect Soma contracts:

```bash
ROBONIX_ATLAS=127.0.0.1:50251 \
target/debug/rbnx contracts -p robonix/system/soma -v
```

Expected Soma capabilities:

```text
robonix/system/soma/get_health
robonix/system/soma/health
```

## Optional RPC Check

If `grpcurl` is available:

```bash
grpcurl -plaintext -d '{}' 127.0.0.1:50291 \
  robonix.contracts.RobonixServiceVitalsGet/GetVitals
```

The response should contain:

```text
power
components
bodies
```

For the mock demo, `bodies` should include root-child groups such as:

```text
computer_jetson / jetson_agx_orin
arm_right / piper
battery_main / mock_bms
```

Each group contains its child component-tree entries. For example, `computer_jetson` contains CPU and GPU, while `arm_right` contains the six joints.

## Stop

Press `Ctrl-C` in the three running terminals:

```text
Vitals
mock Soma
Atlas
```
