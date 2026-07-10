# Vitals Health Demo

This runbook shows how to demo Vitals collecting health from two paths:

```text
hardware health:
  Soma health source -> SomaHealthSnapshot -> VitalsSnapshot

software health:
  vitals / executor / pilot -> ModuleHealthReport -> ModuleHealthSnapshot
```

The demo does not depend on whether the Soma source is mock or real. The
commands below use mock Soma by default because it is stable for recording and
does not require hardware. If a real Soma deployment is available, replace only
Terminal 4; the Vitals side stays the same.

## Ports

Use one Atlas endpoint for every process:

| Process | Address |
|---|---|
| Atlas | `127.0.0.1:50051` |
| Executor | `127.0.0.1:50061` |
| Pilot | `127.0.0.1:50071` |
| Soma health source | `127.0.0.1:50092` |
| Vitals | `127.0.0.1:50091` |

## Build

```bash
cd /path/to/robonix

cargo build -p robonix-atlas
cargo build -p robonix-executor
cargo build -p robonix-pilot
cargo build -p robonix-vitals
```

If you plan to use real Soma instead of mock Soma:

```bash
cargo build -p robonix-soma
```

## Start The Demo

Use five terminals.

### Terminal 1: Atlas

```bash
cd /path/to/robonix

cargo run -p robonix-atlas -- \
  --listen 127.0.0.1:50051 \
  --capabilities capabilities
```

### Terminal 2: Executor

```bash
cd /path/to/robonix

cargo run -p robonix-executor -- \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50061
```

### Terminal 3: Pilot

```bash
cd /path/to/robonix

cargo run -p robonix-pilot -- \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50071 \
  --vlm-upstream http://127.0.0.1:9/v1 \
  --vlm-api-key dummy \
  --vlm-model dummy
```

The dummy VLM settings are enough for this demo because no Pilot task is
submitted. Only Pilot's `get_health` endpoint is exercised.

### Terminal 4: Soma Health Source

Default, hardware-free demo source:

```bash
cd /path/to/robonix

cargo run -p robonix-vitals -- \
  --atlas 127.0.0.1:50051 \
  --mock-soma \
  --mock-soma-listen 127.0.0.1:50092 \
  --mock-soma-scenario normal \
  --mock-soma-interval-ms 10000 \
  --log info
```

This process declares the same Soma health contracts that Vitals consumes:

```text
robonix/system/soma/get_health
robonix/system/soma/health
```

If you want to use real Soma instead, run this in Terminal 4:

```bash
cd /path/to/robonix

cargo run -p robonix-soma -- \
  --atlas 127.0.0.1:50051 \
  --robot-yaml /path/to/deploy/soma.yaml \
  --listen 127.0.0.1:50092
```

For real hardware data, the deployment directory must contain a valid
`soma.yaml`, a matching `robonix_manifest.yaml`, and a health primitive that
exposes `robonix/primitive/health/stream`.

### Terminal 5: Vitals

```bash
cd /path/to/robonix

cargo run -p robonix-vitals -- \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50091 \
  --thresholds-path system/vitals/thresholds/example_thresholds.yaml \
  --log robonix_vitals=info
```

Vitals declares:

```text
robonix/system/vitals/get
robonix/system/vitals/stream
robonix/system/vitals/modules/get
```

## What To Show

In the Vitals terminal, show these lines:

```text
[vitals] connected to Soma provider ...
[vitals] connected to Soma StreamHealth
[vitals] module health poll connected: executor (robonix/system/executor/get_health)
[vitals] module health poll connected: pilot (robonix/system/pilot/get_health)
```

These lines prove that both collection paths are alive:

```text
Soma health source -> Vitals hardware health
executor / pilot -> Vitals software module health
```

## Query Results

Use a sixth terminal.

Prepare the generated protobuf import path:

```bash
cd /path/to/robonix

PROTO_DIR=$(ls -td target/debug/build/robonix-vitals-*/out | head -n1)
```

### 1. Query Soma Source Health

```bash
grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50092 \
  robonix.contracts.RobonixSystemSomaGetHealth/GetHealth | jq
```

Expected evidence:

- `snapshot.schemaVersion` is `1`.
- `snapshot.components` contains the hardware component tree.
- Mock Soma returns deterministic demo values.
- Real Soma returns hardware data from the configured health primitive.

### 2. Query Vitals Hardware Snapshot

```bash
grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50091 \
  robonix.contracts.RobonixSystemVitalsGet/GetVitals | jq
```

Expected evidence:

- `snapshot.bodies` contains normalized hardware health from Soma.
- Joint entries include stable component ids such as `body/arm/joint_1`.
- Temperature, enable state, and error-code fields are present.

### 3. Query Vitals Software Module Health

```bash
grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50091 \
  robonix.contracts.RobonixSystemVitalsModulesGet/GetModuleHealthSnapshot | jq
```

Expected evidence:

```text
vitals   OK active SELF_REPORTED
executor OK active SELF_REPORTED
pilot    OK active SELF_REPORTED
```

This proves that Vitals is collecting software module health in the same demo.

## Optional Fault Demo

Stop Executor in Terminal 2. After the configured TTL, Vitals should log:

```text
[vitals] module health poll lost executor: ...
[vitals] module executor health: OK -> ERROR (STALE)
[vitals] ALERT: module executor - no health report received within ttl
```

Query module health again:

```bash
grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50091 \
  robonix.contracts.RobonixSystemVitalsModulesGet/GetModuleHealthSnapshot | jq
```

Executor should now have:

```text
health: ERROR
state: stale
reason_code: STALE
source: VITALS_SYNTHESIZED_STALE
```

Restart Executor, and Vitals should log:

```text
[vitals] module health poll connected: executor (robonix/system/executor/get_health)
[vitals] module executor health: ERROR -> OK (OK)
```

## Common Pitfalls

- All processes must use the same Atlas endpoint.
- Do not mix `50051` and `50251` unless every process is changed together.
- The Soma source can be mock or real; Vitals consumes the same contract either
  way.
- Real hardware data requires a health primitive in the deployment manifest.
- Vitals can start before Soma is available and retries the Soma stream in the
  background.
