# Vitals Module Health Demo

This document demonstrates the minimal system-module health loop in Vitals:

```text
executor / pilot self-report ModuleHealthReport
        -> Vitals polls get_health periodically
        -> Vitals aggregates ModuleHealthSnapshot
        -> Vitals synthesizes STALE / ERROR after ttl_ms expires
        -> Vitals records ERROR -> OK after the module reports again
```

No real robot hardware is required. The hardware-health path uses mock Soma,
while the module-health path uses the executor and pilot `get_health` services.

## 1. Prepare

Enter the repository and build the required binaries:

```bash
cd /path/to/robonix

cargo build -p robonix-atlas
cargo build -p robonix-executor
cargo build -p robonix-pilot
cargo build -p robonix-vitals
```

Install `grpcurl` and `jq` if you want to query the gRPC snapshot directly.
Without `grpcurl`, the same flow can still be verified from Vitals logs.

## 2. Start The Stack

Use five terminals.

### Terminal 1: Atlas

```bash
cd /path/to/robonix

cargo run -p robonix-atlas -- \
  --listen 127.0.0.1:50051 \
  --capabilities capabilities
```

Atlas is the service registry and capability discovery service.

### Terminal 2: Executor

```bash
cd /path/to/robonix

cargo run -p robonix-executor -- \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50061
```

Executor declares:

```text
robonix/system/executor/get_health
```

Vitals polls this service.

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

The VLM fields may use dummy values for this demo because no task is submitted
to Pilot. Only `get_health` is exercised.

Pilot declares:

```text
robonix/system/pilot/get_health
```

### Terminal 4: Mock Soma

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

Mock Soma declares:

```text
robonix/system/soma/health
robonix/system/soma/get_health
```

The `--mock-soma-interval-ms 10000` setting emits one hardware-health frame
every 10 seconds, which keeps screen recordings readable.

### Terminal 5: Vitals

```bash
cd /path/to/robonix

cargo run -p robonix-vitals -- \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50091 \
  --thresholds-path system/vitals/thresholds/soma_mock.yaml \
  --log robonix_vitals=info
```

Vitals declares:

```text
robonix/system/vitals/get
robonix/system/vitals/stream
robonix/system/vitals/modules/get
```

## 3. Observe Normal Startup

Vitals should print lines like:

```text
declared GetVitals at 127.0.0.1:50091
declared StreamVitals at 127.0.0.1:50091
declared ModuleHealthSnapshot at 127.0.0.1:50091
connected to Soma provider 'mock-soma' through Atlas
connected to Soma StreamHealth
module health poll connected: executor (robonix/system/executor/get_health)
module health poll connected: pilot (robonix/system/pilot/get_health)
```

This confirms that both paths are alive:

```text
mock Soma -> Vitals                       hardware-health path
executor / pilot -> Vitals modules/get   module-health path
```

After about 10 seconds, Vitals also prints the mock Soma body baseline:

```text
[vitals] body: NORMAL (computer_jetson/jetson_agx_orin)
[vitals] body: NORMAL (arm/mock_arm)
[vitals] body: NORMAL (battery_main/mock_bms)
```

Those body lines are hardware-health data, not module-health data.

## 4. Query ModuleHealthSnapshot

If `grpcurl` is installed, query the Vitals aggregate module-health snapshot:

```bash
cd /path/to/robonix

PROTO_DIR=$(ls -td target/debug/build/robonix-vitals-*/out | head -n1)

grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50091 \
  robonix.contracts.RobonixSystemVitalsModulesGet/GetModuleHealthSnapshot | jq
```

The normal response should include executor and pilot:

```json
{
  "snapshot": {
    "schemaVersion": 1,
    "modules": [
      {
        "moduleKey": "executor",
        "moduleId": "executor",
        "providerId": "executor",
        "health": 0,
        "state": "active",
        "reasonCode": "OK",
        "detail": "executor serving",
        "source": "SELF_REPORTED",
        "ttlMs": 5000
      },
      {
        "moduleKey": "pilot",
        "moduleId": "pilot",
        "providerId": "pilot",
        "health": 0,
        "state": "active",
        "reasonCode": "OK",
        "detail": "pilot serving",
        "source": "SELF_REPORTED",
        "ttlMs": 5000
      }
    ]
  }
}
```

Field summary:

| Field | Meaning |
|---|---|
| `health = 0` | OK |
| `health = 1` | WARN |
| `health = 2` | ERROR |
| `source = SELF_REPORTED` | Reported by the module's own `get_health` service |
| `ttlMs = 5000` | The report is valid for 5 seconds |

## 5. Test Executor Stale

Keep Atlas, mock Soma, Vitals, and Pilot running.

Stop executor in Terminal 2:

```bash
Ctrl+C
```

Vitals first notices that polling failed:

```text
[vitals] module health poll lost executor: ...
```

If executor had previously reported successfully, Vitals synthesizes stale after
`ttlMs = 5000` expires:

```text
[vitals] module executor health: OK -> ERROR (STALE)
[vitals] ALERT: module executor - no health report received within ttl
```

Query `modules/get` again. Executor should now look like:

```json
{
  "moduleKey": "executor",
  "moduleId": "executor",
  "providerId": "executor",
  "health": 2,
  "state": "stale",
  "reasonCode": "STALE",
  "detail": "no health report received within ttl",
  "source": "VITALS_SYNTHESIZED_STALE",
  "ttlMs": 5000
}
```

Vitals only synthesizes stale for modules that have reported at least once.
A module that never appeared is not immediately marked as ERROR.

## 6. Test Executor Recovery

Restart Terminal 2:

```bash
cd /path/to/robonix

cargo run -p robonix-executor -- \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50061
```

Vitals should print:

```text
[vitals] module health poll connected: executor (robonix/system/executor/get_health)
[vitals] module executor health: ERROR -> OK (OK)
```

Query `modules/get` again. Executor should recover to:

```json
{
  "moduleKey": "executor",
  "health": 0,
  "state": "active",
  "reasonCode": "OK",
  "source": "SELF_REPORTED"
}
```

## 7. Test Pilot Stale

Pilot uses the same flow:

1. Stop pilot in Terminal 3.
2. Wait about 5 seconds.
3. Vitals should print `pilot OK -> ERROR (STALE)`.
4. Restart pilot.
5. Vitals should print `pilot ERROR -> OK (OK)`.

## 8. Troubleshooting

### Vitals Does Not Print Executor / Pilot Connected

Check that executor and pilot are running and using the same Atlas endpoint:

```text
--atlas 127.0.0.1:50051
```

### Pilot Fails Because VLM Config Is Missing

Pilot currently requires VLM fields at startup even when only `get_health` is
tested. Use the dummy values shown in this demo.

### Port Already In Use

This demo uses:

| Service | Port |
|---|---|
| Atlas | `50051` |
| Executor | `50061` |
| Pilot | `50071` |
| Vitals | `50091` |
| Mock Soma | `50092` |

Stop the old process or replace all affected ports consistently.

### grpcurl Query Fails

First confirm that Vitals has been built and that the generated proto exists:

```bash
PROTO_DIR=$(ls -td target/debug/build/robonix-vitals-*/out | head -n1)
ls "$PROTO_DIR/robonix_contracts.proto"
```

If `grpcurl` is unavailable, verify the flow from Vitals logs instead.

## 9. What This Demo Proves

After completing the steps above, the demo proves that:

1. executor and pilot expose `ModuleHealthReport` through the V1 protocol.
2. Vitals discovers and polls their `get_health` services.
3. Vitals exposes the aggregate `ModuleHealthSnapshot` through `modules/get`.
4. Vitals synthesizes `STALE / ERROR` when a known module exceeds `ttl_ms`.
5. Vitals records and exposes recovery when the module returns to `OK`.
