# robonix-pilot

robonix-pilot is the VLM-driven reasoning service: a ReAct-style loop that receives `Intent` messages from Liaison, fetches tool definitions from Executor, calls an OpenAI-compatible VLM, dispatches `TaskGraph` slices (v1: linear tool calls; BT/RTDL TODO) to Executor, and streams `PilotEvent` results back.

## Environment

| Variable | Purpose |
|----------|---------|
| `ROBONIX_ATLAS` | Atlas gRPC `host:port` (default `localhost:50051`) |
| `ROBONIX_EXECUTOR_ENDPOINT` | Executor gRPC endpoint (default `http://localhost:50061`) |
| `ROBONIX_PILOT_SOUL` | Path to `SOUL.md` personality file (default `~/.robonix/SOUL.md`) |
| `ROBONIX_PILOT_MAX_TOOL_ROUNDS` | Max VLM→Executor rounds per turn (default 64) |

## How it works

1. Connects to robonix-atlas via robonix-sdk; registers as `com.robonix.runtime.pilot`.
2. Connects to robonix-executor for `ListTools` and `Execute` RPCs.
3. On each `HandleIntent` RPC: fetches tool list, builds system prompt (SOUL.md + each registered package's CAPABILITY.md), then runs the ReAct loop until the VLM produces no tool calls.

```bash
cargo run -p robonix-pilot
```
