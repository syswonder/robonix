# robonix-agent

robonix-agent is the system agent: a ReAct-style loop that talks to an OpenAI-compatible VLM, discovers capabilities from the control plane, and calls tools derived from registered SKILL.md content.

## Environment

| Variable | Purpose |
|----------|---------|
| `ROBONIX_SERVER` | gRPC server `host:port` or `http://…` (default `localhost:50051`) |

## How it works

1. Connects to robonix-server via robonix-sdk.
2. Queries skills from the registry and builds tool definitions for the chat API.
3. Runs an interactive ReAct loop: the model proposes tool calls, the agent executes them against the runtime; results return until the user types `quit`.

```bash
cargo run -p robonix-agent
```
