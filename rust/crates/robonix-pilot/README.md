# robonix-pilot

`robonix-pilot` is the VLM-driven reasoning and planning service for Robonix. It receives `SystemPilot` tasks, builds the model prompt from the current Atlas capability catalog, asks the VLM for an RTDL plan, expands that RTDL into a `Plan`, and sends the plan to `robonix-executor`.

## Build

From `rust/`:

```sh
cargo build -p robonix-pilot
```

The crate is also built by the workspace `make build` and installed by `make install`.

## Run

Manual launch:

```sh
robonix-pilot \
  --vlm-upstream https://api.openai.com/v1 \
  --vlm-api-key sk-... \
  --vlm-model gpt-5.5
```

Common configuration:

- `--atlas` / `ROBONIX_ATLAS_ENDPOINT`: Atlas endpoint. Defaults to `127.0.0.1:50051`.
- `--listen` / `ROBONIX_PILOT_LISTEN`: Pilot gRPC listen address. Defaults to `127.0.0.1:50071`.
- `--capability-id` / `ROBONIX_PILOT_CAPABILITY_ID`: registered Pilot capability id.
- `--vlm-upstream` / `ROBONIX_VLM_UPSTREAM`: OpenAI-compatible API base URL.
- `--vlm-api-key` / `ROBONIX_VLM_API_KEY`: VLM API key.
- `--vlm-model` / `ROBONIX_VLM_MODEL`: VLM model name.
- `--vlm-format` / `ROBONIX_VLM_FORMAT`: API dialect. Only `openai` is currently supported.
- `--config` / `ROBONIX_CONFIG_PATH`: optional YAML config file.
- `--log`: env_logger filter. Falls back to `RUST_LOG`, then `robonix_pilot=info`.
- `ROBONIX_PILOT_SOUL`: optional path to a SOUL markdown file. If unset, Pilot tries `~/.robonix/SOUL.md`.
- `ROBONIX_PILOT_MAX_TOOL_ROUNDS`: maximum RTDL execution rounds per turn. Defaults to `64`.

## RTDL Planning Flow

Pilot no longer sends OpenAI `tools` / function schemas as the primary planning path. Instead, it writes the RTDL grammar and available capability list into the prompt. The model must return a single JSON object:

```json
{
  "content": "I will inspect the current scene.",
  "rtdl": {
    "op": "sequence",
    "children": [
      {
        "op": "do",
        "cap": "camera_snapshot",
        "args": {}
      }
    ]
  }
}
```

MVP RTDL supports only:

- `sequence`: ordered children.
- `do`: one capability call, where `cap` is the unique `capability_name` shown in the prompt and `args` is a JSON object.

Pilot buffers the full assistant JSON before showing user-visible text, validates the RTDL, expands it into the existing `Plan { calls }` shape, sends that `Plan` to Liaison, and dispatches it to Executor. Executor still receives `Execute(Plan)` and does not parse RTDL.

An empty sequence means the model is done for the turn:

```json
{ "content": "Done.", "rtdl": { "op": "sequence", "children": [] } }
```
