# robonix-pilot

`robonix-pilot` is the VLM-driven reasoning and planning service for Robonix. It receives `SystemPilot` tasks, builds the model prompt from the current Atlas capability catalog, asks the VLM for an RTDL plan, expands that RTDL into a `Plan`, and sends the plan to `robonix-executor`.

## Build

From the repo root:

```sh
cargo build -p robonix-pilot
```

The crate is part of the top-level Cargo workspace and is built / installed by
`make build` and `make install` respectively.

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
- `--id` / `ROBONIX_PILOT_PROVIDER_ID`: Pilot provider id registered with Atlas. Defaults to `pilot`.
- `--vlm-upstream` / `ROBONIX_VLM_UPSTREAM`: OpenAI-compatible API base URL.
- `--vlm-api-key` / `ROBONIX_VLM_API_KEY`: VLM API key.
- `--vlm-model` / `ROBONIX_VLM_MODEL`: VLM model name.
- `--vlm-format` / `ROBONIX_VLM_FORMAT`: API dialect. Only `openai` is currently supported.
- `--config` / `ROBONIX_CONFIG_PATH`: optional YAML config file.
- `--log`: env_logger filter. Falls back to `RUST_LOG`, then `robonix_pilot=info`.
- `ROBONIX_PILOT_SOUL`: optional path to a SOUL markdown file. If unset, Pilot tries `~/.robonix/SOUL.md`.
- `ROBONIX_PILOT_MAX_TOOL_ROUNDS`: maximum RTDL execution rounds per turn. Defaults to `64`.

## Prompt assets

The VLM-facing RTDL envelope rules (grammar, example, constraints) live in `rtdl_protocol.md` at the crate root and are embedded at compile time via `include_str!`. Pilot sends that frozen document on the first planning round and a bounded grammar/admission reminder on later rounds. Edit the document to change the authoritative RTDL instructions without touching `planner.rs`.

Pilot's standing system prompt is built in `src/planner.rs`; it includes the runtime operating principles, including the rule that failed required capability calls stop autonomous physical task progress until the user confirms the next step.

Atlas discovery still runs before every planning round. When its LLM-visible provider, contract, description, and schema fields are unchanged, Pilot reuses the rendered capability catalog. The startup-cached Soma YAML is serialized as compact JSON while the authoritative URDF remains unchanged.

Every request writes a `[pilot/prompt]` JSON log with text bytes and four-byte token estimates for the standing context, RTDL protocol, capability catalog, task, in-flight trees, Executor state, live embodiment, environment, history, and correction. Pilot supplies a random per-turn `prompt_cache_key`, with stable sections ordered before live state, so supporting providers can reuse the longest unchanged prefix without receiving a session identifier. Compatible providers also report exact prompt/completion tokens and cached prompt tokens through streaming usage; those totals are logged separately under the same prefix. The fake VLM's reported usage is explicitly a deterministic four-byte estimate, not a production tokenizer result. A provider that rejects a named optional cache/usage field with HTTP 400 or 422 is retried once without those fields; unrelated client errors are returned unchanged.

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
- `parallel`: concurrent children. Executor waits for every child and does not cancel sibling branches when one fails.
- `do`: one capability call, where `cap` is the unique `capability_name` shown in the prompt and `args` is a JSON object.

Pilot buffers the full assistant JSON before showing user-visible text, validates the RTDL, expands it into an arena-style `Plan { nodes, root_index }`, sends that `Plan` to Liaison, and dispatches it to Executor. Executor interprets `sequence`, `parallel`, and `do` nodes directly.

Parallel example:

```json
{
  "content": "I will inspect both signals.",
  "rtdl": {
    "op": "parallel",
    "children": [
      { "op": "do", "cap": "camera_snapshot", "args": {} },
      { "op": "do", "cap": "battery_status", "args": {} }
    ]
  }
}
```

An empty sequence means the model is done for the turn:

```json
{ "content": "Done.", "rtdl": { "op": "sequence", "children": [] } }
```
