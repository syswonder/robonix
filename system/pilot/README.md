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

Sections that instruct the model about an optional capability family are emitted only when this deployment registered it and the planning model may call it, decided from the contract ids Atlas reports for the turn: Scene resolution rules need a `/scene/` contract, the long-term-memory note a `/memory/` one, and the chassis burst figures a `/chassis/` one. Discovery therefore runs before the standing prompt is built. A body without those families is not told about them: the rules would name tools that do not exist, and the burst figures are actively wrong for a body whose only movement capability is one discrete navigation action.

Atlas discovery still runs before every planning round. Pilot omits contracts whose `[contract]` metadata sets `llm_callable = false` from the model-facing catalog and target map, while leaving them available to Executor and other Atlas consumers. An omitted field, including metadata from an older Atlas, defaults to visible. When the remaining provider, contract, description, and schema fields are unchanged, Pilot reuses the rendered capability catalog. The startup-cached Soma YAML is serialized as compact JSON while the authoritative URDF remains unchanged.

A plan may name a capability either by its catalog entry, `<provider_id>.<llm_name>`, or by its contract id. The contract id is what Atlas logs, what the capability file declares, and what skill documentation quotes, so a plan that writes one is naming a capability the runtime has; it resolves whenever exactly one visible provider offers it and it does not shadow a catalog name. With two providers behind the same contract id the alias would have to choose between them, so only the catalog name resolves. A capability the contract metadata withholds from the model is absent from both spellings.

With `ROBONIX_PILOT_AUTO_CAMERA_OBSERVATION` set, Pilot reads one camera frame through the registered Camera Primitive before every planning round and appends it after the history, so the current view is the most recent image the model sees. It is captured per round rather than per turn because the robot moves between rounds. `ROBONIX_PILOT_OBSERVATION_CAMERA_PROVIDER` picks the camera when several are registered. The flag is off by default: a body whose planner does not need per-round vision should not pay for an image on every request, and every failure path degrades the round to what it was without an observation rather than ending the turn.

Both Soma blocks are projected onto what a planner acts on. From the body description that drops the `urdf` file reference — a path in the provider's own filesystem namespace, pointing at the kinematics this crate already declines to inject — the `footprint` collision polygon that navigation rather than the planner consumes, and the description of each exported capability, which the capability catalog in the same prompt already carries verbatim; the component-to-capability mapping stays, since the catalog does not say which part of the body offers a capability. From the live health snapshot it drops fields at an absent value, which the prompt's own "missing means unknown" rule already covers. `dimensions`, sensor `placement`, `can_do` / `cannot_do`, and the deployment notes are body facts and remain.

The catalog prints each capability's name, the opening paragraph of its description, and its argument schema. A description longer than that paragraph — a provider that wrote its request/response manual into the description field rather than a `CAPABILITY.md` — is summarized, and the entry carries a `more:` line naming the `provider_id` to pass to the Executor's `read_capability_doc` builtin for the full text. The catalog ships on every planning request while a manual ships only when the model decides it needs that provider, so one verbose provider cannot tax every call for the whole system.

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
