# robonix-pilot

`robonix-pilot` is the VLM-driven reasoning and planning service for Robonix. It receives `SystemPilot` tasks, builds the model prompt from the current Atlas capability catalog, asks the VLM for an RTDL plan, expands that RTDL into a `Plan`, and sends the plan to `robonix-executor`.

## Build

From the repo root:

```sh
cargo build -p robonix-pilot
```

The crate is part of the top-level Cargo workspace and is built / installed by `make build` and `make install` respectively.

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
- `--vlm-context-window-tokens` / `ROBONIX_VLM_CONTEXT_WINDOW_TOKENS`: optional operator override for the deployed model's total context capacity.
- `--vlm-format` / `ROBONIX_VLM_FORMAT`: API dialect. Only `openai` is currently supported.
- `--config` / `ROBONIX_CONFIG_PATH`: optional YAML config file.
- `--log`: env_logger filter. Falls back to `RUST_LOG`, then `robonix_pilot=info`.
- `ROBONIX_PILOT_SOUL`: optional path to a SOUL markdown file. If unset, Pilot tries `~/.robonix/SOUL.md`.
- `ROBONIX_PILOT_MAX_TOOL_ROUNDS`: maximum RTDL execution rounds per turn. Defaults to `64`.

## Context capacity: automatic, visible, and overrideable

Pilot compacts history before the next request would exceed the model's total context capacity. This is not the same thing as `max_tokens`: it is the full input + output budget.

History's room is the context window minus this round's non-history context (the system prefix and the current runtime sections) and a fixed 6,144-token reserve for the next reply and estimation error. When history outgrows its room, one compaction leaves behind at most:

| Kept | Share of the room | Form |
| --- | --- | --- |
| The most recent messages | 50% | verbatim; no single message over 20% of the room |
| The current task's own words, if they fell out of that tail | 10% | verbatim |
| Everything older, including the previous summary | 15%, and at most 6,000 tokens | one rolling summary written by the model |

Together that is at most 75% of the room, so every compaction frees at least a quarter of it and the next one cannot follow immediately. If the summarizer fails, the older messages are still dropped behind a note saying so, and the previous summary is kept; compaction never leaves history as large as it found it. After a compaction the next round re-sends the full capability catalog, Soma body, and doc index, because the summary may have absorbed the earlier copies. Each compaction writes a `[pilot/compaction]` record with the room, the caps, and the history size before and after.

Pilot resolves the capacity in this order:

1. A manual deployment value (`vlm.context_window_tokens`, the environment variable, or the CLI flag). This is authoritative.
2. Provider metadata from `GET /models/{id}`.
3. Provider metadata from `GET /models`: first the exact id, then a unique canonical terminal name. This handles a gateway advertising `anthropic/claude-opus-5.5` when the deployment configured `claude-opus-5.5`. Two different providers with the same terminal name are intentionally treated as ambiguous, never guessed.
4. Otherwise Pilot starts without pre-emptive history compaction. A manual declaration enables predictable compaction for an unknown/private model.

The startup `[pilot/context_budget]` JSON record includes the configured model, the selected provider id when a canonical match was used, capacity, and source. Canonical matches emit a warning asking the operator to verify the gateway route. At `rbnx boot`, `context=... (manual)` means an operator override; otherwise `context=auto` means Pilot will resolve and log the real source after startup.

Set a manual value only when the route is private, metadata is missing or ambiguous, or the gateway has a smaller server cap:

```yaml
system:
  pilot:
    vlm:
      upstream: https://provider.example/v1
      api_key: ${VLM_API_KEY}
      model: local-vision-planner
      context_window_tokens: 32768 # manual: inspect the model card/server
```

Use the smaller effective limit when a model card and an inference-server setting (such as `max_model_len` or `max_seq_len`) disagree.

## Prompt assets

The VLM-facing RTDL envelope rules live in `rtdl_protocol.md` and are embedded via `include_str!`. Planning uses the same bounded grammar/admission reminder on every round to preserve the cacheable prefix; corrective retries can include the full protocol.

Pilot's standing system prompt is built in `src/planner.rs`. Prompt assembly and usage accounting live in `src/prompt.rs`; each round's section list supplies both context-budget estimates and request rendering.

Pilot's standing prompt is deployment-independent. It does not hard-code Scene, memory, chassis, or another provider family's semantics; those come from the current Atlas capability catalog and the provider documentation loaded through `read_capability_doc` when needed. A provider appearing, disappearing, or reconnecting therefore does not rewrite the standing system prefix.

Atlas discovery still runs before every planning round. Pilot omits contracts whose `[contract]` metadata sets `llm_callable = false` from the model-facing catalog and target map, while leaving them available to Executor and other Atlas consumers. An omitted field, including metadata from an older Atlas, defaults to visible. When the remaining provider, contract, description, and schema fields are unchanged, Pilot reuses the rendered capability catalog. The startup-cached Soma YAML is serialized as compact JSON while the authoritative URDF remains unchanged.

A plan names a capability by the exact catalog entry `<provider_id>.<llm_name>`. Contract ids remain runtime identifiers for Atlas, contracts, and internal configuration; they are not a second model-facing spelling. Pilot omits a capability from both the catalog and target map when its contract metadata sets `llm_callable = false`.

A planning request is split so that its prefix is cacheable. The system message carries only what never changes, Pilot rules and the RTDL contract, before the ordered conversation history. Everything that can change lives in history. Memory and an explicit response mode are appended once for each submitted task; every task records either voice or text mode, so a later text request supersedes voice-only constraints without rewriting history. The callable capability catalog, the Soma body data, and the provider-doc index are re-read before every planning round. The first round of each task, and the first round after history compaction, appends each of them in full; later rounds append only what changed, and for the catalog that is a list of the capabilities added, changed, or removed. A snapshot counts as shown only once its round is written to history, so a round dropped for a steer or a stale plan does not leave the model without it. A provider registering or leaving therefore never invalidates the cached prefix. Live embodiment health, in-flight trees, Executor state, and environment state are appended on every round, followed by the assistant reply and any task-state/result records. The next request therefore extends rather than reorders the prior message sequence. Prompt logs include both the cacheable-prefix byte length and fingerprint, making accidental prefix churn observable.

Both Soma blocks are projected onto what a planner acts on. From the body description that drops the `urdf` file reference — a path in the provider's own filesystem namespace, pointing at the kinematics this crate already declines to inject — the `footprint` collision polygon that navigation rather than the planner consumes, and the description of each exported capability, which the capability catalog in the same prompt already carries verbatim; the component-to-capability mapping stays, since the catalog does not say which part of the body offers a capability. From the live health snapshot it drops fields at an absent value, which the prompt's own "missing means unknown" rule already covers. `dimensions`, sensor `placement`, `can_do` / `cannot_do`, and the deployment notes are body facts and remain.

The catalog prints each capability's name, the opening paragraph of its description, and its argument schema. A description longer than that paragraph — a provider that wrote its request/response manual into the description field rather than a `CAPABILITY.md` — is summarized, and the entry carries a `more:` line naming the `provider_id` to pass to the Executor's `read_capability_doc` builtin for the full text. The catalog ships on every planning request while a manual ships only when the model decides it needs that provider, so one verbose provider cannot tax every call for the whole system.

Every request writes a `[pilot/prompt]` JSON log with text bytes and four-byte token estimates for standing context, RTDL protocol, current runtime-context sections (including the capability catalog record, when there is one), and full ordered history. Pilot supplies one opaque `prompt_cache_key` for the **lifetime of the running Pilot deployment**, rather than one key per task: the unchanged system prefix and completed histories can route to the same cache across independent tasks without using a user/session identifier. The message-ordering test asserts that a later normal planning request is an exact extension of the prior wire-message sequence; this is the code-level reusable-prefix invariant. Compatible providers also report exact input, output, total, cached-input, and uncached-input tokens through streaming usage. Pilot logs those values per request together with `cache_hit` and `cache_hit_ratio`, plus cumulative token totals, cache-hit request counts, and the cumulative cache-hit ratio for the interaction. Provider-reported cache ratio remains a runtime measurement: routing, TTL expiry, provider cache granularity, and a newly appended suffix make it distinct from code-level prefix reuse. Pricing is not hard-coded because an OpenAI-compatible proxy may route the same model name to differently priced backends; the logged totals can be priced against the operator's actual provider contract. The fake VLM's reported usage is explicitly a deterministic four-byte estimate, not a production tokenizer result. A provider that rejects a named optional cache/usage field with HTTP 400 or 422 is retried once without those fields; unrelated client errors are returned unchanged.

The `info` records above carry sizes and token counts, never prompt text. To see exactly what the model receives and returns, raise Pilot's log level to `debug`: set `log: debug` on the Pilot component in the deployment manifest, or `SCRIBE_FILE_LEVEL=debug` for the log file and `SCRIBE_CONSOLE_LEVEL=debug` for the terminal. Each planning request then also writes `[pilot/prompt/messages]`, the full ordered message sequence sent to the provider with inline images reduced to their byte size, and each reply writes `[pilot/rtdl/raw]`, the model output as received before parsing. These records contain user tasks and scene contents verbatim, so enable them only while debugging.

## Session transcripts

Pilot keeps every session's conversation in an append-only JSON-lines file, `<log dir>/pilot-sessions/<session_id>.jsonl`, where the log directory is Scribe's (`$SCRIBE_LOG_DIR`, or `./logs`). History compaction removes old messages from what the model sees; the transcript keeps all of them, so no interaction is lost. Each line has `ts` and `kind`: `session` (the first line, with the session id and Pilot version), `message` (one history message; an image is recorded by size only, as `image_bytes`), `task_state` (the task state after it changed), and `compaction` (the full history that replaced the old one, with how many messages were evicted and pinned).

When Pilot receives a task for a session it has no history for, such as after a restart, it rebuilds that session from the transcript: the last `compaction` record's history, or an empty one, followed by every later `message`, plus the last `task_state`. Images are not restored. A line that does not parse, such as one cut short by a crash, is skipped with a warning. A failed write is logged and retried with the next record, and never stops the turn. There is no retention policy yet; delete old files by hand.

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

Pilot forwards `VERIFYING` node states for live visibility but does not put them in LLM history or `BatchResult`. Once verification finishes, the final leaf result is written to history exactly once; a final failure triggers replanning immediately. `BatchResult` contains only the latest final state for each node.

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
