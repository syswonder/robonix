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
- `--vlm-context-window-tokens` / `ROBONIX_VLM_CONTEXT_WINDOW_TOKENS`: total
  context capacity for this deployed model. Required for a model that Pilot
  cannot identify from provider metadata or its checked built-in registry.
- `--vlm-reserved-output-tokens` / `ROBONIX_VLM_RESERVED_OUTPUT_TOKENS`:
  capacity retained for the next planner reply (default: 4096).
- `--vlm-context-safety-tokens` / `ROBONIX_VLM_CONTEXT_SAFETY_TOKENS`:
  conservative framing/reasoning headroom (default: 2048).
- `--vlm-format` / `ROBONIX_VLM_FORMAT`: API dialect. Only `openai` is currently supported.
- `--config` / `ROBONIX_CONFIG_PATH`: optional YAML config file.
- `--log`: env_logger filter. Falls back to `RUST_LOG`, then `robonix_pilot=info`.
- `ROBONIX_PILOT_SOUL`: optional path to a SOUL markdown file. If unset, Pilot tries `~/.robonix/SOUL.md`.
- `ROBONIX_PILOT_MAX_TOOL_ROUNDS`: maximum RTDL execution rounds per turn. Defaults to `64`.

## Context capacity: required, visible, and reviewable

Pilot compacts history before the next request would exceed the model's total
context capacity. This is not the same thing as `max_tokens`: it is the full
input + output budget. An OpenAI-compatible `/models` endpoint normally
returns only a model id and owner, so Pilot cannot safely infer this number for
an arbitrary proxy, local model, or provider alias.

Pilot resolves the capacity in this order:

1. A manual deployment value (`vlm.context_window_tokens`, the environment
   variable, or the CLI flag). This is authoritative.
2. A provider-specific extension on `GET /models/{id}` or `GET /models` for
   the exact selected id, when it exposes a context field.
3. An exact built-in direct-provider match listed below.
4. Otherwise Pilot starts in `learning` mode without pre-emptive history
   compaction. A manual declaration remains the operator override for a
   predictable long-running deployment.

| Exact model names | Registered context | Meaning |
| --- | ---: | --- |
| `gpt-5.6`, `gpt-5.6-sol`, `gpt-5.6-terra`, `gpt-5.6-luna` | 1,050,000 | OpenAI direct-provider profile |
| `gpt-4.1`, `gpt-4.1-mini`, `gpt-4.1-nano` | 1,000,000 | OpenAI direct-provider profile |
| `gpt-4o`, `gpt-4o-mini` | 128,000 | OpenAI direct-provider profile |
| `gemini-2.5-pro`, `gemini-2.5-flash` | 1,048,576 | Published input limit used conservatively |
| `claude-sonnet-4-5`, `claude-sonnet-4-5-20250929` | 200,000 | Direct API / Bedrock profile |

The registry is deliberately exact-name only. A gateway may call a smaller,
quantized, or differently capped backend `gpt-4o`; a built-in match therefore
logs `source=builtin_registry` and a prominent **verify/override** warning.
For any gateway, local server, custom deployment, or mismatched alias, set the
real server limit explicitly:

```yaml
system:
  pilot:
    vlm:
      upstream: https://provider.example/v1
      api_key: ${VLM_API_KEY}
      model: local-vision-planner
      context_window_tokens: 32768 # manual: inspect the model card/server
      reserved_output_tokens: 4096
      context_safety_tokens: 2048
```

Find that number in the model card's **context length/window** field, the
inference-server launch/configuration (`max_model_len`, `max_seq_len`, or the
vendor equivalent), or the provider's model page. Use the smaller effective
limit when these disagree. At `rbnx boot`, the Pilot line prints either
`context=… (manual)` or `context=… (auto: …; verify/override if proxied)`;
unknown models are marked `context=learning` at boot. The Pilot log emits
the corresponding JSON `context_window_source` and `registry_match` fields.

## Prompt assets

The VLM-facing RTDL envelope rules (grammar, example, constraints) live in `rtdl_protocol.md` at the crate root and are embedded at compile time via `include_str!`. Pilot sends that frozen document on the first planning round and a bounded grammar/admission reminder on later rounds. Edit the document to change the authoritative RTDL instructions without touching `planner.rs`.

Pilot's standing system prompt is built in `src/planner.rs`; it includes the runtime operating principles, including the rule that failed required capability calls stop autonomous physical task progress until the user confirms the next step.

Pilot's standing prompt is deployment-independent. It does not hard-code Scene, memory, chassis, or another provider family's semantics; those come from the current Atlas capability catalog and the provider documentation loaded through `read_capability_doc` when needed. A provider appearing, disappearing, or reconnecting therefore does not rewrite the standing system prefix.

Atlas discovery still runs before every planning round. Pilot omits contracts whose `[contract]` metadata sets `llm_callable = false` from the model-facing catalog and target map, while leaving them available to Executor and other Atlas consumers. An omitted field, including metadata from an older Atlas, defaults to visible. When the remaining provider, contract, description, and schema fields are unchanged, Pilot reuses the rendered capability catalog. The startup-cached Soma YAML is serialized as compact JSON while the authoritative URDF remains unchanged.

A plan names a capability by the exact catalog entry `<provider_id>.<llm_name>`. Contract ids remain runtime identifiers for Atlas, contracts, and internal configuration; they are not a second model-facing spelling. Pilot omits a capability from both the catalog and target map when its contract metadata sets `llm_callable = false`.

A planning request is split so that its prefix is cacheable. The system message carries Pilot rules, the RTDL contract, and the current callable capability catalog, all before the ordered conversation history. Each round appends its complete runtime observation — Soma body and live state, memory, provider-doc index, voice mode, in-flight trees, Executor state, and environment — as a user history record, then appends the assistant reply and any task-state/result records. The next request therefore extends rather than reorders the prior message sequence. User tasks and steers, task-state changes, dispatch records, execution feedback, and plan-control outcomes are authoritative history records: normal bounding evicts only disposable narration, and rolling compaction retains those source records beside its summary. Prompt logs include both the cacheable-prefix byte length and fingerprint, making accidental prefix churn observable.

Both Soma blocks are projected onto what a planner acts on. From the body description that drops the `urdf` file reference — a path in the provider's own filesystem namespace, pointing at the kinematics this crate already declines to inject — the `footprint` collision polygon that navigation rather than the planner consumes, and the description of each exported capability, which the capability catalog in the same prompt already carries verbatim; the component-to-capability mapping stays, since the catalog does not say which part of the body offers a capability. From the live health snapshot it drops fields at an absent value, which the prompt's own "missing means unknown" rule already covers. `dimensions`, sensor `placement`, `can_do` / `cannot_do`, and the deployment notes are body facts and remain.

The catalog prints each capability's name, the opening paragraph of its description, and its argument schema. A description longer than that paragraph — a provider that wrote its request/response manual into the description field rather than a `CAPABILITY.md` — is summarized, and the entry carries a `more:` line naming the `provider_id` to pass to the Executor's `read_capability_doc` builtin for the full text. The catalog ships on every planning request while a manual ships only when the model decides it needs that provider, so one verbose provider cannot tax every call for the whole system.

Every request writes a `[pilot/prompt]` JSON log with text bytes and four-byte token estimates for standing context, RTDL protocol, capability catalog, current runtime-context sections, and full ordered history. Pilot supplies one opaque `prompt_cache_key` for the **lifetime of the running Pilot deployment**, rather than one key per task: the unchanged system prefix and completed histories can route to the same cache across independent tasks without using a user/session identifier. The message-ordering test asserts that a later normal planning request is an exact extension of the prior wire-message sequence; this is the code-level reusable-prefix invariant. Compatible providers also report exact input, output, total, cached-input, and uncached-input tokens through streaming usage. Pilot logs those values per request together with `cache_hit` and `cache_hit_ratio`, plus cumulative token totals, cache-hit request counts, and the cumulative cache-hit ratio for the interaction. Provider-reported cache ratio remains a runtime measurement: routing, TTL expiry, provider cache granularity, and a newly appended suffix make it distinct from code-level prefix reuse. Pricing is not hard-coded because an OpenAI-compatible proxy may route the same model name to differently priced backends; the logged totals can be priced against the operator's actual provider contract. The fake VLM's reported usage is explicitly a deterministic four-byte estimate, not a production tokenizer result. A provider that rejects a named optional cache/usage field with HTTP 400 or 422 is retried once without those fields; unrelated client errors are returned unchanged.

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

Pilot forwards `VERIFYING` node states for live visibility but does not put
them in LLM history or `BatchResult`. Once verification finishes, the final
leaf result is written to history exactly once; a final failure triggers
replanning immediately. `BatchResult` contains only the latest final state for
each node.

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
