# robonix_api.llm

Model catalog and model resolver for Robonix consumers, shipped as the
`robonix_api.llm` submodule of `robonix-api`.

The submodule answers one question: **given what this request needs, which
model should the consumer connect to?** It returns a model endpoint descriptor;
the consumer opens that connection itself. There is no proxy, no model traffic
and no health service here.

## ModelCatalog

A catalog is a YAML or JSON file with a `models` list:

```yaml
models:
  - id: deepseek/flash
    provider: deepseek
    base_url: https://api.deepseek.com
    model: deepseek-flash
    api_key_env: DEEPSEEK_API_KEY
    protocol: openai-chat
    features:
      vision: true
      json_object: true
      json_schema: false
      tools: true
      stream: true
    context: 131072
    latency_p95_ms: 2000
    price_input_per_mtok: 0.14
    price_output_per_mtok: 0.28
```

`api_key_env` is the name of an environment variable, never the key. The
resolver returns it so the consumer can read the key locally and connect
directly. Omit it for a keyless endpoint such as a local vLLM/Ollama server.

`catalog.example.yaml` is a complete example.

## Python

```python
from openai import OpenAI
from robonix_api.llm import ModelCatalog, Requirements, api_key_for

catalog = ModelCatalog.load("catalog.example.yaml")
targets = catalog.resolve(
    Requirements(
        features=("vision", "tools"),
        input_tokens=12000,
        max_output_tokens=512,
    ),
)
target = targets[0]

# The consumer's own OpenAI-compatible client connects directly.
client = OpenAI(
    base_url=target.base_url,
    api_key=api_key_for(target),   # "" for a keyless local endpoint
)
client.chat.completions.create(model=target.model, messages=...)
```

Matching models are ordered by latency, then combined token price, then id.
The caller tries the first and may try the next on failure.

When the request needs `json_schema` and no schema-capable model fits, the
default `allow_downgrade=True` lets a `json_object` model stand in and marks
the returned targets `downgraded=True`. Pass `allow_downgrade=False` to require
a schema-capable model or get an empty list.

## Consumer integration

Consumers that keep their own HTTP client can resolve their configuration from
the deployment catalog and then connect directly:

```python
from robonix_api.llm import Requirements, api_key_for, resolve_from_env

target = resolve_from_env(Requirements(features=("vision",)))
if target is not None:
    base_url = target.base_url
    model = target.model
    api_key = api_key_for(target)   # "" when the key env is unset
else:
    ...                             # keep the consumer's legacy VLM_* config
```

`resolve_from_env` reads `ROBONIX_LLM_CATALOG` and returns the first usable
target. Targets without `api_key_env` need no credential; targets that name an
unset key are skipped. If no target is usable, it raises an error naming the
missing variables. When the catalog variable is unset it returns `None`, which
keeps the legacy configuration. The catalog is read once per process; restart
the consumer after editing it.

### Deployment

Under `rbnx boot`, the catalog normally comes from the deployment manifest's
`model_catalog:` path or the user-level `~/.robonix/models.yaml`. rbnx
validates it, resolves Pilot's target, and exports `ROBONIX_LLM_CATALOG` plus
`ROBONIX_LLM_KEY_ENVS` (the exact `api_key_env` names). Standalone consumers
can set `ROBONIX_LLM_CATALOG` directly to an absolute catalog path and export
the `api_key_env` of every target they may use (keyless local targets need no
variable). Consumers without the variable keep their legacy configuration, so
the migration can be rolled out one service at a time.

Scene runs in a container: its start script mounts the catalog at the same
absolute path and forwards `DEEPSEEK_API_KEY`, `OPENAI_API_KEY`,
`DASHSCOPE_API_KEY`, `VLLM_API_KEY`, `ANTHROPIC_API_KEY` and `GEMINI_API_KEY`.
The catalog is read once per process; restart a consumer after editing it.

The requirements used by the in-tree consumers:

| Consumer | Features |
|---|---|
| Pilot | `vision, tools, stream` (schema is downgraded at request time) |
| Scene relation/caption | `vision` |
| Scene perception | `vision` |
| Memory search | `json_object` |
| Memory observe | `vision` |
| Memsearch compact | text (no hard feature) |
| VLM Verifier | `vision` |
| CI diagnose | `json_object` |
| CI label judge | text (no hard feature) |

## Rust

The Rust implementation lives in `system/llm-catalog` and reads the same
catalog with the same rules. The shared cases in
`robonix_api/llm/contract/resolve_cases.json` are run by both test suites.

## Tests

```bash
PYTHONPATH=pylib/robonix-api python3 -m unittest discover \
  -s pylib/robonix-api/tests -p 'test_llm_*.py' -v
cargo test -p robonix-llm-catalog
```

## Direct-connect smoke test

`examples/llm_deepseek_smoke.py` resolves DeepSeek from the example catalog and
calls it directly with a generated image:

```bash
export DEEPSEEK_API_KEY=...
PYTHONPATH=pylib/robonix-api python3 pylib/robonix-api/examples/llm_deepseek_smoke.py
```

## Deliberately not included

Health tracking, circuit breakers, usage reports, decision caching, image
budget enforcement and an HTTP service. Those belong to a consumer or to an
optional gateway and should be added only when a concrete requirement needs
them.
