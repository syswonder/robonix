# robonix_api.llm

Model catalog and model resolver for Robonix consumers, shipped as the
`robonix_api.llm` submodule of `robonix-api`.

Consumers resolve an endpoint from their requirements and connect directly;
the catalog adds no proxy or service.

## ModelCatalog

A catalog is a YAML or JSON file with a `models` list:

```yaml
models:
  - id: local/vision
    provider: vllm
    base_url: http://127.0.0.1:8000/v1
    model: vision
    features: {vision: true, tools: true, stream: true}
```

`api_key_env` is the name of an environment variable, never the key. The
resolver returns it so the consumer can read the key locally and connect
directly. Omit it for a keyless endpoint such as a local vLLM/Ollama server.

[catalog.example.yaml](catalog.example.yaml) also shows protocol, context,
latency and price fields.

## Python

```python
from openai import OpenAI
from robonix_api.llm import ModelCatalog, Requirements, api_key_for

catalog = ModelCatalog.load("catalog.example.yaml")
targets = catalog.resolve(
    Requirements(features=("vision", "tools")),
)
target = targets[0]

# The consumer's own OpenAI-compatible client connects directly.
client = OpenAI(
    base_url=target.base_url,
    api_key=api_key_for(target),   # "" for a keyless local endpoint
)
client.chat.completions.create(model=target.model, messages=...)
```

All requested features must be supported. Matching models are ordered by
latency, then combined token price, then id.
The caller tries the first and may try the next on failure.

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

Native Python package hooks launch through
`python -m robonix_api.llm MODULE [ARGS...]`. With a catalog configured, this
keeps only the variables in [runtime_env.txt](runtime_env.txt) and the catalog's
`api_key_env` names before executing the consumer. The list includes Python,
ROS, logging, cache paths and consumer settings. Additional runtime variables
must be added explicitly; arbitrary inherited variables are dropped.
`rbnx` uses the same list for Pilot, also preserving an explicit
`ROBONIX_VLM_API_KEY` when that is the credential Pilot selected.

Containerized Scene mounts the catalog at the same absolute path and forwards only its
`api_key_env` variables, by name. Under `rbnx boot`, `ROBONIX_LLM_KEY_ENVS`
supplies that list; standalone launches derive it with the host's Python and
`robonix_api.llm` (PyYAML is needed for YAML catalogs). Without a catalog,
Scene forwards only the legacy `VLM_API_KEY` and `OPENAI_API_KEY` variables.
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
