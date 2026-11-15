# robonix_api.llm

Model catalog, model resolver and per-request usage records for Robonix
consumers, shipped as the `robonix_api.llm` submodule of `robonix-api`.

Consumers resolve an endpoint from their requirements and connect directly;
the catalog adds no proxy or service. `metrics` records one JSON line per
model call so usage can be counted across consumers.

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

All requested features must be supported, and the model's `context` must fit
`input_tokens + max_output_tokens` when both are set. Matching models are
ordered by latency, then combined token price, then id.
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

Explicit configuration wins: a consumer with its own endpoint configured
(`VLM_BASE_URL`, or memory's `MEMGRAPH_LLM_BASE_URL`) keeps that endpoint and
does not consult the catalog; a key or model alone does not disable the catalog,
since the launcher forwards catalog `api_key_env` names as keys. Without an
explicit endpoint the catalog supplies endpoint, key and model.

Pilot follows the same idea with its `vlm` block rather than environment
variables: a complete block (upstream, model, key) is explicit, a bare
`vlm.model` selects a catalog id, and no block resolves by features. The
verifier accepts either a complete block or a bare model and rejects a partial
one, rather than mixing a block with catalog fields.

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
`VLM_API_KEY` and `MEMGRAPH_LLM_API_KEY` are forwarded only when their own
`*_BASE_URL` is set, so one component's credential does not reach another.
`rbnx` uses the same list for Pilot, also preserving an explicit
`ROBONIX_VLM_API_KEY` when that is the credential Pilot selected.

Containerized Scene mounts the catalog at the same absolute path and forwards only its
`api_key_env` variables, by name. Under `rbnx boot`, `ROBONIX_LLM_KEY_ENVS`
supplies that list; standalone launches derive it with the host's Python and
`robonix_api.llm` (PyYAML is needed for YAML catalogs). Without a catalog,
Scene forwards only the legacy `VLM_API_KEY` and `OPENAI_API_KEY` variables.

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

## Model usage records

Shared helpers for Robonix components that call a model. `metrics` records
one JSON line per model call so usage can be counted, and later billed,
across every component.

### Usage

```python
from robonix_api.llm import metrics

with metrics.Call("scene", "relations", base_url, model) as call:
    response = await client.post(f"{base_url}/chat/completions", json=body)
    call.finish(response)
```

`finish(response)` reads `status_code` and the token usage in `response.json()`;
call it for any answer, including an error status, so the attempt is counted.
`retry()` counts one more request sent for the same call. Leaving the block
without `finish` records `timeout`, `cancelled` or `transport_error` from
the exception, and the exception still propagates; so call `finish` before
`raise_for_status()`. A write failure is logged
once and never fails the model call. The first call logs whether records are
on, so an unset variable is visible in the component's log.

### Record

Records go to `$ROBONIX_LLM_METRICS_DIR/<consumer>-<pid>.jsonl`; `consumer`
must match `[a-z0-9_-]+`, because it names the file, and `Call` raises
`ValueError` otherwise. Nothing is written when the variable is unset.
`rbnx boot` sets it to `<deploy>/rbnx-boot/llm-metrics/`, which boot and
`rbnx clean -f` keep unless `--llm-metrics` is given. Each record is appended
with one write, so a reader sees whole lines except possibly the one being
written; `rbnx llm-usage` skips such a line and sums the rest. The files grow by
one line per call and are never rotated. Each record is a few hundred bytes,
so the directory grows with call volume; `rbnx llm-usage` reads every record it
sums, so archive or remove the directory with `rbnx clean -f --llm-metrics`
when it reaches a size you are willing to parse in one run.

| Field | Meaning |
|---|---|
| `v` | Layout version, currently 1 |
| `ts_ms`, `latency_ms` | Wall-clock end time and request duration |
| `boot_id`, `pid` | `RBNX_BOOT_ID` of the deployment and the writing process |
| `consumer`, `operation` | Component and what it used the model for |
| `host` | Endpoint host and port; never the path, query or credentials |
| `model`, `provider_model` | Model requested, and the one the response names |
| `stream` | Whether the reply was streamed |
| `status`, `http_status` | `ok` (2xx with a JSON body), `http_error`, `timeout`, `cancelled` or `transport_error` (unreadable body, or a streamed reply cut off before it finished) |
| `attempts` | Requests sent for this call, counting retries |
| `input_tokens`, `output_tokens`, `cached_input_tokens`, `reasoning_tokens` | Reported counts; `null` when not reported |

Prompts, replies and keys are never recorded. Prices are not recorded either:
cost is token counts times the operator's price list, joined on `host` +
`model` against the totals `rbnx llm-usage --json` prints. Whether a consumer
writes records depends on `ROBONIX_LLM_METRICS_DIR`: a launcher that filters the
environment must pass it through, and the startup log says when it is off.
`RBNX_BOOT_ID` only attributes a record to a boot; without it records still
appear with `boot_id: null`.

The Rust crate `system/llm-metrics` writes the same record. Both parse usage
with the cases in [contract/usage_cases.json](contract/usage_cases.json).

| Consumer | Operations |
|---|---|
| `pilot` | `plan`, `summary` |
| `scene` | `relations`, `image_relations`, `caption`, `perception` |
| `memory` | `search`, `observe`, `answer` |
| `memsearch` | `compact` |
| `vlm_verifier` | `judge` |


## Tests

```bash
PYTHONPATH=pylib/robonix-api python3 -m unittest discover \
  -s pylib/robonix-api/tests -p 'test_llm_*.py' -v
cargo test -p robonix-llm-catalog
cargo test -p robonix-llm-metrics
```

## Direct-connect smoke test

`examples/llm_deepseek_smoke.py` resolves DeepSeek from the example catalog and
calls it directly with a generated image:

```bash
export DEEPSEEK_API_KEY=...
PYTHONPATH=pylib/robonix-api python3 pylib/robonix-api/examples/llm_deepseek_smoke.py
```
