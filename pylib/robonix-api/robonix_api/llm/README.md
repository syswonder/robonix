# robonix_api.llm

Shared helpers for Robonix components that call a model. `metrics` records
one JSON line per model call so usage can be counted, and later billed,
across every component.

## Usage

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

## Record

Records go to `$ROBONIX_LLM_METRICS_DIR/<consumer>-<pid>.jsonl`; `consumer`
must match `[a-z0-9_-]+`, because it names the file, and `Call` raises
`ValueError` otherwise. Nothing is written when the variable is unset.
`rbnx boot` sets it to `<deploy>/rbnx-boot/llm-metrics/`, which boot and
`rbnx clean -f` keep unless `--llm-metrics` is given. Each record is appended
with one write, so a reader sees whole lines except possibly the one being
written; `rbnx llm-usage` skips such a line and sums the rest. The files grow by
one line per call and are never rotated: archive or remove them with
`rbnx clean -f --llm-metrics` once their totals are taken.

| Field | Meaning |
|---|---|
| `v` | Layout version, currently 1 |
| `ts_ms`, `latency_ms` | Wall-clock end time and request duration |
| `boot_id`, `pid` | `RBNX_BOOT_ID` of the deployment and the writing process |
| `consumer`, `operation` | Component and what it used the model for |
| `host` | Endpoint host and port; never the path, query or credentials |
| `model`, `provider_model` | Model requested, and the one the response names |
| `stream` | Whether the reply was streamed |
| `status`, `http_status` | `ok` (2xx), `http_error`, `timeout`, `cancelled` or `transport_error` (also a streamed reply cut off before it finished) |
| `attempts` | Requests sent for this call, counting retries |
| `input_tokens`, `output_tokens`, `cached_input_tokens`, `reasoning_tokens` | Reported counts; `null` when not reported |

Prompts, replies and keys are never recorded. Prices are not recorded either:
cost is token counts times the operator's price list, joined on `host` +
`model` against the totals `rbnx llm-usage --json` prints. A launcher that
filters the environment must pass `ROBONIX_LLM_METRICS_DIR` and `RBNX_BOOT_ID`
through; otherwise its consumers write nothing, which their startup log says.

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
PYTHONPATH=pylib/robonix-api python3 -m unittest pylib/robonix-api/tests/test_llm_metrics.py
cargo test -p robonix-llm-metrics
```
