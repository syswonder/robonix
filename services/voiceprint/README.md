# robonix/service/voiceprint

ECAPA-TDNN speaker-identification service. It serves
`robonix/service/voiceprint/identify`, which Liaison's voice-gating path
calls, and the enrollment contracts `enroll`, `list`, and `delete`, which
`rbnx chat` uses to manage enrolled speakers.

## Layout

```
services/voiceprint/
  config.spec                    # manifest config schema
  package_manifest.yaml
  voiceprint_service/            # Python package
    __init__.py                  # bootstraps gRPC stubs on first import
    engine.py                    # ECAPA-TDNN wrapper
    service.py                   # gRPC servicers + entry point
  scripts/build.sh               # uv venv + codegen + model pre-download
  tests/                         # unittest suite
```

## Build

```bash
cd services/voiceprint
bash scripts/build.sh
```

The build phase:

1. Creates `rbnx-build/venv/` via `uv venv` + `uv sync`.
2. Runs `rbnx codegen --mcp` with the venv interpreter and import-tests the
   generated contract modules under `rbnx-build/codegen/`.
3. Pre-downloads ECAPA-TDNN (`speechbrain/spkrec-ecapa-voxceleb`) into
   `rbnx-build/models/`. Set `SKIP_MODEL_DOWNLOAD=1` to skip; set
   `HF_ENDPOINT=https://hf-mirror.com` (default) for runner environments where direct model downloads are unreliable.

## Run

```bash
rbnx-build/venv/bin/python -m voiceprint_service.service
```

Environment variables:

| Variable               | Default              | Meaning                              |
|------------------------|----------------------|--------------------------------------|
| `VOICEPRINT_DATA_DIR`  | `rbnx-build/data`    | location of `enrolled.json`          |
| `VOICEPRINT_THRESHOLD` | `0.25`               | cosine-similarity gate for `is_known` |
| `VOICEPRINT_DEVICE`    | auto (`cuda:0`/`cpu`) | torch device for ECAPA-TDNN          |

## Test

```bash
bash scripts/build.sh   # one-time; pulls ECAPA-TDNN weights
rbnx-build/venv/bin/python -m unittest discover -s tests -v
```

The suite covers threshold and device configuration, concurrent enrollment,
rollback when persisting fails, and model loading on activate and deactivate.

## Capability contracts

All four contracts are registered with Atlas under the `voiceprint` provider
and share one gRPC listener; `list` is also exposed over MCP:

- [`identify.v1.toml`](../../capabilities/service/voiceprint/identify.v1.toml) — `Identify`
- [`enroll.v1.toml`](../../capabilities/service/voiceprint/enroll.v1.toml) — `Enroll`
- [`list.v1.toml`](../../capabilities/service/voiceprint/list.v1.toml) — `ListEnrolled`
- [`delete.v1.toml`](../../capabilities/service/voiceprint/delete.v1.toml) — `DeleteEnrolled`

The IDL lives in [`capabilities/lib/voiceprint/srv/`](../../capabilities/lib/voiceprint/srv/).
The listen ports are picked at startup and registered with Atlas; the bind
address follows `ROBONIX_PROVIDER_BIND_HOST` (see the robonix-api README).
