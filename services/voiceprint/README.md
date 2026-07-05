# robonix/system/voiceprint

ECAPA-TDNN speaker-identification service. Implements the frozen v0.1
capability `robonix/service/voiceprint/identify` (one-shot `Identify`)
consumed by Liaison's voice-gating path, and adds two package-private
admin RPCs (`Enroll`, `ListEnrolled`) for off-line enrollment tooling.

## Layout

```
system/voiceprint/
  proto/voiceprint.proto         # package-private gRPC schema
  voiceprint_service/            # Python package
    __init__.py                  # bootstraps gRPC stubs on first import
    engine.py                    # ECAPA-TDNN wrapper
    service.py                   # gRPC servicer + entry point
  scripts/build.sh               # uv venv + codegen + model pre-download
  tests/                         # unittest suite (Enroll → Identify → Reject)
```

## Build

```bash
cd system/voiceprint
bash scripts/build.sh
```

The build phase:

1. Creates `rbnx-build/venv/` via `uv venv` + `uv sync`.
2. Generates gRPC stubs from `proto/voiceprint.proto` into `rbnx-build/codegen/proto_gen/`.
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
| `VOICEPRINT_PORT`      | `50092`              | gRPC listen port                     |
| `VOICEPRINT_BIND_ADDR` | `0.0.0.0`            | gRPC bind address                    |
| `VOICEPRINT_DATA_DIR`  | `rbnx-build/data`    | location of `enrolled.json`          |
| `VOICEPRINT_THRESHOLD` | `0.25`               | cosine-similarity gate for `is_known` |
| `VOICEPRINT_DEVICE`    | auto (`cuda:0`/`cpu`) | torch device for ECAPA-TDNN          |

## Test

```bash
bash scripts/build.sh   # one-time; pulls ECAPA-TDNN weights
rbnx-build/venv/bin/python -m unittest discover -s tests -v
```

The suite mirrors the original out-of-tree `test_tui.py` voiceprint-gating
flow: enrol `liukaile` + `com`, identify the matching second sample of
each as accepted, identify `change.wav` (un-enrolled) as rejected.

## Capability contract

This package serves
[`capabilities/system/speech/voiceprint.v1.toml`](../../capabilities/system/speech/voiceprint.v1.toml),
whose IDL is
[`capabilities/lib/voiceprint/srv/Identify.srv`](../../capabilities/lib/voiceprint/srv/Identify.srv).
The two extra RPCs (`Enroll`, `ListEnrolled`) are package-private and are
NOT registered as Atlas capabilities; they exist only on the local
service port for admin tooling.
