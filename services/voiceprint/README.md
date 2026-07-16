# Robonix Voiceprint Service

The Voiceprint service identifies an enrolled speaker from a short audio
sample. It uses SpeechBrain's ECAPA-TDNN speaker-recognition model and stores
one 192-dimensional embedding per enrolled user. The model weights come from
the ModelScope repository `speechbrain/spkrec-ecapa-voxceleb` and are warmed
during the package build so normal startup does not need to download them.

The package lives at `services/voiceprint` in the Robonix source tree. It uses
the standard Robonix capability definitions and `rbnx codegen`; it does not
carry or compile a package-private `.proto` file.

## Capability surface

The lifecycle interface and four Voiceprint RPCs are attached to one provider,
`voiceprint`, and registered with Atlas under the
`robonix/service/voiceprint` namespace.

| Contract | Request | Result | Behavior |
| --- | --- | --- | --- |
| `robonix/service/voiceprint/driver` | Lifecycle command and instance config | Lifecycle result | INIT validates configuration and opens the enrollment database; ACTIVATE loads the model, while DEACTIVATE releases it. |
| `robonix/service/voiceprint/identify` | Audio bytes, encoding, and sample rate | User ID, display name, cosine confidence, known flag, and error | Returns the closest enrolled speaker; `is_known` is true only when the score reaches the configured threshold. |
| `robonix/service/voiceprint/enroll` | User ID, display name, and audio | Success and error | Persists a new embedding. Duplicate ID, display name, or matching voice is rejected. |
| `robonix/service/voiceprint/list` | Empty request | JSON user list, count, and error | Lists enrolled user IDs and display names. This operation is also exposed through MCP. |
| `robonix/service/voiceprint/delete` | User ID | Success and error | Removes an enrollment. Deleting an absent ID is an idempotent success. |

The contract metadata is in
[`capabilities/service/voiceprint/`](../../capabilities/service/voiceprint/),
and the request and response definitions are in
[`capabilities/lib/voiceprint/srv/`](../../capabilities/lib/voiceprint/srv/).
Accepted per-instance configuration is documented in
[`config.spec`](config.spec).
The service accepts `pcm_s16le` and WAV input. An empty encoding defaults to
`pcm_s16le`, a zero sample rate defaults to 16 kHz, and other sample rates are
resampled to 16 kHz before inference.

## Source layout

```text
services/voiceprint/
├── package_manifest.yaml       # package build, start, stop, and catalog data
├── pyproject.toml              # Python dependencies resolved by the root uv.lock
├── scripts/
│   └── build.sh                # venv, codegen, import check, and model warm-up
└── voiceprint_service/
    ├── __init__.py             # package marker; no import-time code generation
    ├── engine.py               # ModelScope + SpeechBrain ECAPA-TDNN wrapper
    └── service.py              # lifecycle hooks and the four RPC servicers
```

Generated and runtime files are placed under `services/voiceprint/rbnx-build/`:

```text
rbnx-build/
├── venv/                       # uv-managed Python environment
├── codegen/
│   ├── proto_gen/              # generated protobuf and gRPC modules
│   └── robonix_mcp_types/      # generated MCP dataclasses
├── models/spkrec-ecapa-voxceleb/
└── data/enrolled.json          # default enrollment database after first write
```

## Build

Install Robonix and `uv`, configure the source tree once, then build the
package from the repository root:

```bash
cd /path/to/robonix
rbnx setup "$PWD"
rbnx build -p services/voiceprint
```

The build performs these steps:

1. Creates `rbnx-build/venv` and synchronizes the root workspace lock with
   `uv`.
2. Runs `rbnx codegen --mcp` against the standard capability and IDL trees.
   Python stubs are generated with the same virtual environment used at
   runtime, then the complete service entry point is imported immediately to
   catch protobuf, gRPC, and generated-servicer incompatibility.
3. Resolves `speechbrain/spkrec-ecapa-voxceleb` through ModelScope, loads the
   ECAPA-TDNN model, and verifies that a 192-dimensional embedding can be
   produced.

On Jetson, the build reuses a CUDA-enabled Torch installation supplied for the
board instead of installing an unrelated PyPI CUDA stack. A working
JetPack-compatible Torch build is required for GPU acceleration; when it is not
available, Voiceprint continues on CPU with higher latency.

The following variables affect the build only:

| Variable | Default | Meaning |
| --- | --- | --- |
| `UV_INDEX_URL` | `https://pypi.tuna.tsinghua.edu.cn/simple` | Python package index used by `uv`. |
| `PIP_INDEX_URL` | `https://pypi.tuna.tsinghua.edu.cn/simple` | Python package index used by `pip`-compatible operations. |
| `SKIP_MODEL_DOWNLOAD` | unset | Set to `1` only for a build that does not need to start Voiceprint. A later start still needs a cached model or network access. |
| `VOICEPRINT_SKIP_JETSON_TORCH` | unset | Set to `1` to disable linking the host JetPack Torch installation into the package venv. |
| `VOICEPRINT_HOST_PYTHON` | `python3` | Host Python inspected for a CUDA-enabled Torch installation on Jetson. |

`rbnx build --clean -p services/voiceprint` removes the complete
`rbnx-build` directory first. If the default data directory is in use, that
also removes `enrolled.json`; use a persistent `data_dir` before clean builds
on a deployed robot.

## Deploy and start

The normal deployment path is a robot manifest. A minimal entry is:

```yaml
service:
  - name: voiceprint
    path: ${ROBONIX_SOURCE_PATH}/services/voiceprint
    config:
      data_dir: /path/to/persistent/voiceprint
      threshold: 0.25
      device: cuda:0
```

`rbnx boot` starts the package, sends the `config` mapping through
`Driver(CMD_INIT)` to validate it and open the enrollment database, then sends
`Driver(CMD_ACTIVATE)` to load the model and activate the provider. The Robonix
service framework chooses a free gRPC port, binds it on
all interfaces, and advertises the selected endpoint to Atlas; this package
does not use a fixed `VOICEPRINT_PORT` or `VOICEPRINT_BIND_ADDR`.

For standalone package development, start Atlas first and then run:

```bash
cd /path/to/robonix
rbnx start \
  -p services/voiceprint \
  --endpoint 127.0.0.1:50051 \
  --set data_dir=/path/to/persistent/voiceprint \
  --set threshold=0.25 \
  --set device=cpu
```

`rbnx start` executes the package manifest's start block. That block uses the
already-built virtual environment and generated modules; it does not run `uv`
or code generation at startup.

## Runtime configuration

Per-instance manifest configuration and matching environment variables are:

| Config key | Environment fallback | Default | Meaning |
| --- | --- | --- | --- |
| `data_dir` | `VOICEPRINT_DATA_DIR` | `services/voiceprint/rbnx-build/data` | Directory containing `enrolled.json`. |
| `threshold` | `VOICEPRINT_THRESHOLD` | `0.25` | Finite cosine similarity in `[0, 1]`; minimum score for a known-speaker result and duplicate-voice rejection. |
| `device` | `VOICEPRINT_DEVICE` | `cuda:0` when CUDA is available, otherwise `cpu` | Torch device used for ECAPA-TDNN inference. |

A value supplied in the manifest or with `rbnx start --set` takes precedence
over its environment fallback. Keep `data_dir` outside `rbnx-build` when the
enrollment database must survive clean package builds.

## Data and model storage

- The enrollment database is `<data_dir>/enrolled.json`. In-process reads and
  writes are guarded by a re-entrant lock. Enrollment performs duplicate checks,
  mutation, and persistence as one transaction; writes use a temporary file
  followed by `os.replace`, so readers do not observe a partially written JSON
  document.
- ModelScope keeps its downloaded snapshot in the user's ModelScope cache
  (normally below `~/.cache/modelscope`). SpeechBrain prepares its working copy
  under `rbnx-build/models/spkrec-ecapa-voxceleb`.
- `identify` and `enroll` are available only after lifecycle activation has
  loaded the model. Initialization retains configuration and the enrollment
  database while the provider is inactive; deactivation releases the model.
  Lifecycle failures leave requests rejected instead of starting a second,
  hidden model load from an RPC handler.

## Verification

After building, verify the generated modules with the package runtime:

```bash
cd /path/to/robonix/services/voiceprint
PYTHONPATH="$PWD/rbnx-build/codegen/proto_gen:$PWD/rbnx-build/codegen/robonix_mcp_types" \
  rbnx-build/venv/bin/python - <<'PY'
import robonix_contracts_pb2
import robonix_contracts_pb2_grpc
import voiceprint_mcp
import voiceprint_pb2
import voiceprint_service.service

print("Voiceprint generated modules: OK")
PY
```

After the stack is running, confirm that Atlas sees the four RPC contracts and
inspect the provider log:

```bash
rbnx caps -v | grep -E 'robonix/service/voiceprint/(driver|identify|enroll|list|delete)'
rbnx logs -t voiceprint
```

For a functional check, enroll a user through the supported client, capture a
second sample from the same speaker, and confirm that `identify` returns the
same `user_id` with `is_known=true`. Then delete that enrollment and confirm
that the same sample is no longer identified as known. The persisted database
can be inspected without changing it:

```bash
python3 -m json.tool /path/to/persistent/voiceprint/enrolled.json
```
