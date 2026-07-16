# Memsearch memory service

Memsearch is the reference provider for `robonix/service/memory/*`. It indexes
Markdown files from a configured directory with `memsearch`'s ONNX embedding
backend and a Milvus Lite vector store. It is a regular Robonix service package,
not a system component.

## Capabilities

| Contract | Behavior |
| --- | --- |
| `robonix/service/memory/driver` | Runs lifecycle initialization and builds the initial corpus index before the provider becomes active. |
| `robonix/service/memory/search` | Searches the index and returns up to two relevant Markdown excerpts. |
| `robonix/service/memory/save` | Appends text to `<memory-dir>/YYYY-MM-DD_notes.md`, then refreshes the index. |
| `robonix/service/memory/compact` | Uses an OpenAI-compatible model to compact the indexed memory and returns the generated summary path. |

The IDL files are in
[`capabilities/lib/memory/`](../../capabilities/lib/memory/), and the contract
definitions are in
[`capabilities/service/memory/`](../../capabilities/service/memory/).

The three memory operations are exposed through MCP. `robonix-api` serves the
lifecycle contract and passes the package instance's `config:` mapping to
`CMD_INIT`. The provider becomes active only after its backend is constructed
and the initial index succeeds.

## Data and model files

Memsearch uses three kinds of local state:

- `memory_dir` contains the Markdown corpus. The default is `./agent_memory`,
  resolved from the package's runtime working directory.
- `milvus_uri` selects the Milvus store. The default is `./agent_milvus.db`,
  also resolved from the runtime working directory. Remote endpoints and URIs
  are passed to the backend without path resolution.
- The ONNX embedding model (`gpahal/bge-m3-onnx-int8`) is downloaded into the
  standard Hugging Face cache during `rbnx build`, not on the first request.

Set absolute paths for the corpus and Milvus database when their contents must
survive deployment-cache replacement. The package also writes redirected
process output to `rbnx-build/data/memsearch.log`; lifecycle and service records
remain available through Scribe and `rbnx logs` during a deployment.

## Runtime configuration

The accepted instance mapping is documented in [`config.spec`](config.spec).
Values in `config:` take priority over their environment compatibility
fallbacks.

| Config key | Environment fallback | Default | Purpose |
| --- | --- | --- | --- |
| `memory_dir` | `AGENT_MEMORY_DIR` | `./agent_memory` | Directory containing indexed Markdown files and newly saved notes. |
| `milvus_uri` | `AGENT_MILVUS_URI` | `./agent_milvus.db` | Milvus Lite database path or a remote URI accepted by `memsearch`. |
| `onnx_threads` | `MEMSEARCH_ONNX_THREADS` | `1` | ONNX intra-op thread count on aarch64; must be at least 1. |

Logging and model credentials remain environment variables because they are
needed before lifecycle initialization or contain secrets:

| Variable | Default | Purpose |
| --- | --- | --- |
| `MEMSEARCH_LOG_LEVEL` | `INFO` | Python log level forwarded to Scribe. |
| `VLM_BASE_URL` | unset | Preferred OpenAI-compatible endpoint used by `compact`. |
| `VLM_API_KEY` | unset | Preferred API key used by `compact`; required for compaction. |
| `VLM_MODEL` | unset | Preferred model used by `compact`. |
| `OPENAI_BASE_URL` | unset | Fallback endpoint when `VLM_BASE_URL` is unset. |
| `OPENAI_API_KEY` | unset | Fallback key when `VLM_API_KEY` is unset. |
| `OPENAI_MODEL` | unset | Fallback model when `VLM_MODEL` is unset. If both are unset, `compact` uses `gpt-5.5`. |
| `ROBONIX_ATLAS` | supplied by `rbnx` | Atlas endpoint used by `robonix-api`. |

`search` and `save` do not require model API credentials. When neither
`VLM_API_KEY` nor `OPENAI_API_KEY` is set, `compact` returns a clear
no-credentials result without calling an external endpoint.

A deployment entry can therefore remain minimal:

```yaml
env:
  VLM_BASE_URL: ${VLM_BASE_URL}
  VLM_API_KEY: ${VLM_API_KEY}
  VLM_MODEL: ${VLM_MODEL}

service:
  - name: memory
    path: ${ROBONIX_SOURCE_PATH}/services/memsearch
    config:
      memory_dir: /home/robot/.robonix/memory
      milvus_uri: /home/robot/.robonix/memory/milvus.db
      onnx_threads: 1
```

## Build

From a Robonix source checkout that has already been registered with
`rbnx setup`:

```bash
rbnx validate services/memsearch
rbnx build -p services/memsearch
```

The build script creates `rbnx-build/venv`, installs the locked Python
dependencies, generates the MCP and protobuf modules, and warms the ONNX model.
The warm-up is attempted up to three times and the build fails if the model
cannot be loaded.

The build script reads the following variables:

| Variable | Default | Purpose |
| --- | --- | --- |
| `RBNX_BUILD_CLEAN` | unset | Set to `1` to remove `rbnx-build` before rebuilding. |
| `RBNX_PACKAGE_ROOT` | detected from the script | Explicit package root used by the build script. |
| `UV_INDEX_URL` | Tsinghua PyPI mirror | Python package index used by `uv`. |
| `PIP_INDEX_URL` | Tsinghua PyPI mirror | Package index inherited by Python packaging tools. |
| `HF_ENDPOINT` | `https://huggingface.co` | Hugging Face Hub endpoint used to warm the embedding model. |

An `HF_ENDPOINT` override must implement the Hugging Face Hub metadata and file
download APIs; a mirror that only redirects model files may still fail during
repository metadata lookup.

## Start

In a robot deployment, `rbnx boot` builds the package when needed and executes
the `start:` block from `package_manifest.yaml`. To start only this package,
first make sure Atlas is running, then run:

```bash
rbnx start -p services/memsearch --endpoint 127.0.0.1:50051
```

The package entry point is
`rbnx-build/venv/bin/python -m memsearch_service.service`. Running it directly
requires the generated protobuf and MCP directories on `PYTHONPATH`; use
`rbnx start` unless debugging the package startup script itself.

## Verify

Run the lightweight compatibility tests after a build:

```bash
cd services/memsearch
PYTHONPATH=. rbnx-build/venv/bin/python -m unittest discover -s tests -v
```

With the deployment running, verify the provider and its three contracts:

```bash
rbnx caps -v | rg 'robonix/service/memory/(driver|search|save|compact)'
rbnx logs -t memory
```

Startup logs report the resolved corpus path, Milvus URI, Python environment,
and each initialization phase. If model warm-up fails during the build, check
network access and `HF_ENDPOINT`. If runtime startup reports that Milvus cannot
open the local database, verify that the database parent directory is writable
and stop the previous deployment through `rbnx shutdown` before retrying. The
package never terminates an arbitrary process merely because it has the database
open.
