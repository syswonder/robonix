#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Speech service build phase.
#
# Robonix package convention: every Python package gets a uv-managed venv
# under rbnx-build/. Build phase installs deps + runs codegen + warms
# every model the runtime needs (ASR Whisper, ASR FunASR streaming, TTS).
# Runtime (`start:`) only activates the venv and serves; no model
# downloads at runtime.
#
# Layout under rbnx-build/:
#   rbnx-build/venv/                   per-package Python venv (uv)
#   rbnx-build/codegen/proto_gen/      ROS IDL → .proto + grpc stubs
#   rbnx-build/codegen/robonix_mcp_types/  MCP dataclasses
#   rbnx-build/ws/install/setup.bash   rbnx-cli's PYTHONPATH stub (existing)

set -euo pipefail
# Use TUNA mirror for pip / uv when GFW-bound. Override via env.
: "${UV_INDEX_URL:=https://pypi.tuna.tsinghua.edu.cn/simple}"
: "${PIP_INDEX_URL:=https://pypi.tuna.tsinghua.edu.cn/simple}"
export UV_INDEX_URL PIP_INDEX_URL
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
VENV="$BUILD/venv"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. uv venv ──────────────────────────────────────────────────────────────
if ! command -v uv >/dev/null 2>&1; then
    echo "[build] error: 'uv' not found on PATH. Install: https://docs.astral.sh/uv/" >&2
    exit 1
fi
if [[ ! -d "$VENV" ]]; then
    echo "[build] uv venv → $VENV"
    uv venv "$VENV"
fi

# ── 2. uv sync (deps from pyproject.toml + workspace uv.lock) ──────────────
echo "[build] uv sync (pyproject.toml → $VENV)"
VIRTUAL_ENV="$PKG/$VENV" uv sync --active --no-managed-python

# ── 3. Codegen (.proto + grpc stubs → rbnx-build/codegen/) ──────────────────
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# ── 4. Pre-download models (skip in CI / SKIP_MODEL_DOWNLOAD=1) ─────────────
# Default HF_ENDPOINT to hf-mirror.com — direct huggingface.co is essentially
# unreachable from CN networks (was failing with 0% throughput on partial
# 1.6GB cache). Override by exporting HF_ENDPOINT before invoking build.sh.
: "${HF_ENDPOINT:=https://hf-mirror.com}"
export HF_ENDPOINT
if [[ "${SPEECH_CI_MODE:-}" != "1" && "${SKIP_MODEL_DOWNLOAD:-}" != "1" ]]; then
    PY="$VENV/bin/python"
    # Whisper is opt-in: it's a 20+ GB pull (whisper-large-v3) that only
    # the one-shot `robonix/system/speech/asr` contract uses. The default
    # streaming path (FunASR paraformer-zh-streaming, ~500 MB) is what
    # the dialog stack actually invokes. Override with DOWNLOAD_WHISPER=1
    # if a caller for the one-shot ASR contract is in the picture.
    if [[ "${DOWNLOAD_WHISPER:-}" == "1" ]]; then
        echo "[build] downloading ASR model (openai/whisper-large-v3)…"
        # `local_files_only=False` is the pipeline default; passing it via
        # model_kwargs collides with the explicit kwarg in newer
        # transformers (TypeError: got multiple values). `snapshot_download`
        # is more direct for "just fetch the weights into the HF cache"
        # intent and avoids spinning up the full pipeline.
        "$PY" -c "from huggingface_hub import snapshot_download; snapshot_download('openai/whisper-large-v3')" \
            || echo "[build] WARNING: Whisper model download failed; ASR backend will fail at runtime."
    else
        echo "[build] skipping Whisper download (set DOWNLOAD_WHISPER=1 to pull the 20 GB one-shot ASR weights)."
    fi
    echo "[build] downloading FunASR model (paraformer-zh-streaming)…"
    "$PY" -c "from funasr import AutoModel; AutoModel(model='paraformer-zh-streaming')" \
        || echo "[build] WARNING: FunASR model download failed; streaming ASR backend will fail at runtime."
else
    echo "[build] skipping model download (CI mode or SKIP_MODEL_DOWNLOAD=1)."
fi

echo "[build] done."
