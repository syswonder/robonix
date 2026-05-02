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
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
VENV="$BUILD/venv"
GEN="$BUILD/codegen"
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
FLAGS=(--out-dir "$GEN")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# ── 4. Pre-download models (skip in CI / SKIP_MODEL_DOWNLOAD=1) ─────────────
if [[ "${SPEECH_CI_MODE:-}" != "1" && "${SKIP_MODEL_DOWNLOAD:-}" != "1" ]]; then
    PY="$VENV/bin/python"
    echo "[build] downloading ASR model (openai/whisper-large-v3)…"
    "$PY" -c "from transformers import pipeline; pipeline('automatic-speech-recognition', model='openai/whisper-large-v3', model_kwargs={'local_files_only': False})" \
        || echo "[build] WARNING: Whisper model download failed; ASR backend will fail at runtime."
    echo "[build] downloading FunASR model (paraformer-zh-streaming)…"
    "$PY" -c "from funasr import AutoModel; AutoModel(model='paraformer-zh-streaming')" \
        || echo "[build] WARNING: FunASR model download failed; streaming ASR backend will fail at runtime."
else
    echo "[build] skipping model download (CI mode or SKIP_MODEL_DOWNLOAD=1)."
fi

echo "[build] done."
