#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Voiceprint service build phase.
#
# Robonix package convention: every Python package gets a uv-managed venv
# under rbnx-build/. Build phase installs deps + pre-warms the ECAPA-TDNN
# model so the runtime (`start:`) only activates the venv and serves —
# no downloads at request time.
#
# gRPC stubs are generated below by the `rbnx codegen -p <pkg> --mcp`
# step, which materialises atlas-managed contract bundles into
# rbnx-build/codegen/proto_gen/. `rbnx build` itself only runs this
# manifest's `build:` body — packages call codegen explicitly here,
# same as services/memsearch and services/speech.
# The previous version of this script ran grpc_tools.protoc against a
# bespoke `proto/voiceprint.proto` — that bespoke proto has been
# removed in favour of the standard capabilities/*.toml flow.
#
# Layout under rbnx-build/:
#   rbnx-build/venv/                per-package Python venv (uv)
#   rbnx-build/codegen/proto_gen/   atlas-managed stubs (created by `rbnx codegen`)
#   rbnx-build/models/              ECAPA-TDNN weights
#   rbnx-build/data/                runtime state (enrolled.json)

set -euo pipefail

# Use TUNA mirror for pip / uv when GFW-bound. Override via env.
: "${UV_INDEX_URL:=https://pypi.tuna.tsinghua.edu.cn/simple}"
: "${PIP_INDEX_URL:=https://pypi.tuna.tsinghua.edu.cn/simple}"
export UV_INDEX_URL PIP_INDEX_URL

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
VENV="$BUILD/venv"
MODELS="$BUILD/models"
DATA="$BUILD/data"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$MODELS" "$DATA"

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

# ── 3. Pre-download ECAPA-TDNN weights (skip with SKIP_MODEL_DOWNLOAD=1) ───
PY="$VENV/bin/python"
: "${HF_ENDPOINT:=https://hf-mirror.com}"
export HF_ENDPOINT
if [[ "${SKIP_MODEL_DOWNLOAD:-}" != "1" ]]; then
    echo "[build] downloading ECAPA-TDNN weights → $MODELS"
    "$PY" -c "
from speechbrain.inference.speaker import SpeakerRecognition
SpeakerRecognition.from_hparams(
    source='speechbrain/spkrec-ecapa-voxceleb',
    savedir='$MODELS/spkrec-ecapa-voxceleb',
    run_opts={'device': 'cpu'},
)
" || echo "[build] WARNING: ECAPA-TDNN download failed; service will retry at startup."
else
    echo "[build] SKIP_MODEL_DOWNLOAD=1 — skipping ECAPA-TDNN download."
fi

# ── 4. Codegen (.proto + grpc stubs + MCP dataclasses → rbnx-build/codegen/) ─
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

echo "[build] done."
