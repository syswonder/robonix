#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Voiceprint service build phase.
#
# Robonix package convention: every Python package gets a uv-managed venv
# under rbnx-build/. Build phase installs deps + downloads the CAM++
# speaker-embedding model so the runtime (`start:`) only activates the venv
# and serves — no downloads at request time.
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
#   rbnx-build/models/              CAM++ speaker-embedding weights
#   rbnx-build/data/                runtime state (enrolled.json)

set -euo pipefail

# Use TUNA mirror for pip / uv when on a domestic (CN) network. Override via env.
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

# The plain `sherpa-onnx` wheel does not bundle the native onnxruntime core
# library; install the complete, version-matched wheel set like the speech
# service build does. PyPI and the default TUNA mirror publish Linux
# x86_64/aarch64 wheels for all three packages.
SHERPA_ONNX_INDEX_URL="${SHERPA_ONNX_INDEX_URL:-$UV_INDEX_URL}"
uv pip install --python "$VENV/bin/python" \
    --index-url "$SHERPA_ONNX_INDEX_URL" \
    sherpa-onnx==1.13.4 sherpa-onnx-bin==1.13.4 sherpa-onnx-core==1.13.4

# ── 3. Codegen (.proto + grpc stubs + MCP dataclasses → rbnx-build/codegen/) ─
FLAGS=(--mcp)
# The complete build directory was already removed above. A second clean here
# would delete the freshly synchronized venv.
echo "[build] rbnx codegen ${FLAGS[*]}"
RBNX_CODEGEN_PYTHON="$PKG/$VENV/bin/python" \
    PATH="$PKG/$VENV/bin:$PATH" \
    rbnx codegen -p "$PKG" "${FLAGS[@]}"

CODEGEN_PYTHONPATH="$PKG/$BUILD/codegen/proto_gen:$PKG/$BUILD/codegen/robonix_mcp_types"
PYTHONPATH="$CODEGEN_PYTHONPATH:${PYTHONPATH:-}" "$VENV/bin/python" - <<'PY'
import robonix_contracts_pb2
import robonix_contracts_pb2_grpc
import voiceprint_mcp
import voiceprint_pb2

print("[build] generated Voiceprint gRPC and MCP imports OK")
PY

# ── 4. Pre-download the CAM++ speaker-embedding model (skip with SKIP_MODEL_DOWNLOAD=1) ─
# k2-fsa hosts the 3D-Speaker CAM++ zh-cn 16k ONNX weights on the
# sherpa-onnx "speaker-recongition-models" release tag (sic). ghfast.top
# mirrors GitHub for CN runners; fall back to the official URL.
SPEAKER_MODEL="$MODELS/campplus_zh.onnx"
SPEAKER_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/speaker-recongition-models/3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx"
if [[ "${SKIP_MODEL_DOWNLOAD:-}" != "1" ]]; then
    if [[ ! -f "$SPEAKER_MODEL" ]]; then
        echo "[build] downloading CAM++ speaker-embedding model"
        if ! curl -fL --retry 3 --connect-timeout 15 -o "$SPEAKER_MODEL" "https://ghfast.top/$SPEAKER_URL"; then
            rm -f "$SPEAKER_MODEL"
            curl -fL --retry 3 --connect-timeout 15 -o "$SPEAKER_MODEL" "$SPEAKER_URL"
        fi
    fi
    "$VENV/bin/python" - <<'PY'
import numpy as np
from voiceprint_service.engine import SpeakerEmbeddingEngine

engine = SpeakerEmbeddingEngine()
t = np.arange(8000, dtype=np.float32) / 8000.0
pcm = (np.sin(2 * np.pi * 440.0 * t) * 32000).astype(np.int16).tobytes()
embedding = engine.extract_from_pcm(pcm)
if embedding.shape != (192,) or not np.isfinite(embedding).all():
    raise RuntimeError(f"invalid CAM++ embedding: shape={embedding.shape}")
print(f"[build]   CAM++ ready; embedding shape={embedding.shape}")
PY
else
    echo "[build] SKIP_MODEL_DOWNLOAD=1 — skipping CAM++ download."
fi

echo "[build] done."
