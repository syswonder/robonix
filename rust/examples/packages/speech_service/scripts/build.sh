#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

# Step 1: rbnx codegen (generates proto stubs from ROS IDL + contracts)
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# Step 2: Download model weights (skip in CI mode or if SKIP_MODEL_DOWNLOAD=1)
if [[ "${SPEECH_CI_MODE:-}" != "1" && "${SKIP_MODEL_DOWNLOAD:-}" != "1" ]]; then
    echo "[build] Downloading ASR model (openai/whisper-large-v3)..."
    python3 -c "from transformers import pipeline; pipeline('automatic-speech-recognition', model='openai/whisper-large-v3', model_kwargs={'local_files_only': False})" || echo "[build] WARNING: Whisper model download failed. Service will fail to start ASR backend."

    echo "[build] Downloading FunASR model (paraformer-zh-streaming)..."
    python3 -c "from funasr import AutoModel; AutoModel(model='paraformer-zh-streaming')" || echo "[build] WARNING: FunASR model download failed. Service will fail to start streaming ASR backend."
else
    echo "[build] Skipping model download (CI mode or SKIP_MODEL_DOWNLOAD=1)."
fi

echo "[build] done."
