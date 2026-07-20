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
# Use TUNA mirror for pip / uv when on a domestic (CN) network. Override via env.
: "${UV_INDEX_URL:=https://pypi.tuna.tsinghua.edu.cn/simple}"
: "${PIP_INDEX_URL:=https://pypi.tuna.tsinghua.edu.cn/simple}"
export UV_INDEX_URL PIP_INDEX_URL
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
VENV="$BUILD/venv"
CLEAN="${RBNX_BUILD_CLEAN:-}"
IS_JETSON=0
[[ -f /etc/nv_tegra_release ]] && IS_JETSON=1
BACKEND="${SPEECH_BACKEND:-local}"
case "$BACKEND" in
    local|tencent|custom|mock) ;;
    *)
        echo "[build] error: unsupported SPEECH_BACKEND=$BACKEND" >&2
        exit 1
        ;;
esac
USE_JETSON_TORCH=0
if [[ "$IS_JETSON" == "1" && "$BACKEND" == "local" ]]; then
    USE_JETSON_TORCH=1
fi
echo "[build] speech backend: $BACKEND"

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
if [[ -d "$VENV" ]]; then
    has_system_site=0
    grep -q '^include-system-site-packages = true$' "$VENV/pyvenv.cfg" \
        && has_system_site=1
    if [[ "$USE_JETSON_TORCH" == "1" && "$has_system_site" == "0" ]]; then
        echo "[build] Jetson local backend: recreating venv with host CUDA packages"
        rm -rf "$VENV"
    elif [[ "$USE_JETSON_TORCH" == "0" && "$has_system_site" == "1" ]]; then
        echo "[build] $BACKEND backend: recreating isolated venv"
        rm -rf "$VENV"
    fi
fi
if [[ ! -d "$VENV" ]]; then
    echo "[build] uv venv → $VENV"
    if [[ "$USE_JETSON_TORCH" == "1" ]]; then
        uv venv --system-site-packages "$VENV"
    else
        uv venv "$VENV"
    fi
fi

# This venv's site-packages (robust to the python minor version).
SITEPKG="$("$VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))' 2>/dev/null || echo "$VENV/lib/python3.10/site-packages")"

# Jetson rebuild safety: a prior build (step 2b) may have left host-torch
# SYMLINKS in this venv. Remove them BEFORE uv sync — otherwise uv, seeing the
# deleted dist-info, reinstalls torch and writes THROUGH the symlink into the
# host's shared JetPack torch tree, corrupting the system torch. Re-linked below.
if [[ "$USE_JETSON_TORCH" == "1" ]]; then
    for _m in torch torchaudio torchvision torchgen functorch torio; do
        [[ -L "$SITEPKG/$_m" ]] && rm -f "$SITEPKG/$_m"
    done
fi

# ── 2. uv sync (deps from pyproject.toml + workspace uv.lock) ──────────────
echo "[build] uv sync (pyproject.toml → $VENV)"
SYNC_ARGS=(--active --no-managed-python)
if [[ "$BACKEND" == "local" ]]; then
    SYNC_ARGS+=(--extra local)
fi
if [[ "$USE_JETSON_TORCH" == "1" ]]; then
    SYNC_ARGS+=(
        --no-install-package torch
        --no-install-package torchaudio
        --no-install-package torchvision
    )
fi
VIRTUAL_ENV="$PKG/$VENV" uv sync "${SYNC_ARGS[@]}"

# Wake-word detection is a core Speech capability, independent of whether ASR
# and TTS use local models or Tencent. Install the complete, version-matched
# sherpa-onnx wheel set from the configured Python package index. PyPI and the
# default TUNA mirror publish Linux x86_64/aarch64 wheels for all three
# packages, so the build does not need to scrape k2-fsa.github.io (which is not
# reliably reachable from mainland China).
SHERPA_ONNX_INDEX_URL="${SHERPA_ONNX_INDEX_URL:-$UV_INDEX_URL}"
uv pip install --python "$VENV/bin/python" \
    --index-url "$SHERPA_ONNX_INDEX_URL" \
    sherpa-onnx==1.13.4 sherpa-onnx-bin==1.13.4 sherpa-onnx-core==1.13.4

# ── 2b. Jetson: use the host's JetPack CUDA torch, not PyPI's ──────────────
# On Jetson (aarch64), PyPI ships a torch built against a CUDA version that
# does NOT match the JetPack driver (e.g. a cu130 wheel vs the CUDA 12.6
# driver), so torch.cuda.is_available() is False and FunASR/Whisper silently
# fall back to CPU — streaming ASR then emits each character with a long lag.
# JetPack already provides a working CUDA torch on the host python; reuse it
# by symlinking the torch family into this venv (all other deps stay from uv).
# Generalised: resolve each module's dir from the host python, so it works
# regardless of where JetPack / editable installs place them.
# Disable with SPEECH_SKIP_JETSON_TORCH=1.
if [[ "$USE_JETSON_TORCH" == "1" && "${SPEECH_SKIP_JETSON_TORCH:-}" != "1" ]]; then
    SP="$SITEPKG"
    if ! "$VENV/bin/python" -c 'import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)' 2>/dev/null; then
        HOSTPY="${SPEECH_HOST_PYTHON:-python3}"
        if "$HOSTPY" -c 'import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)' 2>/dev/null; then
            echo "[build] Jetson: venv torch is CPU-only — linking host JetPack CUDA torch"
            for mod in torch torchaudio torchvision torchgen functorch torio; do
                d="$("$HOSTPY" -c "import os,importlib.util as u; s=u.find_spec('$mod'); print(os.path.dirname(s.origin) if s and s.origin else '')" 2>/dev/null || true)"
                [[ -n "$d" && -d "$d" ]] || continue
                rm -rf "$SP/$mod" "$SP/$mod"-*.dist-info
                ln -sfn "$d" "$SP/$mod"
                echo "[build]   linked $mod ← $d"
            done
            "$VENV/bin/python" -c 'import torch; print("[build]   venv torch.cuda.is_available() =", torch.cuda.is_available())' || true
        else
            echo "[build] WARNING: on Jetson but host python has no CUDA torch; ASR will run on CPU (slow)." >&2
        fi
    fi
fi

# ── 3. Codegen (.proto + grpc stubs → rbnx-build/codegen/) ──────────────────
echo "[build] rbnx codegen"
PATH="$PKG/$VENV/bin:$PATH" rbnx codegen --mcp -p "$PKG"

# The service imports both gRPC stubs (`speech_pb2*`) and MCP dataclasses
# (`speech_mcp`) at startup. Verify both generated Python roots now so a
# missing runtime PYTHONPATH cannot degrade into a 60-second Atlas
# registration timeout during `rbnx boot`.
CODEGEN_PYTHONPATH="$PKG/$BUILD/codegen/proto_gen:$PKG/$BUILD/codegen/robonix_mcp_types"
PYTHONPATH="$CODEGEN_PYTHONPATH:${PYTHONPATH:-}" "$VENV/bin/python" - <<'PY'
import speech_mcp
import speech_pb2
import speech_pb2_grpc

print("[build] generated Speech gRPC and MCP imports OK")
PY

# ── 4. Pre-download models (skip with SKIP_MODEL_DOWNLOAD=1) ────────────────
# Default HF_ENDPOINT to hf-mirror.com for runners where direct model downloads
# are slow or unreliable. Override by exporting HF_ENDPOINT before invoking build.sh.
: "${HF_ENDPOINT:=https://hf-mirror.com}"
export HF_ENDPOINT

if [[ "${SKIP_MODEL_DOWNLOAD:-}" != "1" ]]; then
    PY="$VENV/bin/python"
    MODEL_NAME=sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20
    MODEL_DIR="$BUILD/models/$MODEL_NAME"
    ARCHIVE="$BUILD/models/$MODEL_NAME.tar.bz2"
    OFFICIAL_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/$MODEL_NAME.tar.bz2"
    MIRROR_URL="https://ghfast.top/$OFFICIAL_URL"
    mkdir -p "$BUILD/models"
    if [[ ! -f "$MODEL_DIR/tokens.txt" ]]; then
        echo "[build] downloading Speech wake-word model ($MODEL_NAME)"
        rm -f "$ARCHIVE"
        if ! curl -fL --retry 3 --connect-timeout 15 -o "$ARCHIVE" \
            "${SHERPA_KWS_MODEL_URL:-$MIRROR_URL}"; then
            rm -f "$ARCHIVE"
            curl -fL --retry 3 --connect-timeout 15 -o "$ARCHIVE" "$OFFICIAL_URL"
        fi
        bzip2 -t "$ARCHIVE"
        tar -xjf "$ARCHIVE" -C "$BUILD/models"
        rm -f "$ARCHIVE"
    fi
    "$PY" -c 'import sherpa_onnx; print("[build] sherpa-onnx", sherpa_onnx.__version__)'
else
    echo "[build] skipping wake-word model download (SKIP_MODEL_DOWNLOAD=1)."
fi

if [[ "$BACKEND" == "local" && "${SKIP_MODEL_DOWNLOAD:-}" != "1" ]]; then
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
        "$PY" -c "from huggingface_hub import snapshot_download; snapshot_download('openai/whisper-large-v3')"
    else
        echo "[build] skipping Whisper download (set DOWNLOAD_WHISPER=1 to pull the 20 GB one-shot ASR weights)."
    fi
    echo "[build] downloading FunASR model (paraformer-zh-streaming)…"
    "$PY" -c "from funasr import AutoModel; AutoModel(model='paraformer-zh-streaming')"
elif [[ "$BACKEND" == "tencent" ]]; then
    echo "[build] Tencent backend: validating lightweight cloud dependencies"
    "$VENV/bin/python" - <<'PY'
from speech_service.tencent_cloud import TencentRealtimeASRBackend, TencentTTSBackend
import requests
from websockets.sync.client import connect

assert TencentRealtimeASRBackend and TencentTTSBackend and requests and connect
print("[build]   Tencent ASR/TTS imports OK; no local model or GPU required")
PY
elif [[ "$BACKEND" == "local" ]]; then
    echo "[build] skipping model download (SKIP_MODEL_DOWNLOAD=1)."
else
    echo "[build] $BACKEND backend: no bundled model download"
fi

echo "[build] done."
