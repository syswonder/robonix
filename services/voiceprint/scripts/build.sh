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

# This venv's site-packages (robust to the python minor version).
SITEPKG="$("$VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))' 2>/dev/null || echo "$VENV/lib/python3.10/site-packages")"

# Jetson rebuild safety: a prior build (step 2b) may have left host-torch
# SYMLINKS in this venv. Remove them BEFORE uv sync — otherwise uv, seeing the
# deleted dist-info, reinstalls torch and writes THROUGH the symlink into the
# host's shared JetPack torch tree, corrupting the system torch. Re-linked below.
if [[ -f /etc/nv_tegra_release ]]; then
    for _m in torch torchaudio torchvision torchgen functorch torio; do
        [[ -L "$SITEPKG/$_m" ]] && rm -f "$SITEPKG/$_m"
    done
fi

# ── 2. uv sync (deps from pyproject.toml + workspace uv.lock) ──────────────
echo "[build] uv sync (pyproject.toml → $VENV)"
VIRTUAL_ENV="$PKG/$VENV" uv sync --active --no-managed-python

# ── 2b. Jetson: use the host's JetPack CUDA torch, not PyPI's ──────────────
# On Jetson (aarch64), PyPI's torch is built against a CUDA version that does
# NOT match the JetPack driver, so torch.cuda.is_available() is False and the
# ECAPA-TDNN speaker model runs on CPU (slow enrol/identify). JetPack ships a
# working CUDA torch on the host python; reuse it by symlinking the torch
# family into this venv (other deps stay from uv). Resolve each module's dir
# from the host python so it works regardless of install location.
# Disable with VOICEPRINT_SKIP_JETSON_TORCH=1.
if [[ -f /etc/nv_tegra_release && "${VOICEPRINT_SKIP_JETSON_TORCH:-}" != "1" ]]; then
    SP="$SITEPKG"
    if ! "$VENV/bin/python" -c 'import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)' 2>/dev/null; then
        HOSTPY="${VOICEPRINT_HOST_PYTHON:-python3}"
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
            echo "[build] WARNING: on Jetson but host python has no CUDA torch; running on CPU (slow)." >&2
        fi
    fi
fi

# ── 3. Pre-warm ECAPA-TDNN weights from ModelScope (skip with SKIP_MODEL_DOWNLOAD=1) ─
# Fetch from ModelScope, not HuggingFace: hf_hub's metadata HEAD only follows
# same-host redirects, and the reachable hf-mirror.com bounces resolve/main to
# huggingface.co with a cross-host 308 that no hf_hub version follows, so every
# HF path fails on CN networks. ModelScope's SDK has no such issue (it's also
# where the speech service pulls FunASR). This populates ~/.cache/modelscope so
# the service loads offline at startup; engine.py resolves the same repo.
PY="$VENV/bin/python"
if [[ "${SKIP_MODEL_DOWNLOAD:-}" != "1" ]]; then
    echo "[build] pre-warming ECAPA-TDNN weights from ModelScope → ~/.cache/modelscope"
    "$PY" -c "
from modelscope.hub.snapshot_download import snapshot_download
d = snapshot_download('speechbrain/spkrec-ecapa-voxceleb')
print('[build] ModelScope snapshot:', d)
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
