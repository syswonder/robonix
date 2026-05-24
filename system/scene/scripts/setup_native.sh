#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# One-shot installer / verifier for the scene service in NATIVE mode.
#
# Mirrors what docker/Dockerfile bakes into the `robonix-scene` image,
# but on the host. Idempotent: safe to re-run; existing weights / pip
# packages are reused.
#
# What this does:
#
#   1. Verify ROS 2 Humble + python3-cv-bridge + tf2_ros are installed.
#      Print apt commands if anything is missing; do NOT auto-`apt`
#      (sudo prompts inside `rbnx boot` would deadlock the deploy).
#   2. Create / reuse a Python venv at ${SCENE_NATIVE_VENV:-rbnx-build/venv}.
#      Install torch + torchvision (cu124 for x86 / Jetson; CPU build
#      via ROBONIX_FORCE_CPU=1) plus everything in docker/requirements.txt.
#   3. Install concept-graphs (ali-dev branch) into the venv with
#      `pip install --no-deps -e`. Cloned to ${SCENE_CG_DIR:-/opt/concept-graphs};
#      override that path if the operator wants it elsewhere.
#   4. Pre-fetch CV weights into docker/_weights/ (build.sh's
#      pre_fetch_weights logic, replicated here so native users don't
#      need to run docker build).
#   5. Pre-warm CLIP caches: openai `clip` ViT-B/32 (used by
#      YOLO-World's text encoder) and open_clip ViT-B-32 LAION-2B
#      (used for per-detection visual similarity in concept-graphs).
#
# Env knobs:
#   SCENE_NATIVE_VENV          venv path to create / reuse (default: rbnx-build/venv)
#   SCENE_CG_DIR               concept-graphs source clone path (default: /opt/concept-graphs)
#   SCENE_TORCH_INDEX          pip --find-links / --index-url for torch
#                              (default: aliyun pytorch-wheels cu124, fallback
#                              to download.pytorch.org/whl/cu124)
#   ROBONIX_FORCE_CPU=1        install CPU-only torch (no cu124 wheels)
#   RBNX_GH_MIRROR             github prefix for weight downloads (default: ghfast.top mirror)
#   PIP_INDEX_URL              respected (TUNA recommended in CN)
#   SKIP_VENV=1                use the current Python env directly (no venv creation)

set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

VENV_DIR="${SCENE_NATIVE_VENV:-${PKG}/rbnx-build/venv}"
CG_DIR="${SCENE_CG_DIR:-/opt/concept-graphs}"
WEIGHTS_DIR="${PKG}/docker/_weights"
GH_MIRROR="${RBNX_GH_MIRROR-https://ghfast.top/}"

echo "[scene/setup-native] package root  : ${PKG}"
echo "[scene/setup-native] target venv   : ${VENV_DIR}"
echo "[scene/setup-native] concept-graphs: ${CG_DIR}"

# ── 1. ROS 2 system check ──────────────────────────────────────────────
need_apt=()
if [[ -z "${ROS_DISTRO:-}" || -z "${AMENT_PREFIX_PATH:-}" ]] || ! command -v ros2 >/dev/null 2>&1; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        # shellcheck disable=SC1091
        set +u; source /opt/ros/humble/setup.bash; set -u
    else
        echo "[scene/setup-native] ROS 2 Humble not found at /opt/ros/humble." >&2
        echo "[scene/setup-native] sudo apt install ros-humble-ros-base" >&2
        exit 2
    fi
fi
for pkg_apt in python3-cv-bridge ros-humble-tf2-ros ros-humble-tf-transformations; do
    if ! dpkg -s "$pkg_apt" >/dev/null 2>&1; then
        need_apt+=("$pkg_apt")
    fi
done
if (( ${#need_apt[@]} > 0 )); then
    echo "[scene/setup-native] missing apt packages: ${need_apt[*]}" >&2
    echo "[scene/setup-native] run:  sudo apt install ${need_apt[*]}" >&2
    exit 2
fi

# ── 2. Venv ────────────────────────────────────────────────────────────
if [[ "${SKIP_VENV:-0}" == "1" ]]; then
    PY="python3"
    PIP="pip"
    echo "[scene/setup-native] SKIP_VENV=1 — using current Python env"
else
    if [[ ! -d "$VENV_DIR" ]]; then
        echo "[scene/setup-native] creating venv at ${VENV_DIR}"
        # `--system-site-packages` so rclpy + cv_bridge from apt are
        # picked up automatically (they live under
        # /opt/ros/humble/lib/python*/site-packages and are sourced
        # via AMENT_PREFIX_PATH, but having site-packages visible to
        # the venv keeps `import rclpy` working when ROS isn't
        # currently sourced).
        python3 -m venv --system-site-packages "$VENV_DIR"
    else
        echo "[scene/setup-native] reusing venv at ${VENV_DIR}"
    fi
    PY="${VENV_DIR}/bin/python"
    PIP="${VENV_DIR}/bin/pip"
fi

"$PIP" install --upgrade pip wheel setuptools

# Torch: cu124 by default (works on x86 sm_120 Blackwell + Jetson Orin's
# r36 image). Operator can flip ROBONIX_FORCE_CPU=1 for headless / CI.
if [[ "${ROBONIX_FORCE_CPU:-0}" == "1" ]]; then
    echo "[scene/setup-native] ROBONIX_FORCE_CPU=1 — installing CPU torch"
    "$PIP" install torch==2.5.1 torchvision==0.20.1 \
        --index-url https://download.pytorch.org/whl/cpu \
        || "$PIP" install torch==2.5.1 torchvision==0.20.1
else
    TORCH_INDEX="${SCENE_TORCH_INDEX:-https://mirrors.aliyun.com/pytorch-wheels/cu124/}"
    echo "[scene/setup-native] installing torch 2.5.1 + torchvision 0.20.1 from ${TORCH_INDEX}"
    "$PIP" install --find-links "$TORCH_INDEX" torch==2.5.1 torchvision==0.20.1 \
        || "$PIP" install --index-url https://download.pytorch.org/whl/cu124 \
            torch==2.5.1 torchvision==0.20.1
fi

# requirements.txt: same file the docker image installs from. Note
# the `pytorch3d` line is x86_64-only via PEP 508 marker; Jetson
# (aarch64) wheel index has no pytorch3d wheel, and source build
# would need nvcc — we accept that pytorch3d is missing on Jetson
# (concept-graphs slam.utils + slam.mapping don't import it; see
# requirements.txt comment for the exact reasoning).
echo "[scene/setup-native] installing scene Python deps from docker/requirements.txt"
"$PIP" install -r "${PKG}/docker/requirements.txt"

# ── 3. concept-graphs (ali-dev branch) ─────────────────────────────────
if [[ ! -d "$CG_DIR/.git" ]]; then
    if [[ -e "$CG_DIR" ]]; then
        echo "[scene/setup-native] ERR: ${CG_DIR} exists but isn't a git repo; remove it manually." >&2
        exit 2
    fi
    echo "[scene/setup-native] cloning concept-graphs into ${CG_DIR}"
    parent="$(dirname "$CG_DIR")"
    if [[ ! -w "$parent" ]]; then
        echo "[scene/setup-native] ${parent} is not writable for $USER." >&2
        echo "[scene/setup-native] either chown ${parent} or set SCENE_CG_DIR to somewhere you own." >&2
        exit 2
    fi
    if [[ -n "$GH_MIRROR" ]]; then
        if ! git clone --depth 1 --branch ali-dev \
                "${GH_MIRROR%/}/https://github.com/concept-graphs/concept-graphs.git" \
                "$CG_DIR" 2>/dev/null; then
            echo "[scene/setup-native] mirror clone failed; trying direct"
            git clone --depth 1 --branch ali-dev \
                https://github.com/concept-graphs/concept-graphs.git "$CG_DIR"
        fi
    else
        git clone --depth 1 --branch ali-dev \
            https://github.com/concept-graphs/concept-graphs.git "$CG_DIR"
    fi
else
    echo "[scene/setup-native] concept-graphs repo already at ${CG_DIR} — pulling"
    ( cd "$CG_DIR" && git pull --ff-only origin ali-dev || true )
fi
"$PIP" install --no-deps -e "$CG_DIR"

# ── 4. CV model weights (yolov8l-world.pt + mobile_sam.pt) ─────────────
mkdir -p "$WEIGHTS_DIR"
fetch_weight() {
    local url="$1" dest="$2"
    if [[ -s "$dest" ]]; then
        echo "[scene/setup-native] weight already present: $(basename "$dest")"
        return 0
    fi
    local primary="$url"
    [[ -n "$GH_MIRROR" ]] && primary="${GH_MIRROR%/}/$url"
    echo "[scene/setup-native] downloading $(basename "$dest") from $primary"
    if curl -fL --connect-timeout 30 --retry 5 --retry-all-errors --retry-delay 5 \
            -o "$dest" "$primary"; then
        return 0
    fi
    rm -f "$dest"
    if [[ "$primary" != "$url" ]]; then
        echo "[scene/setup-native] mirror failed; falling back to direct: $url" >&2
        curl -fL --connect-timeout 30 --retry 5 --retry-all-errors --retry-delay 5 \
            -o "$dest" "$url"
    else
        return 1
    fi
}
fetch_weight \
    "https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8l-world.pt" \
    "$WEIGHTS_DIR/yolov8l-world.pt"
fetch_weight \
    "https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt" \
    "$WEIGHTS_DIR/mobile_sam.pt"

# ── 5. CLIP cache pre-warm ─────────────────────────────────────────────
# Same two downloads the Dockerfile RUNs at build time. Skipping these
# means first inference will reach huggingface / openai's CDN for
# ~500 MB of weights — robonix invariant violation on customer
# hardware. Honour HF_ENDPOINT (CN: hf-mirror.com) if set in env.
echo "[scene/setup-native] pre-warming CLIP caches (first run downloads ~200 MB)"
"$PY" - <<'PY'
import open_clip, clip, os
print(f"[scene/setup-native] HF_ENDPOINT={os.environ.get('HF_ENDPOINT','<unset>')}")
print("[scene/setup-native] open_clip ViT-B-32 / laion2b_s34b_b79k ...", flush=True)
open_clip.create_model_and_transforms("ViT-B-32", pretrained="laion2b_s34b_b79k")
print("[scene/setup-native] openai clip ViT-B/32 ...", flush=True)
os.makedirs(os.path.expanduser("~/.cache/clip"), exist_ok=True)
clip.load("ViT-B/32", device="cpu", download_root=os.path.expanduser("~/.cache/clip"))
print("[scene/setup-native] CLIP caches OK")
PY

# ── 6. Sanity import check ─────────────────────────────────────────────
"$PY" - <<PY
import importlib, sys
mods = ["torch", "torchvision", "ultralytics", "open_clip", "clip",
        "supervision", "hydra", "omegaconf", "open3d", "conceptgraph",
        "mcp", "fastmcp", "grpc"]
bad = []
for m in mods:
    try:
        importlib.import_module(m)
    except Exception as e:
        bad.append((m, type(e).__name__, str(e)[:80]))
if bad:
    print("[scene/setup-native] some imports still broken:")
    for m, ek, em in bad:
        print(f"  - {m}: {ek}: {em}")
    sys.exit(1)
print(f"[scene/setup-native] all imports OK ({len(mods)} modules)")
PY

echo "[scene/setup-native] done."
echo
echo "Next steps:"
echo "  - Set SCENE_NATIVE_PYTHON=${VENV_DIR}/bin/python in the shell that runs \`rbnx boot\`."
echo "    (Or add it to the deploy manifest's top-level \`env:\` block, which rbnx propagates"
echo "    to every spawned package.)"
echo "  - Set ROBONIX_SCENE_FORCE=native (or ROBONIX_SCENE_PLATFORM=jetson_orin) so"
echo "    \`scripts/start.sh\` picks the native path."
