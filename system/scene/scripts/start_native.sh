#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scene service native (no-docker) launcher.
#
# Equivalent of docker/entrypoint.sh, but executed directly on the host
# ROS 2 install. Picked by scripts/start.sh when the deploy manifest's
# launching shell sets ROBONIX_SCENE_FORCE=native (or ROBONIX_SCENE_PLATFORM
# matches the native whitelist — currently jetson_orin).
#
# Same single-process flow as the container path:
#
#   exec python3 -m scene_service.service
#
# Removed vs container path:
#   - docker run / image build: scene runs as a host process in the
#     same DDS namespace as every other native robonix package.
#   - SHM-disable FastRTPS profile: with all participants in the same
#     /dev/shm namespace (everyone is a host process now), SHM transport
#     works fine; UDP-only is no longer required. We respect any user-set
#     FASTRTPS_DEFAULT_PROFILES_FILE / RMW_IMPLEMENTATION in the parent
#     shell instead of forcing them.
#   - bind-mounted /scene + /robonix-api: not needed; we already cd into
#     the package on disk and resolve robonix-api via `rbnx path`.
#
# Pre-conditions (the operator is expected to have these on the host;
# we check + fail loud rather than silently degrading):
#
#   1. ROS 2 Humble installed (or available at /opt/ros/humble) plus
#      python3-cv-bridge + ros-humble-tf2-ros + ros-humble-tf-transformations.
#      apt-equivalent: sudo apt install ros-humble-ros-base \
#          python3-cv-bridge ros-humble-tf2-ros ros-humble-tf-transformations
#   2. A Python venv (or system Python) with scene's deps. Easiest path:
#      `bash scripts/setup_native.sh` (one-shot installer / verifier).
#      Manual: install torch + everything in docker/requirements.txt
#      into a venv, point SCENE_NATIVE_PYTHON at its python3.
#   3. CV model weights at SCENE_YOLO_WORLD_WEIGHTS / SCENE_MOBILE_SAM_WEIGHTS
#      (defaults: docker/_weights/yolov8l-world.pt + docker/_weights/mobile_sam.pt,
#      pre-fetched by scripts/build.sh's pre_fetch_weights block).
#   4. CLIP caches warm (first inference will otherwise reach out to the
#      net for ~500 MB of weights — robonix invariant violation on
#      customer hardware).
#   5. Codegen output present at rbnx-build/codegen/ (produced by
#      scripts/build.sh's `rbnx codegen` step; native skips the docker
#      build step but still needs codegen for atlas proto stubs).
#
# Trap discipline mirrors the container's: SIGTERM tears down the
# scene_service process tree.

set -eo pipefail

source_setup_bash() {
    local setup="$1"
    local old_opts="$-"
    # ament/colcon setup scripts are not guaranteed to be safe under
    # strict caller environments; relax for the duration of the source.
    export COLCON_TRACE="${COLCON_TRACE:-}"
    set +u
    # shellcheck disable=SC1090
    source "$setup"
    case "$old_opts" in
        *u*) set -u ;;
        *)   set +u ;;
    esac
}

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ── Pre-flight: ROS 2 ──────────────────────────────────────────────────
# rbnx boot may preserve ROS_DISTRO but not PATH/AMENT_PREFIX_PATH, so
# checking ROS_DISTRO alone is not enough.
if [[ -z "${ROS_DISTRO:-}" || -z "${AMENT_PREFIX_PATH:-}" ]] || ! command -v ros2 >/dev/null 2>&1; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source_setup_bash /opt/ros/humble/setup.bash
    else
        echo "[scene-native] ERR: ROS 2 not sourced and /opt/ros/humble/setup.bash missing." >&2
        echo "[scene-native]      Either source ros2 in the launching shell, or set" >&2
        echo "[scene-native]      ROBONIX_SCENE_FORCE=docker to fall back to the containerised path." >&2
        exit 2
    fi
fi

# tf2_ros + cv_bridge are dlopen'd lazily by rclpy bindings inside scene;
# verify they're discoverable now so the failure mode is clear up front
# rather than a 30-second perception-wait timeout later.
for pkg in tf2_ros cv_bridge; do
    if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        echo "[scene-native] ERR: ROS 2 package '$pkg' missing." >&2
        echo "[scene-native]      sudo apt install python3-cv-bridge ros-humble-tf2-ros ros-humble-tf-transformations" >&2
        exit 2
    fi
done

# ── Pre-flight: codegen output ────────────────────────────────────────
CODEGEN="${PKG}/rbnx-build/codegen"
if [[ ! -d "${CODEGEN}/proto_gen" ]]; then
    echo "[scene-native] ERR: ${CODEGEN}/proto_gen missing — run \`bash scripts/build.sh\` first" >&2
    echo "[scene-native]      (it runs \`rbnx codegen\` to generate atlas_pb2 etc.)" >&2
    exit 2
fi

# ── PYTHONPATH ─────────────────────────────────────────────────────────
# Container path uses /scene + /robonix-api; native uses the package
# root + the location reported by `rbnx path robonix-api` (same
# resolution the docker start.sh did).
export PYTHONPATH="${PKG}:${CODEGEN}/proto_gen:${CODEGEN}/robonix_mcp_types:${PYTHONPATH:-}"
if command -v rbnx >/dev/null 2>&1; then
    if ROBONIX_API_DIR="$(rbnx path robonix-api 2>/dev/null)" && [[ -d "$ROBONIX_API_DIR" ]]; then
        export PYTHONPATH="${ROBONIX_API_DIR}:${PYTHONPATH}"
    fi
fi

mkdir -p "${PKG}/rbnx-build/data"

# ── Env defaults (mirror the docker -e block) ──────────────────────────
export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-com.robonix.system.scene}"
export ROBONIX_PKG_HOST_DIR="${PKG}"
export SCENE_WEB_PORT="${SCENE_WEB_PORT:-50107}"
export SCENE_LOG_LEVEL="${SCENE_LOG_LEVEL:-INFO}"
export SCENE_PERCEPTION_WAIT_S="${SCENE_PERCEPTION_WAIT_S:-30}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# CV model weights default to the same docker/_weights/ files the
# container path bakes into the image. Operator can override either
# variable to point at custom weights.
export SCENE_YOLO_WORLD_WEIGHTS="${SCENE_YOLO_WORLD_WEIGHTS:-${PKG}/docker/_weights/yolov8l-world.pt}"
export SCENE_MOBILE_SAM_WEIGHTS="${SCENE_MOBILE_SAM_WEIGHTS:-${PKG}/docker/_weights/mobile_sam.pt}"
export SCENE_CLIP_MODEL="${SCENE_CLIP_MODEL:-ViT-B-32}"
export SCENE_CLIP_PRETRAINED="${SCENE_CLIP_PRETRAINED:-laion2b_s34b_b79k}"

# Sanity-check the weights — perception_concept_graphs.py would crash
# on first inference otherwise, and the failure mode (60 s perception
# wait then a one-line "detector init failed") is hard to debug.
for w in "$SCENE_YOLO_WORLD_WEIGHTS" "$SCENE_MOBILE_SAM_WEIGHTS"; do
    if [[ ! -s "$w" ]]; then
        echo "[scene-native] WARN: CV weight missing or empty: $w" >&2
        echo "[scene-native]       Run \`bash scripts/setup_native.sh\` (or scripts/build.sh)" >&2
        echo "[scene-native]       to fetch yolov8l-world.pt + mobile_sam.pt into docker/_weights/." >&2
        echo "[scene-native]       (Continuing — perception falls back to VLM if rgb is the only stream;" >&2
        echo "[scene-native]        if rgb+depth are both available the ConceptGraphs path will fail at start.)" >&2
    fi
done

# RMW: do NOT override here. The Jetson Orin host is the same DDS namespace
# as the chassis / lidar / camera primitives (all running native FastRTPS
# defaults), so any forced RMW would surprise the operator without upside.
# Same logic applies to FASTRTPS_DEFAULT_PROFILES_FILE — the container
# needed UDP-only because /dev/shm SHM was incompatible across containers,
# but on a single native host SHM is fine.

# ── Optional: switch to a venv if SCENE_NATIVE_PYTHON points at one ────
# scene's deps (torch + ultralytics + open_clip + open3d + …) are heavy
# enough that operators usually maintain a dedicated venv. Honour
# SCENE_NATIVE_PYTHON if set; fall back to system python3 otherwise.
PYTHON="${SCENE_NATIVE_PYTHON:-python3}"
if ! "$PYTHON" -c "import sys; sys.exit(0 if sys.version_info >= (3,10) else 2)" 2>/dev/null; then
    echo "[scene-native] ERR: $PYTHON is not Python >=3.10" >&2
    echo "[scene-native]      Set SCENE_NATIVE_PYTHON=/path/to/venv/bin/python (>=3.10)." >&2
    exit 2
fi

# Quick import-check the heavy deps that the container path bakes in.
# We don't try to install them here (that's setup_native.sh's job) —
# this is a fail-loud sanity gate.
#
# `set +e` around the probe so a non-zero rc lands in $? instead of
# triggering errexit and exiting the script with a generic non-zero
# (which would hide the actual diagnostic).
set +e
"$PYTHON" - <<'PY'
import importlib, sys
needed = [
    ("torch",          "torch"),
    ("ultralytics",    "ultralytics"),
    ("open_clip",      "open_clip_torch"),
    ("clip",           "clip-anytorch"),
    ("supervision",    "supervision==0.14.0"),
    ("hydra",          "hydra-core==1.3.2"),
    ("omegaconf",      "omegaconf==2.3.0"),
    ("open3d",         "open3d>=0.17,<0.19"),
    ("conceptgraph",   "(install from https://github.com/concept-graphs/concept-graphs branch ali-dev)"),
    ("rclpy",          "(apt: ros-humble-rclpy)"),
    ("cv_bridge",      "(apt: python3-cv-bridge)"),
    ("robonix_api",    "(rbnx path robonix-api on PYTHONPATH)"),
    ("scene_service",  "(this package; check PYTHONPATH includes pkg root)"),
]
missing = []
for mod, hint in needed:
    try:
        importlib.import_module(mod)
    except Exception as e:
        missing.append((mod, hint, type(e).__name__, str(e)[:80]))
if missing:
    print("[scene-native] missing/broken Python imports:", file=sys.stderr)
    for mod, hint, ek, em in missing:
        print(f"  - {mod:18s} {ek}: {em}  (install: {hint})", file=sys.stderr)
    sys.exit(1)
PY
probe_rc=$?
set -e
if [[ "$probe_rc" -ne 0 ]]; then
    echo "[scene-native] ERR: scene Python deps missing in $PYTHON env." >&2
    echo "[scene-native]      Run \`bash scripts/setup_native.sh\` to install them" >&2
    echo "[scene-native]      (or set SCENE_NATIVE_PYTHON to a venv that has them)." >&2
    exit 2
fi

# ── Trap discipline ────────────────────────────────────────────────────
SCENE_PID=
cleanup() {
    [[ -n "$SCENE_PID" ]] && kill -TERM "$SCENE_PID" 2>/dev/null || true
    pkill -TERM -P $$ 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ── Run ────────────────────────────────────────────────────────────────
echo "[scene-native] launching scene_service.service"
echo "[scene-native]   atlas=${ROBONIX_ATLAS}"
echo "[scene-native]   cap=${ROBONIX_CAPABILITY_ID}"
echo "[scene-native]   web_port=${SCENE_WEB_PORT}"
echo "[scene-native]   python=${PYTHON}"
echo "[scene-native]   yolo=${SCENE_YOLO_WORLD_WEIGHTS}"
echo "[scene-native]   sam=${SCENE_MOBILE_SAM_WEIGHTS}"

# `python3 -u` for unbuffered stdout/stderr so log lines land in
# rbnx boot's tail in real time. service.py also configures
# basicConfig() with the same level so the format matches the
# container path verbatim.
exec "$PYTHON" -u -m scene_service.service
