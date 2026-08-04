# SPDX-License-Identifier: MulanPSL-2.0
# system/scene container — ROS 2 base + scene's Python deps.
#
# Why scene gets its own image (and isn't, say, exec'd into the
# webots sim container): scene is a system service that should be
# robot-agnostic. Pinning it to the webots Dockerfile would make
# every soma's container have to bundle scene's deps. Separate image
# keeps coupling minimal — scene only needs to share the host DDS
# bus, which `--network host` gives us regardless of where the sim
# is running.
#
# ROS distro is NOT hard-pinned: Robonix does not bind to a single ROS
# release. Pick it with the `ROS_DISTRO` build arg (default `humble`); the
# base image, the `ros-<distro>-*` apt packages, and the runtime
# `source /opt/ros/<distro>/setup.bash` all follow it. Supported distros
# (have the tf2 + zenoh-bridge packages scene needs): humble (default,
# verified), iron, jazzy, rolling. Set it with
# `ROBONIX_SCENE_ROS_DISTRO=iron rbnx build -p system/scene` (build.sh wires
# the env var to this build arg) — see the scene README "ROS distro" section.
#
# Layout inside the image:
#   /opt/ros/$ROS_DISTRO/...  ROS 2 (from base; $ROS_DISTRO set by the base)
#   /scene                    package dir, bind-mounted at runtime
#                             (so live edits don't need image rebuild)
#   /entrypoint.sh            sources ros + execs scene_service.service
#
# The image is built on demand by `bash system/scene/scripts/build.sh`
# the first time, then reused. Rebuild only when this Dockerfile or
# requirements/ change.

# Selects the ROS package names + runtime setup.bash path. build.sh also passes
# ROS_BASE_IMAGE, defaulting to a local robonix-ros:<distro>-ros-base alias so
# BuildKit does not query Docker Hub metadata on every rebuild. Override with
# ROBONIX_SCENE_ROS_BASE_IMAGE for a mirror, digest-pinned base, or custom image.
ARG ROS_DISTRO=humble
ARG ROS_BASE_IMAGE=robonix-ros:humble-ros-base
FROM ${ROS_BASE_IMAGE}

# Global ARG values can select FROM but are out of scope in the stage.  Keep
# package names independent of whether the selected base exports ROS_DISTRO.
ARG ROS_DISTRO

# ── Build-time proxy helpers ──────────────────────────────────────
# Usage:
#   - apt / pip: use `without-proxy ...`
#   - git / HuggingFace / OpenAI CDN: use `with-build-proxy ...`
#
# Note:
#   HTTP_PROXY_HOST / HTTPS_PROXY_HOST / NO_PROXY_HOST are declared later,
#   just before the first proxy-needed layer, so changing proxy args does
#   not invalidate earlier apt/pip layers.
RUN cat > /usr/local/bin/with-build-proxy <<'EOF' \
 && chmod +x /usr/local/bin/with-build-proxy \
 && cat > /usr/local/bin/without-proxy <<'EOF2' \
 && chmod +x /usr/local/bin/without-proxy
#!/usr/bin/env sh
set -eu

export HTTP_PROXY="${HTTP_PROXY_HOST:-}"
export HTTPS_PROXY="${HTTPS_PROXY_HOST:-}"
export http_proxy="${HTTP_PROXY_HOST:-}"
export https_proxy="${HTTPS_PROXY_HOST:-}"
export NO_PROXY="${NO_PROXY_HOST:-}"
export no_proxy="${NO_PROXY_HOST:-}"

exec "$@"
EOF
#!/usr/bin/env sh
set -eu

unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy
unset ALL_PROXY all_proxy NO_PROXY no_proxy

exec "$@"
EOF2

# ── TUNA mirrors ───────────────────────────────────────────────────
RUN set -eux; \
    find /etc/apt/sources.list.d/ -maxdepth 1 \( -name '*.list' -o -name '*.sources' \) -print0 \
      | xargs -0 -r sed -i --follow-symlinks \
          -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
          -e 's|^Types: deb deb-src$|Types: deb|'; \
    if [ -f /etc/apt/sources.list ]; then \
        sed -i --follow-symlinks \
            -e 's|http://archive\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://security\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://ports\.ubuntu\.com/ubuntu-ports/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/|g' \
            /etc/apt/sources.list; \
    fi

ENV PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    UV_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    HF_ENDPOINT=https://hf-mirror.com

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# System packages we need at runtime.
# apt: do NOT use proxy; we already switched to TUNA mirrors.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    without-proxy sh -c ' \
        apt-get update && \
        apt-get install -y --no-install-recommends \
            python3-pip \
            python3-cv-bridge \
            python3-numpy \
            ros-$ROS_DISTRO-tf2-ros \
            ros-$ROS_DISTRO-tf-transformations \
            ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
            ros-$ROS_DISTRO-rmw-zenoh-cpp \
            ros-$ROS_DISTRO-zenoh-bridge-dds && \
        rm -rf /var/lib/apt/lists/* \
    '

# pip: do NOT use proxy.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy python3 -m pip install \
        --progress-bar on \
        --timeout 180 \
        --retries 3 \
        --upgrade pip

# torch + torchvision — sm_120 (Blackwell / RTX 5090) requires
# torch 2.7+ with CUDA 12.8 kernels. cu124 wheels only cover up to
# sm_90 and will fail with "no kernel image" on Blackwell.
# Aliyun mirror carries cu128 wheels; fall back to upstream PyTorch index.
# torch / torchvision cu128 only live on the aliyun pytorch-wheels mirror
# (TUNA has no cu128), so curl those two big wheels from aliyun — curl's
# -C - (resume) + --retry saturates the link (~11 MB/s), whereas pip's
# downloader does NOT resume and dies in an infinite restart loop when the
# aliyun mirror stalls a ~1 GB transfer partway. The nvidia-* deps are then
# resolved by pip from the default PIP_INDEX_URL (TUNA, set above) — do NOT
# point pip at aliyun's pypi index here, its large-file throttling stalls
# pip the same way.
# The saved filenames MUST be the canonical wheel names (pip parses the
# name/version/abi from the filename and rejects e.g. "torch.whl" with
# "Invalid wheel filename"), which would silently drop every build into
# the slow fallback below after wasting the full aliyun download.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy sh -c 'set -eu; \
        base="https://mirrors.aliyun.com/pytorch-wheels/cu128"; \
        tw="torch-2.7.1+cu128-cp310-cp310-manylinux_2_28_x86_64.whl"; \
        tvw="torchvision-0.22.1+cu128-cp310-cp310-manylinux_2_28_x86_64.whl"; \
        wheel_cache=/root/.cache/pip/scene-wheels; \
        mkdir -p "$wheel_cache"; \
        download_wheel() { \
            url="$1"; dest="$2"; max_time="$3"; attempt=1; \
            while [ "$attempt" -le 6 ]; do \
                if curl -fSL --connect-timeout 20 --max-time "$max_time" \
                        --speed-time 60 --speed-limit 1024 \
                        -C - -o "$dest" "$url"; then \
                    return 0; \
                fi; \
                attempt=$((attempt + 1)); \
                sleep 2; \
            done; \
            return 1; \
        }; \
        download_wheel \
             "$base/torch-2.7.1%2Bcu128-cp310-cp310-manylinux_2_28_x86_64.whl" \
             "$wheel_cache/$tw" 900; \
        download_wheel \
             "$base/torchvision-0.22.1%2Bcu128-cp310-cp310-manylinux_2_28_x86_64.whl" \
             "$wheel_cache/$tvw" 300; \
        python3 -m pip install --progress-bar on --timeout 60 --retries 3 \
             "$wheel_cache/$tw" "$wheel_cache/$tvw"' \
    || with-build-proxy python3 -m pip install \
        --progress-bar on \
        --timeout 60 \
        --retries 3 \
        --index-url https://download.pytorch.org/whl/cu128 \
        torch==2.7.1 torchvision==0.22.1

# Generic scene deps + self-contained perception stack.
# pip: do NOT use proxy; use PIP_INDEX_URL=TUNA.

# Override only the dependency layers below. Keeping this ARG after apt and
# Torch preserves their expensive cache chain when an operator changes PyPI
# mirrors to recover from a stalled regional index.
ARG SCENE_PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
ENV PIP_INDEX_URL=${SCENE_PIP_INDEX_URL} \
    UV_INDEX_URL=${SCENE_PIP_INDEX_URL}

# base
COPY requirements/scene-numpy1-abi.txt /tmp/requirements/scene-numpy1-abi.txt
COPY requirements/scene-base.txt /tmp/requirements/scene-base.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy pip install \
        -c /tmp/requirements/scene-numpy1-abi.txt \
        -r /tmp/requirements/scene-base.txt \
        --progress-bar on \
        --timeout 60 \
        --retries 3 \
    && without-proxy pip uninstall -y faiss-cpu \
    && rm /tmp/requirements/scene-base.txt

# core
COPY patch_ultralytics_runtime.py /tmp/patch_ultralytics_runtime.py
COPY requirements/scene-perception-core.txt /tmp/requirements/scene-perception-core.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy pip install \
        -c /tmp/requirements/scene-numpy1-abi.txt \
        -r /tmp/requirements/scene-perception-core.txt \
        --progress-bar on \
        --timeout 60 \
        --retries 3 \
    && python3 /tmp/patch_ultralytics_runtime.py \
    && without-proxy pip uninstall -y faiss-cpu matplotlib \
    && rm /tmp/requirements/scene-perception-core.txt \
        /tmp/requirements/scene-numpy1-abi.txt \
        /tmp/patch_ultralytics_runtime.py \
    && python3 - <<'PY'
import importlib.util
import cv2
import numpy as np
from cv_bridge import CvBridge

for name in ("faiss", "matplotlib"):
    if importlib.util.find_spec(name) is not None:
        raise RuntimeError(f"unused heavyweight dependency leaked: {name}")
from ultralytics import SAM, YOLO  # noqa: F401
task_map = YOLO.task_map.fget(None)
if "detect" not in task_map or "semantic" in task_map:
    raise RuntimeError("Ultralytics detect task-map patch is inconsistent")
if int(np.__version__.split(".", 1)[0]) >= 2:
    raise RuntimeError("ROS Humble cv_bridge requires the NumPy 1.x ABI")
PY

# pytorch3d (see requirements/scene-pytorch3d.txt to decide whether installing it or not.)
# COPY requirements/scene-pytorch3d.txt /tmp/requirements/scene-pytorch3d.txt
# RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
#     without-proxy pip install -r /tmp/requirements/scene-pytorch3d.txt \
#         --progress-bar on \
#         --timeout 60 \
#         --retries 3 \
#     && rm /tmp/requirements/scene-pytorch3d.txt

# ── Proxy-needed network layers start here ─────────────────────────
# Declare build args as late as possible to avoid invalidating earlier
# apt/pip layers when proxy values change.
ARG HTTP_PROXY_HOST
ARG HTTPS_PROXY_HOST
ARG NO_PROXY_HOST

# Bake CV model weights into the image.
ENV HF_HOME=/opt/models/hf \
    YOLO_CONFIG_DIR=/opt/ultralytics

RUN mkdir -p /opt/models /opt/models/hf /opt/models/ultralytics/clip /opt/ultralytics

COPY bake_yolo_world.py /tmp/bake_yolo_world.py
COPY bake_fp16_weights.py /tmp/bake_fp16_weights.py
COPY yolo_world_classes.json /tmp/yolo_world_classes.json

# Two CLIP-flavored downloads bake in here:
#   1. open_clip's ViT-B-32 LAION-2B — copied above from the host model cache.
#   2. Ultralytics YOLO-World's text encoder — goes to the Ultralytics
#      weights_dir under /opt/models/ultralytics, only while the fixed prompt
#      embeddings are baked. The text-encoder package and weights are removed
#      in the same layer and are not present at runtime.
#
# open_clip receives a local checkpoint path, so it never calls the Hugging
# Face metadata API. Ultralytics uses openai/clip internally from
# YOLOWorld.set_classes(); run that exact path at build time so start never
# downloads the 354 MB ViT-B-32.pt into /scene/weights/clip.
RUN --mount=type=bind,source=_weights/yolov8l-world.pt,target=/tmp/yolov8l-world.pt,ro \
    --mount=type=bind,source=_weights/yolov8s-worldv2.pt,target=/tmp/yolov8s-worldv2.pt,ro \
    --mount=type=bind,source=_weights/mobile_sam.pt,target=/tmp/mobile_sam.pt,ro \
    --mount=type=bind,source=_weights/open_clip_pytorch_model.bin,target=/tmp/open_clip_pytorch_model.bin,ro \
    --mount=type=bind,source=_weights/ViT-B-32.pt,target=/opt/models/ultralytics/clip/ViT-B-32.pt,ro \
    with-build-proxy sh <<'EOF'
set -eux
export YOLO_CONFIG_DIR=/opt/ultralytics
without-proxy python3 -m pip install --no-deps "clip-anytorch>=2.6"

python3 /tmp/bake_fp16_weights.py \
    --open-clip-input /tmp/open_clip_pytorch_model.bin \
    --open-clip-output /opt/models/open_clip_vit_b32.fp16.safetensors \
    --mobile-sam-input /tmp/mobile_sam.pt \
    --mobile-sam-output /opt/models/mobile_sam.fp16.safetensors
python3 -c "import open_clip; open_clip.create_model_and_transforms('ViT-B-32', pretrained='/opt/models/open_clip_vit_b32.fp16.safetensors')"

python3 - <<'PY'
from ultralytics.utils import SETTINGS

SETTINGS.update({
    "weights_dir": "/opt/models/ultralytics",
    "datasets_dir": "/tmp/ultralytics/datasets",
    "runs_dir": "/tmp/ultralytics/runs",
    "sync": False,
})
PY

python3 /tmp/bake_yolo_world.py \
    --input /tmp/yolov8l-world.pt \
    --output /opt/models/yolov8l-world-baked.safetensors \
    --architecture yolov8l-world.yaml \
    --classes /tmp/yolo_world_classes.json
python3 /tmp/bake_yolo_world.py \
    --input /tmp/yolov8s-worldv2.pt \
    --output /opt/models/yolov8s-worldv2-baked.safetensors \
    --architecture yolov8s-worldv2.yaml \
    --classes /tmp/yolo_world_classes.json

test -s /opt/models/yolov8l-world-baked.safetensors
test -s /opt/models/yolov8l-world-baked.classes.json
test -s /opt/models/yolov8s-worldv2-baked.safetensors
test -s /opt/models/yolov8s-worldv2-baked.classes.json
test -s /opt/models/mobile_sam.fp16.safetensors
test -s /opt/models/open_clip_vit_b32.fp16.safetensors
without-proxy python3 -m pip uninstall -y clip-anytorch
rm -f /tmp/bake_yolo_world.py /tmp/bake_fp16_weights.py /tmp/yolo_world_classes.json
python3 - <<'PY'
import importlib.util
if importlib.util.find_spec("clip") is not None:
    raise RuntimeError("YOLO text encoder package leaked into the runtime image")
PY
chmod -R a+rX,u+rwX /opt/ultralytics
chmod a+r /opt/models/*.safetensors /opt/models/*.json

ls -la /opt/models/
EOF

# No global proxy ENV was set, so there is no build-time proxy to clear here.

# Fixed env so perception_concept_graphs.py knows where the weights
# live without each deployment having to override.
ENV SCENE_YOLO_WORLD_WEIGHTS=/opt/models/yolov8l-world-baked.safetensors \
    SCENE_YOLO_WORLD_WEIGHTS_FULL=/opt/models/yolov8l-world-baked.safetensors \
    SCENE_YOLO_WORLD_WEIGHTS_JETSON=/opt/models/yolov8s-worldv2-baked.safetensors \
    SCENE_MOBILE_SAM_WEIGHTS=/opt/models/mobile_sam.fp16.safetensors \
    YOLO_CONFIG_DIR=/opt/ultralytics \
    SCENE_CLIP_MODEL=ViT-B-32 \
    SCENE_CLIP_PRETRAINED=/opt/models/open_clip_vit_b32.fp16.safetensors

# start.sh regenerates protobuf stubs with this image's exact runtime ABI.
# Keep this late so a codegen-tool-only change does not rebake perception
# models or invalidate their large layers.
COPY requirements/scene-runtime-codegen.txt /tmp/requirements/scene-runtime-codegen.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy python3 -m pip install \
        --progress-bar on --timeout 180 --retries 3 \
        -r /tmp/requirements/scene-runtime-codegen.txt \
    && python3 -m grpc_tools.protoc --version \
    && rm /tmp/requirements/scene-runtime-codegen.txt

WORKDIR /scene

COPY entrypoint.sh /entrypoint.sh
COPY no_shm_profile.xml /etc/fastrtps_no_shm.xml

RUN chmod +x /entrypoint.sh

# Left for operators who explicitly override RMW back to FastRTPS.
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastrtps_no_shm.xml

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

ENTRYPOINT ["/entrypoint.sh"]
