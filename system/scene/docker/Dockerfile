# syntax=docker/dockerfile:1.7
# SPDX-License-Identifier: MulanPSL-2.0
# system/scene container — ROS Humble base + scene's Python deps.
#
# Why scene gets its own image (and isn't, say, exec'd into the
# webots sim container): scene is a system service that should be
# robot-agnostic. Pinning it to the webots Dockerfile would make
# every soma's container have to bundle scene's deps. Separate image
# keeps coupling minimal — scene only needs to share the host DDS
# bus, which `--network host` gives us regardless of where the sim
# is running.
#
# Layout inside the image:
#   /opt/ros/humble/...       ROS Humble (from base)
#   /scene                    package dir, bind-mounted at runtime
#                             (so live edits don't need image rebuild)
#   /entrypoint.sh            sources ros + execs scene_service.service
#
# The image is built on demand by `bash system/scene/scripts/build.sh`
# the first time, then reused. Rebuild only when this Dockerfile or
# requirements.txt change.

FROM ros:humble-ros-base

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

# ── TUNA mirrors for GFW-bound networks ─────────────────────────────
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
            ros-humble-tf2-ros \
            ros-humble-tf-transformations \
            ros-humble-rmw-zenoh-cpp \
            ros-humble-zenoh-bridge-dds && \
        rm -rf /var/lib/apt/lists/* \
    '

# pip: do NOT use proxy.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy python3 -m pip install --upgrade pip

# torch + torchvision pinned to cu124 — required for sm_120.
# pip: do NOT use proxy; use Aliyun PyTorch wheels + Aliyun PyPI.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy python3 -m pip install \
        --progress-bar raw \
        --timeout 60 \
        --retries 3 \
        -i https://mirrors.aliyun.com/pypi/simple/ \
        --find-links https://mirrors.aliyun.com/pytorch-wheels/cu124/ \
        torch==2.5.1 torchvision==0.20.1

# Generic scene deps + concept-graphs perception stack.
# pip: do NOT use proxy; use PIP_INDEX_URL=TUNA.

# base
COPY requirements/scene-base.txt /tmp/requirements/scene-base.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy pip install -r /tmp/requirements/scene-base.txt \
        --progress-bar raw \
        --timeout 60 \
        --retries 3 \
    && rm /tmp/requirements/scene-base.txt

# core
COPY requirements/scene-perception-core.txt /tmp/requirements/scene-perception-core.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy pip install -r /tmp/requirements/scene-perception-core.txt \
        --progress-bar raw \
        --timeout 60 \
        --retries 3 \
    && rm /tmp/requirements/scene-perception-core.txt

# network heavy
COPY requirements/scene-perception-heavy.txt /tmp/requirements/scene-perception-heavy.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy pip install -r /tmp/requirements/scene-perception-heavy.txt \
        --progress-bar raw \
        --timeout 60 \
        --retries 3 \
    && rm /tmp/requirements/scene-perception-heavy.txt

# pytorch3d (see requirements/scene-pytorch3d.txt to decide whether installing it or not.)
# COPY requirements/scene-pytorch3d.txt /tmp/requirements/scene-pytorch3d.txt
# RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
#     without-proxy pip install -r /tmp/requirements/scene-pytorch3d.txt \
#         --progress-bar raw \
#         --timeout 60 \
#         --retries 3 \
#     && rm /tmp/requirements/scene-pytorch3d.txt

# ── Proxy-needed network layers start here ─────────────────────────
# Declare build args as late as possible to avoid invalidating earlier
# apt/pip layers when proxy values change.
ARG HTTP_PROXY_HOST
ARG HTTPS_PROXY_HOST
ARG NO_PROXY_HOST

# concept-graphs source from the ali-dev branch.
# git clone: use proxy.
RUN set -eux; \
    rm -rf /opt/concept-graphs; \
    with-build-proxy git clone --depth 1 --branch ali-dev \
        https://ghfast.top/https://github.com/concept-graphs/concept-graphs.git \
        /opt/concept-graphs \
    || ( \
        rm -rf /opt/concept-graphs && \
        with-build-proxy git clone --depth 1 --branch ali-dev \
            https://github.com/concept-graphs/concept-graphs.git \
            /opt/concept-graphs \
    )

# concept-graphs editable install.
# pip: do NOT use proxy.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    without-proxy pip install --no-deps -e /opt/concept-graphs

# Bake CV model weights into the image.
ENV HF_HOME=/opt/models/hf

RUN mkdir -p /opt/models /opt/models/hf /root/.cache/clip

COPY _weights/yolov8l-world.pt /opt/models/yolov8l-world.pt
COPY _weights/mobile_sam.pt    /opt/models/mobile_sam.pt

# Two CLIP-flavored downloads bake in here:
#   1. open_clip's ViT-B-32 LAION-2B — goes to HF_HOME.
#   2. openai's `clip` ViT-B/32 — goes to /root/.cache/clip.
#
# Non-pip model downloads: use proxy.
RUN with-build-proxy sh <<'EOF'
set -eux
unset HF_ENDPOINT
export HF_HUB_DOWNLOAD_TIMEOUT=120

python3 -c "import open_clip; open_clip.create_model_and_transforms('ViT-B-32', pretrained='laion2b_s34b_b79k')"
python3 -c "import clip; clip.load('ViT-B/32', device='cpu', download_root='/root/.cache/clip')"

ls -la /opt/models/ /root/.cache/clip/
EOF

# No global proxy ENV was set, so there is no build-time proxy to clear here.

# Fixed env so perception_concept_graphs.py knows where the weights
# live without each deployment having to override.
ENV SCENE_YOLO_WORLD_WEIGHTS=/opt/models/yolov8l-world.pt \
    SCENE_MOBILE_SAM_WEIGHTS=/opt/models/mobile_sam.pt \
    SCENE_CLIP_MODEL=ViT-B-32 \
    SCENE_CLIP_PRETRAINED=laion2b_s34b_b79k

WORKDIR /scene

COPY entrypoint.sh /entrypoint.sh
COPY no_shm_profile.xml /etc/fastrtps_no_shm.xml

RUN chmod +x /entrypoint.sh

# Force FastRTPS to skip SHM transport.
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastrtps_no_shm.xml

# Stick with FastRTPS.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ENTRYPOINT ["/entrypoint.sh"]