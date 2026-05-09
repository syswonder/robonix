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

# `ros:humble-ros-base` is the slim official image (no GUI / desktop
# tooling). osrf/ros:* would also work but the user's registry mirror
# can't always reach the osrf namespace.
FROM ros:humble-ros-base

# ── TUNA mirrors for GFW-bound networks ─────────────────────────────
# Swap apt sources (Ubuntu/ports + ROS 2) to TUNA. Use `find` + sed
# --follow-symlinks because /etc/apt/sources.list.d/ros2.sources is
# a symlink into /usr/share/ros-apt-source/ in the upstream ros image.
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
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple/ \
    PIP_TRUSTED_HOST=pypi.tuna.tsinghua.edu.cn \
    HF_ENDPOINT=https://hf-mirror.com

# System packages we need at runtime:
#   python3-pip          — install scene deps
#   python3-cv-bridge    — sensor_msgs/Image ↔ numpy (apt has the
#                          rclpy-aware cv_bridge that pip doesn't)
#   python3-numpy        — pinned to 1.x by ROS Humble; matches host
#   ros-humble-tf2-ros   — tf transforms
#   ros-humble-sensor-msgs / nav-msgs already in -ros-base
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-cv-bridge \
        python3-numpy \
        ros-humble-tf2-ros \
        ros-humble-tf-transformations \
        ros-humble-rmw-zenoh-cpp \
        ros-humble-zenoh-bridge-dds \
 && rm -rf /var/lib/apt/lists/*

# torch + torchvision pinned to cu124 — required for sm_120 (Blackwell
# RTX 5090); the cu118 wheels concept-graphs originally pinned can't
# even import on this hardware. CPU-only torch fallback would be too
# slow for YOLO-World/MobileSAM. Jetson AGX builds will swap this for
# NVIDIA's torch fork wheel; the rest of the deps in requirements.txt
# stay identical because they're CPU-side or torch-only consumers.
RUN pip install --index-url https://download.pytorch.org/whl/cu124 \
        torch==2.5.1 torchvision==0.20.1

# Generic scene deps + concept-graphs perception stack (no
# pytorch3d / chamferdist / gradslam — those are needed only by
# concept-graphs' full SLAM pipeline, not by detection + merge,
# which is all we use).
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt \
 && rm /tmp/requirements.txt

# concept-graphs source from the ali-dev branch (smaller, no
# Grounded-SAM / pytorch3d dependency tangle). We need the
# `conceptgraph.slam.utils` and `conceptgraph.slam.mapping` modules
# (depth backproject + per-object pcd + DBSCAN-based merge). Install
# editable so we can edit-and-test from inside the container without
# rebuilding when iterating on their helpers.
RUN git clone --depth 1 --branch ali-dev \
        https://github.com/concept-graphs/concept-graphs.git \
        /opt/concept-graphs \
 && pip install -e /opt/concept-graphs \
 || (echo "concept-graphs install failed — see comment in Dockerfile" \
     && exit 1)

# Bake CV model weights into the image — robonix invariant: a robot
# booting on a customer network can't reach github releases /
# huggingface CDN. ali-dev's recommended stack:
#   yolov8l-world.pt — 91 MB, open-vocab YOLO via CLIP text encoder
#   mobile_sam.pt    — 40 MB, MobileSAM for promptable masks
# Plus open_clip ViT-B-32 LAION-2B for per-object CLIP features (~150 MB,
# small enough for Jetson; 5090 build can swap to ViT-L/14 via env).
#
# YOLO + MobileSAM are pre-downloaded onto the host into docker/_weights/
# (out-of-band, see scripts/build.sh's `pre_fetch_weights` block) and
# COPY'd into the image. We tried `RUN curl` from inside buildx but
# github CDN connections from CN drop mid-stream after ~10 minutes and
# break a multi-hundred-MB build. Out-of-band download with retries is
# more robust and lets buildx cache the COPY layer.
#
# open_clip ViT-B-32 LAION-2B comes from huggingface via HF_ENDPOINT
# (hf-mirror.com); much smaller and reliable.
ENV HF_HOME=/opt/models/hf
RUN mkdir -p /opt/models /opt/models/hf /root/.cache/clip
COPY _weights/yolov8l-world.pt /opt/models/yolov8l-world.pt
COPY _weights/mobile_sam.pt    /opt/models/mobile_sam.pt
# Two CLIP-flavored downloads bake in here:
#   1. open_clip's ViT-B-32 LAION-2B (we use this for per-detection
#      visual-similarity dedup in concept-graphs) — goes to HF_HOME.
#   2. openai's `clip` ViT-B/32 (separate package; ultralytics'
#      YOLO-World text encoder calls clip.load("ViT-B/32") during
#      set_classes() and would otherwise pull 354 MB from openai's
#      CDN at first inference — robonix invariant violation).
RUN python3 -c "import open_clip; open_clip.create_model_and_transforms('ViT-B-32', pretrained='laion2b_s34b_b79k')" \
 && python3 -c "import clip; clip.load('ViT-B/32', device='cpu', download_root='/root/.cache/clip')" \
 && ls -la /opt/models/ /root/.cache/clip/

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

# Force FastRTPS to skip SHM transport — see mapping_rbnx/docker for
# full reasoning. UDP-only is the only path that data-flows reliably
# across our containers (sim ↔ scene ↔ mapping) in this layout.
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastrtps_no_shm.xml

# Stick with FastRTPS (sim's default RMW). With --network host +
# --ipc=host the SHM transport shares cleanly between sim and scene,
# and FastRTPS UDP multicast discovery covers the rest. We tried
# Zenoh first but `zenoh_bridge_dds` cannot see FastRTPS-only SHM
# publishers reliably across containers, so this is the path that
# actually delivers /amcl_pose, /odom, /scanner, /head_front_camera/*
# into scene's rclpy hub.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ENTRYPOINT ["/entrypoint.sh"]
