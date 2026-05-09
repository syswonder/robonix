# SPDX-License-Identifier: MulanPSL-2.0
# ROS 2 Humble + Webots (GUI) + Nav2 deps + Python deps for robonix
# drivers that get `docker exec`'d into this container by `rbnx boot`.
# This image is the SIM container; it does NOT contain robonix driver
# code. Driver code lives in each primitive/service package directory
# on the host and is bind-mounted in by compose.yaml under /robonix_pkgs.
# Build context = the directory containing this Dockerfile (sim/bridge/).
FROM osrf/ros:humble-desktop-full
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libxcb-cursor0 \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-nav2-msgs \
    ros-humble-nav2-bringup \
    ros-humble-webots-ros2 \
    ros-humble-webots-ros2-driver \
    ros-humble-controller-manager \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    wget \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Webots simulator (Cyberbotics .deb; apt "webots" is often unavailable on minimal images)
# Match ros-humble-webots-ros2 (~2025.x) — see https://github.com/cyberbotics/webots/releases
# Default URL uses ghfast.top mirror for reliable access; override with
# --build-arg WEBOTS_DEB_URL=https://github.com/... for direct GitHub.
ARG WEBOTS_DEB_URL=https://ghfast.top/https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
RUN wget -q -O /tmp/webots.deb "${WEBOTS_DEB_URL}" \
    && apt-get update \
    && apt-get install -y --no-install-recommends /tmp/webots.deb \
    && rm -f /tmp/webots.deb \
    && rm -rf /var/lib/apt/lists/*

# Pre-populate Webots asset cache. Without this, Webots fetches ~100 files from
# raw.githubusercontent.com at first run — often blocked or painfully slow.
# The seed tarball is exported from a successful run; see scripts/update-webots-seed.sh.
COPY bridge/webots_assets_seed.tar.gz /tmp/webots_seed.tar.gz
RUN mkdir -p /root/.cache/Cyberbotics/Webots \
    && tar -xzf /tmp/webots_seed.tar.gz -C /root/.cache/Cyberbotics/Webots \
    && rm -f /tmp/webots_seed.tar.gz \
    && echo "[dockerfile] webots asset cache: $(ls /root/.cache/Cyberbotics/Webots/assets | wc -l) files pre-populated"

# Python deps used by the robonix driver processes that will be `docker
# exec`'d into this container. (Drivers themselves live on host; their
# Python imports resolve against this image's site-packages + the
# bind-mounted package dirs under /robonix_pkgs.)
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN python3 -m pip install --no-cache-dir \
    "grpcio>=1.78.0" "protobuf>=6.30,<7" mcp "fastmcp>=3" numpy Pillow uvicorn httpx

# Build context is sim/bridge/ — ros_ws lives one level up at sim/ros_ws/.
# Compose passes context: ../ so we COPY from there.
WORKDIR /colcon_ws
COPY ros_ws/src/eaios_webots src/eaios_webots
RUN bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select eaios_webots"

COPY bridge/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /
ENTRYPOINT ["/entrypoint.sh"]
