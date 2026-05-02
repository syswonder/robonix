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

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

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

# Python deps for scene. Versions chosen to match what
# system/scene/pyproject.toml asks for, except we skip Open3D (no
# ARM/CP wheels needed in the container; plane_extract no-ops).
COPY requirements.txt /tmp/requirements.txt
# Ubuntu 22.04 ships pip 22 which doesn't have PEP 668 enforcement,
# so plain `pip install` suffices. The `--break-system-packages` flag
# is for newer base images; we don't need it here.
RUN pip install -r /tmp/requirements.txt \
 && rm /tmp/requirements.txt

WORKDIR /scene
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Use Zenoh as the RMW. The scene container starts its own Zenoh
# router (rmw_zenohd) and a zenoh-bridge-dds that mirrors the sim
# container's FastRTPS topics into Zenoh. No FastRTPS XML needed
# anymore — that was the workaround for the cross-container SHM
# problem we no longer have.
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

ENTRYPOINT ["/entrypoint.sh"]
