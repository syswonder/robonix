# SPDX-License-Identifier: MulanPSL-2.0
# ROS 2 Humble + Webots (GUI) + Nav2 + tiago_bridge. Build context = repository root.
FROM osrf/ros:humble-desktop-full
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
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
ARG WEBOTS_DEB_URL=https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
RUN wget -q -O /tmp/webots.deb "${WEBOTS_DEB_URL}" \
    && apt-get update \
    && apt-get install -y --no-install-recommends /tmp/webots.deb \
    && rm -f /tmp/webots.deb \
    && rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN python3 -m pip install --no-cache-dir \
    "grpcio>=1.78.0" "protobuf>=6.30,<7" mcp "fastmcp>=3" numpy Pillow uvicorn httpx

WORKDIR /colcon_ws
COPY rust/examples/packages/tiago_sim_stack/ros_ws/src/eaios_webots src/eaios_webots
RUN bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select eaios_webots"

COPY rust/examples/packages/tiago_sim_stack/nav2_bringup /opt/robonix_nav2
COPY rust/examples/packages/tiago_sim_stack/tiago_bridge /app/tiago_bridge
COPY rust/examples/packages/robonix_mcp_contract /app/robonix_mcp_contract_pkg
COPY rust/examples/packages/tiago_sim_stack/bridge/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


WORKDIR /app
ENV PYTHONPATH=/app:/app/tiago_bridge:/app/tiago_bridge/proto_gen:/app/robonix_mcp_contract_pkg

ENTRYPOINT ["/entrypoint.sh"]
