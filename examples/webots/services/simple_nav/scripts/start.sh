#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Run simple_nav INSIDE the webots sim container — same docker-exec pattern
# as primitive drivers. Sim already has ROS Humble + rclpy.
set -euo pipefail

SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"

exec docker exec \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
    "$SIM_CT" bash -lc '
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        OVL=/robonix_pkgs/services/simple_nav/rbnx-build/codegen/ros2_idl/install/setup.bash
        [ -f "$OVL" ] && source "$OVL" || true
        cd /robonix_pkgs/services/simple_nav
        export PYTHONPATH="$(pwd):/robonix_pkgs/pylib/robonix-api:${PYTHONPATH:-}"
        exec python3 -m simple_nav.atlas_bridge
    '
