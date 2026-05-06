#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Run simple_nav INSIDE the webots sim container — same docker-exec pattern
# as primitive drivers. Sim already has ROS Humble + rclpy.
set -euo pipefail

SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"

cleanup() {
    docker exec "$SIM_CT" sh -c 'pkill -TERM -f simple_nav.atlas_bridge 2>/dev/null || true' 2>/dev/null || true
    kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

exec docker exec -i \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    "$SIM_CT" bash -lc '
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        cd /robonix_pkgs/services/simple_nav
        export PYTHONPATH="$(pwd):/robonix_pkgs/pylib/robonix-py:$(pwd)/rbnx-build/codegen/proto_gen:$(pwd)/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
        exec python3 -m simple_nav.atlas_bridge
    '
