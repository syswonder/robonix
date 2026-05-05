#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Run simple_nav INSIDE the webots sim container — same docker-exec
# pattern as primitive drivers. Sim already has ROS Humble + rclpy.
set -euo pipefail

SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"
ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
MCP_PORT="${SIMPLE_NAV_MCP_PORT:-50122}"

cleanup() {
    docker exec "$SIM_CT" sh -c 'pkill -TERM -f simple_nav.atlas_bridge 2>/dev/null || true' 2>/dev/null || true
    kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

exec docker exec -i \
    -e ROBONIX_ATLAS="$ATLAS" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e SIMPLE_NAV_MCP_PORT="$MCP_PORT" \
    -e RBNX_CONFIG_FILE="${RBNX_CONFIG_FILE:-}" \
    "$SIM_CT" bash -lc '
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        cd /robonix_pkgs/services/simple_nav
        export PYTHONPATH="$(pwd):$(pwd)/proto_gen:/robonix_pkgs/pylib/robonix-py:${PYTHONPATH:-}"
        exec python3 -m simple_nav.atlas_bridge
    '
