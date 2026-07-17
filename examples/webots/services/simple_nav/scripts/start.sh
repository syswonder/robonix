#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Run simple_nav INSIDE the webots sim container — same docker-exec pattern
# as primitive drivers. Sim already has ROS Humble + rclpy.
set -euo pipefail

SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
WEBOTS_SCRIPTS="$(cd "$(dirname "$0")/../../../scripts" && pwd)"
# shellcheck source=../../../scripts/container_network.sh
source "$WEBOTS_SCRIPTS/container_network.sh"

if ! docker ps --format '{{.Names}}' | grep -qx "$SIM_CT"; then
    echo "[simple_nav] error: sim container '$SIM_CT' is not running." >&2
    echo "             Bring it up first: bash examples/webots/sim/start.sh" >&2
    exit 1
fi

ATLAS_ENDPOINT="$(resolve_container_atlas_endpoint "$SIM_CT")"
ATLAS_HOST="${ATLAS_ENDPOINT%:*}"
NO_PROXY_VALUE="$(append_no_proxy_hosts "${NO_PROXY:-${no_proxy:-localhost,127.0.0.1}}" "$ATLAS_HOST")"
no_proxy_value="$(append_no_proxy_hosts "${no_proxy:-${NO_PROXY:-localhost,127.0.0.1}}" "$ATLAS_HOST")"

exec docker exec \
    -e ROBONIX_ATLAS="$ATLAS_ENDPOINT" \
    -e ROBONIX_DRIVER_CONTRACT_ID="${ROBONIX_DRIVER_CONTRACT_ID-robonix/lifecycle/driver}" \
    -e ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK="${ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK:-}" \
    -e NO_PROXY="$NO_PROXY_VALUE" \
    -e no_proxy="$no_proxy_value" \
    -e ROBONIX_PKG_HOST_DIR="$PKG" \
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
