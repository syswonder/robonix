#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scene container entrypoint.
#
# Three processes run in this container, in order:
#   1. rmw_zenohd                 — the Zenoh router; rmw_zenoh_cpp
#                                   ROS2 nodes connect to it as their
#                                   transport.
#   2. zenoh-bridge-dds           — mirrors the sim container's
#                                   FastRTPS topics into Zenoh so
#                                   our rmw_zenoh-side subscribers
#                                   can see them. Without it scene
#                                   only sees publishers that also
#                                   speak rmw_zenoh.
#   3. scene_service.service      — the actual scene Python entry.
#
# All three are siblings in the same shell; we trap on EXIT so that
# Ctrl-C / SIGTERM tears the lot down together.

# `set -u` is incompatible with /opt/ros/humble/setup.bash (it
# references unset AMENT_TRACE_SETUP_FILES). Stick with -eo pipefail.
set -eo pipefail

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd /scene

# Codegen output lives under rbnx-build/codegen/. Build phase produces
# it on the host before this container runs; we just inject onto path.
export PYTHONPATH="/scene/rbnx-build/codegen/proto_gen:/scene/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"

# robonix-api lives in the workspace pylib dir, also bind-mounted.
if [ -d /robonix-api ]; then
    export PYTHONPATH="/robonix-api:${PYTHONPATH}"
fi

mkdir -p /scene/rbnx-build/data

# Direct-DDS path: same RMW (FastRTPS) as sim, --network host shares
# the IP namespace so UDP multicast discovery works, and --ipc=host
# shares /dev/shm so SHM data transfer lines up. No Zenoh router or
# bridge needed in this layout (we tried, the bridge could not see
# FastRTPS-only SHM publishers across containers reliably).
ZENOHD_PID=
BRIDGE_PID=

cleanup() {
    kill -TERM "$BRIDGE_PID" 2>/dev/null || true
    kill -TERM "$ZENOHD_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
    wait "$ZENOHD_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Brief wait so the router is listening before scene_service tries to
# connect. Without this rmw_zenoh_cpp's first node creation can fail
# with "no router available". 1s is plenty on a local socket.
sleep 1

# ── 3. scene_service ────────────────────────────────────────────────────────
exec python3 -m scene_service.service
