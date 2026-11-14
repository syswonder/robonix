#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scene container entrypoint.
#
# The container uses the same ROS 2 RMW as the rest of the deploy and
# starts scene_service.service directly.

# `set -u` is incompatible with ROS's setup.bash (it references unset
# AMENT_TRACE_SETUP_FILES). Stick with -eo pipefail.
set -eo pipefail

# Source whichever ROS distro this image was built against. The ros base
# image exports ROS_DISTRO; fall back to humble for safety. This is the
# only ROS coupling in scene, and it follows the build-time ROS_DISTRO
# arg — Robonix does not bind to a single ROS release.
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

# Robonix ros2_idl overlay (generated `map` interface package — mapping's
# latched lifecycle broadcast that scene's map binding subscribes to).
# Packaged images carry this overlay; legacy images use the workspace build.
# When absent, Scene falls back to static map binding.
ROS2_IDL_SETUP=/scene/rbnx-build/codegen/ros2_idl/install/setup.bash
if [ -f "$ROS2_IDL_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$ROS2_IDL_SETUP"
fi

configure_zenoh_session() {
    if [ "${RMW_IMPLEMENTATION:-}" != "rmw_zenoh_cpp" ] || [ -z "${ROBONIX_ZENOH_ROUTER:-}" ]; then
        return 0
    fi
    local src="/opt/ros/${ROS_DISTRO:-humble}/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"
    local dst="/tmp/robonix_zenoh_session.json5"
    if [ ! -f "$src" ]; then
        echo "[entrypoint] missing Zenoh session config: $src" >&2
        return 1
    fi
    local mode="${ROBONIX_ZENOH_MODE:-client}"
    sed \
        -e "s#mode: \"peer\"#mode: \"${mode}\"#" \
        -e "s#\"tcp/localhost:7447\"#\"${ROBONIX_ZENOH_ROUTER}\"#g" \
        "$src" > "$dst"
    if [ -n "${ROBONIX_ZENOH_LISTEN:-}" ]; then
        sed -i "s#\"tcp/localhost:0\"#\"${ROBONIX_ZENOH_LISTEN}\"#g" "$dst"
    fi
    export ZENOH_SESSION_CONFIG_URI="$dst"
    export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-20}"
    echo "[entrypoint] rmw_zenoh_cpp mode=${mode} router=${ROBONIX_ZENOH_ROUTER} listen=${ROBONIX_ZENOH_LISTEN:-<default>}"
}

configure_zenoh_session

cd /scene

# Generate stubs with the image's protobuf toolchain: during image construction
# for packaged images, or in the launcher for legacy workspace-mounted images.
# The conventional proto_gen path must resolve to the same output because
# robonix-api prepends it when constructing a provider.
SCENE_PROTO_GEN=/scene/rbnx-build/codegen/scene_proto_gen
if [ ! -f "$SCENE_PROTO_GEN/atlas_pb2.py" ] \
   || [ ! -f "$SCENE_PROTO_GEN/robonix_contracts_pb2_grpc.py" ]; then
    echo "[entrypoint] missing runtime-compatible Scene protobuf stubs" >&2
    exit 1
fi
export PYTHONPATH="$SCENE_PROTO_GEN:/scene/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"

# Both packaged and workspace-mounted images expose the API here.
if [ -d /robonix-api ]; then
    export PYTHONPATH="/robonix-api:${PYTHONPATH}"
fi

mkdir -p /scene/rbnx-build/data

# Brief wait so the router is listening before scene_service tries to
# connect. Without this rmw_zenoh_cpp's first node creation can fail
# with "no router available". 1s is plenty on a local socket.
sleep 1

# ── 3. scene_service ────────────────────────────────────────────────────────
exec python3 -m scene_service.service
