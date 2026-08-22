#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_camera runtime — docker-exec into the pre-running sim container.
# Lifecycle cleanup is handled by Driver(CMD_SHUTDOWN) + manifest stop hook.
set -euo pipefail

# Sim container name — overridable via ROBONIX_SIM_CONTAINER for isolated
# CI / parallel deploys. Default keeps single-deploy behaviour.
SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"
WEBOTS_SCRIPTS="$(cd "$(dirname "$0")/../../../scripts" && pwd)"
# shellcheck source=../../../scripts/container_network.sh
source "$WEBOTS_SCRIPTS/container_network.sh"

if ! docker ps --format '{{.Names}}' | grep -qx "$SIM_CT"; then
  echo "[tiago_camera] error: sim container '$SIM_CT' is not running."
  echo "                Bring it up first:  bash examples/webots/sim/start.sh"
  exit 1
fi

# Webots-compat static TFs: the eaios_webots controller stamps RGB images
# with frame_id="head_front_camera_rgb_optical_frame" but the URDF tree
# exposes the corresponding link as "Astra rgb". Without a bridge, anything
# that looks up TF for that optical frame (rtabmap RGBD fusion, scene 3D
# fusion) fails. We publish identity static TFs `Astra rgb` → optical frame
# so the names line up. Compensation lives here (camera primitive) — never
# in mapping/scene.
docker exec -d "$SIM_CT" bash -lc "
    set +u
    source /opt/ros/humble/setup.bash >/dev/null
    export RMW_IMPLEMENTATION=\"${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}\"
    exec ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
        --frame-id 'Astra rgb' --child-frame-id head_front_camera_rgb_optical_frame
" &>/dev/null
docker exec -d "$SIM_CT" bash -lc "
    set +u
    source /opt/ros/humble/setup.bash >/dev/null
    export RMW_IMPLEMENTATION=\"${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}\"
    exec ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
        --frame-id 'Astra depth' --child-frame-id head_front_camera_depth_optical_frame
" &>/dev/null

ADVERTISE_HOST="$(resolve_advertise_host)"
ATLAS_ENDPOINT="$(resolve_container_atlas_endpoint "$SIM_CT")"
ATLAS_HOST="${ATLAS_ENDPOINT%:*}"
NO_PROXY_VALUE="$(append_no_proxy_hosts "${NO_PROXY:-${no_proxy:-localhost,127.0.0.1}}" "$ATLAS_HOST" "$ADVERTISE_HOST")"
no_proxy_value="$(append_no_proxy_hosts "${no_proxy:-${NO_PROXY:-localhost,127.0.0.1}}" "$ATLAS_HOST" "$ADVERTISE_HOST")"

exec docker exec \
  -e ROBONIX_ATLAS="$ATLAS_ENDPOINT" \
  -e ROBONIX_DRIVER_CONTRACT_ID="${ROBONIX_DRIVER_CONTRACT_ID-robonix/lifecycle/driver}" \
  -e ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK="${ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK:-}" \
  -e ROBONIX_ADVERTISE_HOST="$ADVERTISE_HOST" \
  -e NO_PROXY="$NO_PROXY_VALUE" \
  -e no_proxy="$no_proxy_value" \
  -e ROBONIX_PKG_HOST_DIR="$(cd "$(dirname "$0")/.." && pwd)" \
  -e TIAGO_RGB_TOPIC="${TIAGO_RGB_TOPIC:-/head_front_camera/rgb/image_raw}" \
  -e TIAGO_DEPTH_TOPIC="${TIAGO_DEPTH_TOPIC:-/head_front_camera/depth_registered/image_raw}" \
  -e TIAGO_RGB_FRAME_ID="${TIAGO_RGB_FRAME_ID:-head_front_camera_rgb_optical_frame}" \
  -e TIAGO_DEPTH_FRAME_ID="${TIAGO_DEPTH_FRAME_ID:-head_front_camera_depth_optical_frame}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
  -e PYTHONPATH="/robonix_pkgs/pylib/robonix-api:/robonix_pkgs/primitives/tiago_camera/rbnx-build/codegen/proto_gen:/robonix_pkgs/primitives/tiago_camera/rbnx-build/codegen/robonix_mcp_types:/robonix_pkgs/primitives/tiago_camera/robonix_mcp_types" \
  "$SIM_CT" \
  bash -lc 'set -eo pipefail
            set +u
            source /opt/ros/humble/setup.bash >/dev/null
            cd /robonix_pkgs/primitives/tiago_camera
            LOG=/tmp/tiago_camera_driver.log
            : > "$LOG"
            python3 -m camera_driver.driver >>"$LOG" 2>&1 &
            DRIVER_PID=$!
            tail --pid="$DRIVER_PID" -n +1 -F "$LOG" &
            TAIL_PID=$!
            set +e
            wait "$DRIVER_PID"
            STATUS=$?
            set -e
            kill "$TAIL_PID" 2>/dev/null || true
            wait "$TAIL_PID" 2>/dev/null || true
            exit "$STATUS"
            '
