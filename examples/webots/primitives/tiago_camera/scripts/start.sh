#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_camera runtime — docker-exec into the pre-running sim container.
# Lifecycle cleanup is handled by Driver(CMD_SHUTDOWN) + manifest stop hook.
set -euo pipefail

# Sim container name — overridable via ROBONIX_SIM_CONTAINER for isolated
# CI / parallel deploys. Default keeps single-deploy behaviour.
SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"

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

# Cross-host wiring for an isolated (bridge-network) sim — see tiago_chassis
# start.sh for the rationale. Host-network sim containers do not have a bridge
# IP, so fall back to localhost unless Docker returns a valid bridge IPv4.
resolve_advertise_host() {
  if [ -n "${ROBONIX_ADVERTISE_HOST:-}" ]; then
    printf '%s\n' "$ROBONIX_ADVERTISE_HOST"
    return
  fi
  local network_mode inspected
  network_mode="$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$SIM_CT" 2>/dev/null || true)"
  if [ "$network_mode" = "host" ]; then
    printf '%s\n' "127.0.0.1"
    return
  fi
  inspected="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$SIM_CT" 2>/dev/null || true)"
  if [[ "$inspected" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    printf '%s\n' "$inspected"
    return
  fi
  printf '%s\n' "127.0.0.1"
}

ADVERTISE_HOST="$(resolve_advertise_host)"

exec docker exec \
  -e ROBONIX_ATLAS="${ROBONIX_SIM_ATLAS:-${ROBONIX_ATLAS:-127.0.0.1:50051}}" \
  -e ROBONIX_ADVERTISE_HOST="$ADVERTISE_HOST" \
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
            OVL=/robonix_pkgs/primitives/tiago_camera/rbnx-build/codegen/ros2_idl/install/setup.bash
            [ -f "$OVL" ] && source "$OVL" >/dev/null || true
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
