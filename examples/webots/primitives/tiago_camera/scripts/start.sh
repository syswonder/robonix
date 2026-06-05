#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_camera runtime — docker-exec into the pre-running sim container.
# See tiago_chassis/scripts/start.sh for trap-discipline rationale.
set -euo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx robonix_tiago_sim; then
  echo "[tiago_camera] error: sim container 'robonix_tiago_sim' is not running."
  echo "                Bring it up first:  bash examples/webots/sim/start.sh"
  exit 1
fi

cleanup() {
  docker exec robonix_tiago_sim pkill -9 -f 'camera_driver|static_transform_publisher.*head_front_camera' 2>/dev/null || true
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Webots-compat static TFs: the eaios_webots controller stamps RGB images
# with frame_id="head_front_camera_rgb_optical_frame" but the URDF tree
# exposes the corresponding link as "Astra rgb". Without a bridge, anything
# that looks up TF for that optical frame (rtabmap RGBD fusion, scene 3D
# fusion) fails. We publish identity static TFs `Astra rgb` → optical frame
# so the names line up. Compensation lives here (camera primitive) — never
# in mapping/scene.
docker exec -i -d robonix_tiago_sim bash -lc "
    source /opt/ros/humble/setup.bash
    exec ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
        --frame-id 'Astra rgb' --child-frame-id head_front_camera_rgb_optical_frame
" &>/dev/null
docker exec -i -d robonix_tiago_sim bash -lc "
    source /opt/ros/humble/setup.bash
    exec ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
        --frame-id 'Astra depth' --child-frame-id head_front_camera_depth_optical_frame
" &>/dev/null

docker exec -i \
  -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
  -e ROBONIX_PKG_HOST_DIR="$(cd "$(dirname "$0")/.." && pwd)" \
  -e TIAGO_RGB_TOPIC="${TIAGO_RGB_TOPIC:-/head_front_camera/rgb/image_raw}" \
  -e TIAGO_DEPTH_TOPIC="${TIAGO_DEPTH_TOPIC:-/head_front_camera/depth_registered/image_raw}" \
  -e TIAGO_RGB_FRAME_ID="${TIAGO_RGB_FRAME_ID:-head_front_camera_rgb_optical_frame}" \
  -e TIAGO_DEPTH_FRAME_ID="${TIAGO_DEPTH_FRAME_ID:-head_front_camera_depth_optical_frame}" \
  -e PYTHONPATH="/robonix_pkgs/pylib/robonix-api" \
  robonix_tiago_sim \
  bash -lc 'source /opt/ros/humble/setup.bash
            OVL=/robonix_pkgs/primitives/tiago_camera/rbnx-build/codegen/ros2_idl/install/setup.bash
            [ -f "$OVL" ] && source "$OVL" || true
            cd /robonix_pkgs/primitives/tiago_camera
            exec python3 -m camera_driver.driver' &
wait $!
