#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_nav2 runtime — launch Nav2 stack inside the sim container, then
# exec the wrapper that opens an ActionClient against navigate_to_pose
# and registers MCP caps with atlas.
set -euo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx robonix_tiago_sim; then
  echo "[tiago_nav2] error: sim container 'robonix_tiago_sim' is not running."
  echo "             Bring it up first:  bash examples/webots/sim/start.sh"
  exit 1
fi

# Single docker-exec session: launch nav2 in the background, then run
# the wrapper in the foreground. When `rbnx start` SIGTERMs us, the
# wrapper exits and bash's EXIT trap kills the nav2 launch tree.
exec docker exec -i \
  -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
  -e ROBONIX_PKG_HOST_DIR="$(cd "$(dirname "$0")/.." && pwd)" \
  -e TIAGO_NAV2_MCP_PORT="${TIAGO_NAV2_MCP_PORT:-50121}" \
  -e TIAGO_NAV2_DRIVER_PORT="${TIAGO_NAV2_DRIVER_PORT:-50221}" \
  -e TIAGO_NAV2_WAIT_SEC="${TIAGO_NAV2_WAIT_SEC:-30}" \
  -e NAV2_WARMUP_SEC="${NAV2_WARMUP_SEC:-15}" \
  robonix_tiago_sim \
  bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    NAV2_CFG=/robonix_pkgs/services/tiago_nav2/nav2_config/config

    cleanup() { kill -- "-$$" 2>/dev/null || true; }
    trap cleanup EXIT INT TERM

    echo "[tiago_nav2] launching nav2_bringup (warmup ${NAV2_WARMUP_SEC}s)..."
    ros2 launch nav2_bringup bringup_launch.py \
      map:="${NAV2_CFG}/my_map.yml" \
      use_sim_time:=true \
      params_file:="${NAV2_CFG}/nav2_params.yml" &
    sleep "${NAV2_WARMUP_SEC}"

    cd /robonix_pkgs/services/tiago_nav2
    exec python3 -m nav2_driver.driver
  '
