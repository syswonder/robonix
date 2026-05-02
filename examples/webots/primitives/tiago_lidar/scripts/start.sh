#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_lidar runtime — docker-exec into the pre-running sim container.
# See tiago_chassis/scripts/start.sh for trap-discipline rationale.
set -euo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx robonix_tiago_sim; then
  echo "[tiago_lidar] error: sim container 'robonix_tiago_sim' is not running."
  echo "              Bring it up first:  bash examples/webots/sim/start.sh"
  exit 1
fi

cleanup() {
  docker exec robonix_tiago_sim pkill -9 -f 'lidar_driver' 2>/dev/null || true
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

docker exec -i \
  -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
  -e ROBONIX_PKG_HOST_DIR="$(cd "$(dirname "$0")/.." && pwd)" \
  -e TIAGO_LIDAR_MCP_PORT="${TIAGO_LIDAR_MCP_PORT:-50113}" \
  -e TIAGO_LIDAR_DRIVER_PORT="${TIAGO_LIDAR_DRIVER_PORT:-50213}" \
  -e TIAGO_SCAN_TOPIC="${TIAGO_SCAN_TOPIC:-/scanner}" \
  robonix_tiago_sim \
  bash -lc 'source /opt/ros/humble/setup.bash && \
            cd /robonix_pkgs/primitives/tiago_lidar && \
            exec python3 -m lidar_driver.driver' &
wait $!
