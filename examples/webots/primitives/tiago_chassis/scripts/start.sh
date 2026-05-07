#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_chassis runtime — docker-exec into the pre-running sim container.
# Sim must be brought up first by `bash examples/webots/sim/start.sh`.
#
# Trap discipline: when `rbnx boot` SIGTERMs our PGID, bash's EXIT/TERM
# trap fires and explicitly kills the python interpreter inside the
# container. `docker exec` doesn't reliably propagate signals to the
# exec'd process, so without this trap the chassis_driver stays alive
# and breaks the next boot.
set -euo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx robonix_tiago_sim; then
  echo "[tiago_chassis] error: sim container 'robonix_tiago_sim' is not running."
  echo "                Bring it up first:  bash examples/webots/sim/start.sh"
  exit 1
fi

cleanup() {
  docker exec robonix_tiago_sim pkill -9 -f 'chassis_driver' 2>/dev/null || true
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

docker exec -i \
  -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
  -e ROBONIX_PKG_HOST_DIR="$(cd "$(dirname "$0")/.." && pwd)" \
  -e PYTHONPATH="/robonix_pkgs/pylib/robonix-api:/robonix_pkgs/primitives/tiago_chassis/rbnx-build/codegen/proto_gen:/robonix_pkgs/primitives/tiago_chassis/rbnx-build/codegen/robonix_mcp_types" \
  robonix_tiago_sim \
  bash -lc 'source /opt/ros/humble/setup.bash && \
            cd /robonix_pkgs/primitives/tiago_chassis && \
            exec python3 -m chassis_driver.driver' &
wait $!
