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

# Sim container name — overridable via ROBONIX_SIM_CONTAINER so a CI / parallel
# deploy can target its OWN isolated Webots container instead of clobbering the
# shared default (the cleanup pkill below would otherwise kill another run's
# drivers inside a shared container). Default keeps single-deploy behaviour.
SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"

if ! docker ps --format '{{.Names}}' | grep -qx "$SIM_CT"; then
  echo "[tiago_chassis] error: sim container '$SIM_CT' is not running."
  echo "                Bring it up first:  bash examples/webots/sim/start.sh"
  exit 1
fi

cleanup() {
  docker exec "$SIM_CT" pkill -9 -f 'chassis_driver' 2>/dev/null || true
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Cross-host wiring for an isolated (bridge-network) sim: the driver runs INSIDE
# the sim container but registers with an atlas on the host, and the host
# executor must dial the driver's MCP endpoint back.
#  - ROBONIX_SIM_ATLAS: atlas address reachable FROM the container (the bridge
#    gateway, e.g. 172.17.0.1:50151). rbnx overrides ROBONIX_ATLAS to a host
#    localhost value the container can't reach, so this takes precedence.
#  - ROBONIX_ADVERTISE_HOST: the container's own IP, so it advertises an MCP URL
#    the host executor can reach. The container has no `ip` cmd to self-resolve,
#    so compute it host-side. Both default empty on a host-network sim, where
#    127.0.0.1 already works for both directions.
SIM_IP="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$SIM_CT" 2>/dev/null || true)"

docker exec -i \
  -e ROBONIX_ATLAS="${ROBONIX_SIM_ATLAS:-${ROBONIX_ATLAS:-127.0.0.1:50051}}" \
  -e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-$SIM_IP}" \
  -e ROBONIX_PKG_HOST_DIR="$(cd "$(dirname "$0")/.." && pwd)" \
  -e PYTHONPATH="/robonix_pkgs/pylib/robonix-api" \
  "$SIM_CT" \
  bash -lc 'set -eo pipefail
            source /opt/ros/humble/setup.bash
            OVL=/robonix_pkgs/primitives/tiago_chassis/rbnx-build/codegen/ros2_idl/install/setup.bash
            [ -f "$OVL" ] && source "$OVL" || true
            cd /robonix_pkgs/primitives/tiago_chassis
            exec python3 -m chassis_driver.driver' &
wait $!
