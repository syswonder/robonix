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
  timeout 5s docker exec "$SIM_CT" pkill -9 -f '[c]hassis_driver' 2>/dev/null || true
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Cross-host wiring for an isolated (bridge-network) sim: the driver runs INSIDE
# the sim container but registers with an atlas on the host, and the host
# executor must dial the driver's MCP endpoint back. Host-network sim containers
# do not have a bridge IP; Docker may render that as the literal string
# "invalid IP", so only use the inspected address when it is a valid IPv4.
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

docker exec -i \
  -e ROBONIX_ATLAS="${ROBONIX_SIM_ATLAS:-${ROBONIX_ATLAS:-127.0.0.1:50051}}" \
  -e ROBONIX_ADVERTISE_HOST="$ADVERTISE_HOST" \
  -e ROBONIX_PKG_HOST_DIR="$(cd "$(dirname "$0")/.." && pwd)" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
  -e PYTHONPATH="/robonix_pkgs/pylib/robonix-api" \
  "$SIM_CT" \
  bash -lc 'set -eo pipefail
            source /opt/ros/humble/setup.bash
            OVL=/robonix_pkgs/primitives/tiago_chassis/rbnx-build/codegen/ros2_idl/install/setup.bash
            [ -f "$OVL" ] && source "$OVL" || true
            cd /robonix_pkgs/primitives/tiago_chassis
            exec python3 -m chassis_driver.driver' &
wait $!
