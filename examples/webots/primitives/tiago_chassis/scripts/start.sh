#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_chassis runtime — docker-exec into the pre-running sim container.
# Sim must be brought up first by `bash examples/webots/sim/start.sh`.
#
# Lifecycle is owned by rbnx/Soma: Driver(CMD_SHUTDOWN), then this package's
# manifest stop hook, then wrapper PGID TERM/KILL. Keep start.sh as a foreground
# docker-exec wrapper so provider stdout/stderr stay connected and failures are
# visible in rbnx-boot/logs.
set -euo pipefail

# Sim container name — overridable via ROBONIX_SIM_CONTAINER so a CI / parallel
# deploy can target its OWN isolated Webots container instead of clobbering the
# shared default (the cleanup pkill below would otherwise kill another run's
# drivers inside a shared container). Default keeps single-deploy behaviour.
SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"
WEBOTS_SCRIPTS="$(cd "$(dirname "$0")/../../../scripts" && pwd)"
# shellcheck source=../../../scripts/container_network.sh
source "$WEBOTS_SCRIPTS/container_network.sh"

if ! docker ps --format '{{.Names}}' | grep -qx "$SIM_CT"; then
  echo "[tiago_chassis] error: sim container '$SIM_CT' is not running."
  echo "                Bring it up first:  bash examples/webots/sim/start.sh"
  exit 1
fi

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
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
  -e PYTHONPATH="/robonix_pkgs/pylib/robonix-api:/robonix_pkgs/primitives/tiago_chassis/rbnx-build/codegen/proto_gen" \
  "$SIM_CT" \
  bash -lc 'set -eo pipefail
            set +u
            source /opt/ros/humble/setup.bash >/dev/null
            cd /robonix_pkgs/primitives/tiago_chassis
            LOG=/tmp/tiago_chassis_driver.log
            : > "$LOG"
            python3 -m chassis_driver.driver >>"$LOG" 2>&1 &
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
