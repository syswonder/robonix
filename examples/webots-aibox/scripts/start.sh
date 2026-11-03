#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Foreground SSH wrapper: run one TIAGo primitive's `rbnx start` on the sim
# host, so boot/Soma on the board can supervise remote drivers like local
# packages. Soma spawns this with RBNX_DEPLOY_MANAGED=1 locally and boot owns
# the Driver(INIT/ACTIVATE) sequence; the remote side must behave the same
# way, hence RBNX_DEPLOY_MANAGED is re-exported over ssh — otherwise the
# remote wrapper would drive its own lifecycle and double-init the driver.
#
# Lifecycle: stdout/stderr flow back through the ssh channel. When the
# channel dies (rbnx shutdown kills this process, terminal closes, link
# drops) sshd HUPs the remote side; stop.sh sweeps any leftover wrapper or
# driver there. The config arrives via Driver(CMD_INIT) from the board, so
# no --set values belong here.
set -euo pipefail

PKG="${1:?usage: start.sh <primitive-package-name>}"
DIR="$(cd "$(dirname "$0")/.." && pwd)"
[[ -f "$DIR/remote.env" ]] || {
  echo "[webots-aibox] missing $DIR/remote.env — copy remote.env.example and fill in the machine values" >&2
  exit 1
}
# shellcheck disable=SC1091
source "$DIR/remote.env"
# Matches compose.yaml's container_name default; set SIM_CT in remote.env to
# override a custom ROBONIX_SIM_CONTAINER deployment.
SIM_CT="${SIM_CT:-robonix_tiago_sim}"

# The sim container must already run on the sim host — the drivers
# docker-exec into it. Fail here, not at driver-registration time.
running="$(ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST_SSH" \
  "docker inspect -f '{{.State.Running}}' '$SIM_CT' 2>/dev/null || true")"
if [[ "$running" != "true" ]]; then
  echo "[webots-aibox] sim container '$SIM_CT' is not running on '$HOST_SSH' —" \
    "start the Webots sim first (examples/webots/sim/start.sh)" >&2
  exit 1
fi

exec ssh -o BatchMode=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  "$HOST_SSH" "
    export PATH=\"\$HOME/.cargo/bin:\$HOME/.local/bin:\$PATH\"
    set -euo pipefail
    export ROBONIX_ATLAS='$BOARD_ATLAS'
    export ROBONIX_ADVERTISE_HOST='$SIM_HOST_IP'
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export RBNX_DEPLOY_MANAGED=1
    export RBNX_INSTANCE_NAME='$PKG'
    exec rbnx start -p '$HOST_WEBOTS_ROOT/examples/webots/primitives/$PKG' --endpoint '$BOARD_ATLAS'
  "
