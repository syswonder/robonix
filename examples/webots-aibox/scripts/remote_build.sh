#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build one TIAGo primitive package on the sim host. The driver code that
# executes lives in the sim host's robonix checkout (the sim container
# bind-mounts examples/webots), so the codegen must run there, not on the
# board — the board-side stamp this writes only gates re-runs of
# `rbnx build`.
set -euo pipefail

PKG="${1:?usage: remote_build.sh <primitive-package-name>}"
DIR="$(cd "$(dirname "$0")/.." && pwd)"
[[ -f "$DIR/remote.env" ]] || {
  echo "[webots-aibox] missing $DIR/remote.env — copy remote.env.example and fill in the machine values" >&2
  exit 1
}
# shellcheck disable=SC1091
source "$DIR/remote.env"

exec ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST_SSH" "
  export PATH=\"\$HOME/.cargo/bin:\$HOME/.local/bin:\$PATH\"
  cd '$HOST_WEBOTS_ROOT/examples/webots/primitives/$PKG' && exec rbnx build -p .
"
