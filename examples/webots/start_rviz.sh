#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Top-level convenience wrapper for sim/start_rviz.sh.
#
# Why this exists in addition to sim/start_rviz.sh:
#   - sim/start.sh already auto-launches one rviz on boot, but when that
#     window is closed (Ctrl-C in rviz, accidental kill, etc.) you'd
#     have to remember the docker-exec dance to bring it back.
#   - sim/start_rviz.sh runs rviz in the foreground (its `docker exec`
#     blocks until rviz exits), so re-using it requires `&` + redirect
#     boilerplate every time.
#
# This wrapper:
#   - Resolves its sibling sim/start_rviz.sh from the script's own dir
#     (cwd-independent — works whether you call it from the repo root,
#     from examples/webots/, or via an absolute path),
#   - Detaches via `nohup … &` + `disown` so the rviz process survives
#     the launching shell,
#   - Pipes stdout/stderr to /tmp/rviz2.log so this terminal stays clean.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG="${ROBONIX_RVIZ_LOG:-/tmp/rviz2.log}"

nohup bash "$SCRIPT_DIR/sim/start_rviz.sh" >"$LOG" 2>&1 < /dev/null &
PID=$!
disown "$PID" 2>/dev/null || true
echo "[start_rviz] launched in background (pid $PID); log: $LOG"
