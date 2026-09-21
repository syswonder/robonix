#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Bring this worktree's scene up, reproducibly.
#
#   scripts/dev-up.sh            clean stale boots, check the image, boot, report
#   scripts/dev-up.sh --down     tear this worktree's boot down and stop
#   scripts/dev-up.sh --pin      pin the current good image by ID and exit
#
# Four things have gone wrong often enough to be worth a script rather than a
# note. Each one presents as "the page looks nearly right", which is why they
# cost an afternoon each time instead of a minute:
#
#   1. The image tag gets stolen. `robonix-scene` is a mutable tag two
#      worktrees build onto, and `robonix-scene-248` has now pointed at an
#      image with no rerun in it -- twice. Scene then serves the built-in
#      canvas and says nothing. So the image is pinned by ID here, and this
#      script refuses to boot an image that cannot import rerun.
#
#   2. `rbnx` resolves the source tree from ~/.robonix/config.yaml, which is
#      shared with every other deployment on the host. It has pointed at
#      /tmp/cleanrobonix while this worktree sat unused -- boot ran somebody
#      else's scene, and the only symptom was routes 404ing. ROBONIX_HOME
#      redirects that config per-worktree without touching the shared one.
#
#   3. Stale `rbnx boot` processes accumulate: thirteen were found, the oldest
#      a day and seven hours. They hold ports and confuse every check. Only
#      this worktree's are cleaned -- other deployments on this host
#      (robonix-lab, scene-243) are left strictly alone.
#
#   4. A boot started over ssh dies with the session. `nohup` alone did not
#      save it; `setsid` does.
set -euo pipefail

SCENE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WORKTREE="$(cd "$SCENE_DIR/../.." && pwd)"
DEPLOY="$WORKTREE/examples/webots"
MANIFEST="${SCENE_DEV_MANIFEST:-robonix_manifest.scene-ui-local.yaml}"
PIN_FILE="$SCENE_DIR/scripts/dev-image.pin"
ROBONIX_HOME_DIR="${ROBONIX_HOME:-$HOME/.robonix-$(basename "$WORKTREE")}"
PORT="${SCENE_WEB_PORT:-50107}"

say() { printf '\033[1m[dev-up]\033[0m %s\n' "$*"; }
die() { printf '\033[31m[dev-up]\033[0m %s\n' "$*" >&2; exit 1; }

# ── 3. only this worktree's boots ──────────────────────────────────────────
stop_ours() {
  local found=0 p cwd
  for p in $(pgrep -f 'rbnx boot' 2>/dev/null || true); do
    cwd="$(readlink "/proc/$p/cwd" 2>/dev/null || true)"
    case "$cwd" in
      "$WORKTREE"/*)
        found=1
        say "stopping stale boot $p"
        kill -TERM -"$(ps -o pgid= -p "$p" | tr -d ' ')" 2>/dev/null || kill -TERM "$p" 2>/dev/null || true
        ;;
    esac
  done
  [ "$found" = 1 ] && sleep 8
  docker rm -f robonix_scene >/dev/null 2>&1 || true
  return 0
}

if [ "${1:-}" = "--down" ]; then stop_ours; say "down"; exit 0; fi

# ── 1. the image, pinned and checked ───────────────────────────────────────
has_rerun() {
  docker run --rm --entrypoint python3 "$1" \
    -c 'import rerun; print(rerun.__version__)' 2>/dev/null | tail -1
}

if [ "${1:-}" = "--pin" ]; then
  img="${ROBONIX_SCENE_IMAGE:-robonix-scene}"
  v="$(has_rerun "$img")"
  [ -n "$v" ] || die "$img cannot import rerun; refusing to pin it"
  docker image inspect -f '{{.Id}}' "$img" > "$PIN_FILE"
  say "pinned $img ($(cat "$PIN_FILE")), rerun $v"
  exit 0
fi

[ -f "$PIN_FILE" ] || die "no pinned image. Build or pick one, then: $0 --pin
  (ROBONIX_SCENE_IMAGE=<tag> $0 --pin to pin a specific tag)"
IMG="$(cat "$PIN_FILE")"
docker image inspect "$IMG" >/dev/null 2>&1 \
  || die "pinned image $IMG is gone. Re-pin: ROBONIX_SCENE_IMAGE=<tag> $0 --pin"

RERUN_V="$(has_rerun "$IMG")"
[ -n "$RERUN_V" ] || die "pinned image $IMG cannot import rerun.
  This is the failure that looks like a working page: scene falls back to the
  built-in canvas and the only clue is that /api/viewer 404s. Fix the image,
  then re-pin. Refusing to boot."
say "image $IMG  (rerun $RERUN_V)"

# ── 2. a source tree that is ours ──────────────────────────────────────────
mkdir -p "$ROBONIX_HOME_DIR/packages"
cat > "$ROBONIX_HOME_DIR/config.yaml" <<EOF
package_storage_path: $ROBONIX_HOME_DIR/packages
robonix_source_path: $WORKTREE
EOF
resolved="$(ROBONIX_HOME="$ROBONIX_HOME_DIR" rbnx path root 2>/dev/null || true)"
[ "$resolved" = "$WORKTREE" ] \
  || die "rbnx resolves the source tree to '$resolved', not '$WORKTREE'"
say "source tree $resolved  (ROBONIX_HOME=$ROBONIX_HOME_DIR, shared config untouched)"

stop_ours

# ── 4. survive the ssh session ─────────────────────────────────────────────
cd "$DEPLOY"
LOG="${SCENE_DEV_LOG:-/tmp/scene-boot-$(basename "$WORKTREE").log}"
ROBONIX_HOME="$ROBONIX_HOME_DIR" \
ROBONIX_SCENE_IMAGE="$IMG" \
ROBONIX_SOMA_ROBOT_YAML="${ROBONIX_SOMA_ROBOT_YAML:-soma.yaml}" \
  setsid nohup rbnx boot -f "$MANIFEST" < /dev/null > "$LOG" 2>&1 &
disown || true
say "booting ($MANIFEST), log: $LOG"

for _ in $(seq 1 90); do
  if [ "$(curl -s -o /dev/null -m 2 -w '%{http_code}' "http://127.0.0.1:$PORT/api/viewer" 2>/dev/null)" = 200 ]; then
    break
  fi
  sleep 5
done

code="$(curl -s -o /dev/null -m 3 -w '%{http_code}' "http://127.0.0.1:$PORT/api/viewer" 2>/dev/null || echo 000)"
[ "$code" = 200 ] || die "scene did not answer on $PORT (last: $code). See $LOG"

frames="$(curl -s -m 5 "http://127.0.0.1:$PORT/" | grep -c 'rerun?view=3d' || true)"
say "scene up on $PORT"
if [ "${frames:-0}" -gt 0 ]; then
  say "landing page serves the rerun viewer"
else
  die "scene is up but the landing page is the built-in canvas.
  The image has rerun, so this is not the tag problem -- check make_app got a
  rerun_sink, and RerunSink.available. See $LOG"
fi
