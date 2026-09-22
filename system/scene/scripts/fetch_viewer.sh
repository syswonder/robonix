#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Fetch the rerun web viewer this scene embeds.
#
# Vendored rather than loaded from a CDN: a robot on a closed network still
# has to be able to draw its own map. Not committed, because the wasm is 48 MB
# and would sit in every clone of the repository forever.
#
# The version must match the rerun-sdk pinned in
# docker/requirements/scene-viewer.txt -- the viewer and the recording it
# reads are one protocol.
set -euo pipefail
VERSION="${RERUN_VIEWER_VERSION:-0.37.1}"
DEST="$(cd "$(dirname "$0")/.." && pwd)/scene_service/web_assets/rerun"
TMP="$(mktemp -d)"
trap "rm -rf $TMP" EXIT
( cd "$TMP" && npm pack "@rerun-io/web-viewer@$VERSION" >/dev/null   && tar xzf "rerun-io-web-viewer-$VERSION.tgz" )
mkdir -p "$DEST"
cp "$TMP"/package/index.js "$TMP"/package/re_viewer.js    "$TMP"/package/re_viewer_bg.wasm "$DEST/"
echo "[fetch_viewer] @rerun-io/web-viewer@$VERSION -> $DEST"
