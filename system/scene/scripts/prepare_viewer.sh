#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Install the web assets matching Scene's pinned Rerun SDK.
set -euo pipefail
PKG="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="$(sed -n 's/^rerun-sdk==//p' "$PKG/docker/requirements/scene-viewer.txt")"
DEST="${1:?viewer asset destination required}/$VERSION"
if [[ -s "$DEST/index.js" && -s "$DEST/re_viewer.js" && -s "$DEST/re_viewer_bg.wasm" ]]; then
    exit 0
fi
mkdir -p "$(dirname "$DEST")"
STAGING="$(mktemp -d "$(dirname "$DEST")/.viewer.XXXXXX")"
trap 'rm -rf "$STAGING"' EXIT
curl -fSL --retry 3 --connect-timeout 20 \
    "${NPM_REGISTRY:-https://registry.npmmirror.com}/@rerun-io/web-viewer/-/web-viewer-${VERSION}.tgz" \
    -o "$STAGING/viewer.tgz"
mkdir "$STAGING/assets"
tar xzf "$STAGING/viewer.tgz" -C "$STAGING/assets" --strip-components=1 \
    package/index.js package/re_viewer.js package/re_viewer_bg.wasm
for name in index.js re_viewer.js re_viewer_bg.wasm; do
    [[ -f "$STAGING/assets/$name" && ! -L "$STAGING/assets/$name" && -s "$STAGING/assets/$name" ]]
done
rm -rf "$DEST"
mv "$STAGING/assets" "$DEST"
