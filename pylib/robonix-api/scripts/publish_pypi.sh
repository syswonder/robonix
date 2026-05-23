#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Upload the wheels in dist/ to real PyPI. Run scripts/build_dist.sh first
# (to populate dist/) and ideally scripts/publish_testpypi.sh once for a
# smoke test before pushing to the world.
#
# Auth: set TWINE_PASSWORD to a PyPI API token
#   (https://pypi.org/manage/account/token/).
# TWINE_USERNAME defaults to `__token__` per PyPI's token spec.
#
#   export TWINE_PASSWORD='pypi-AgEIcHl…'
#   bash scripts/publish_pypi.sh
#
# PyPI uploads are irreversible — a version can be yanked but the
# version number itself cannot be re-used. Double-check the version
# field in pyproject.toml before running.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "$HERE/.." && pwd)"

if [[ ! -d "$PKG_ROOT/dist" ]] || ! ls "$PKG_ROOT"/dist/*.whl >/dev/null 2>&1; then
    echo "ERR: no dist/*.whl in $PKG_ROOT/dist — run scripts/build_dist.sh first." >&2
    exit 1
fi

PYTHON="${PYTHON:-python3}"
if ! "$PYTHON" -c "import twine" 2>/dev/null; then
    echo "ERR: twine not installed. Install with: $PYTHON -m pip install twine" >&2
    exit 1
fi

: "${TWINE_USERNAME:=__token__}"
if [[ -z "${TWINE_PASSWORD:-}" ]]; then
    echo "ERR: TWINE_PASSWORD not set. Get a token from" >&2
    echo "     https://pypi.org/manage/account/token/" >&2
    exit 1
fi
export TWINE_USERNAME TWINE_PASSWORD

# Display version + artefacts and require operator confirmation; PyPI
# uploads are one-shot so an extra second to read what's about to land
# is the right tradeoff.
VERSION="$(grep -E "^version" "$PKG_ROOT/pyproject.toml" | head -1 | cut -d'"' -f2)"
echo "[publish_pypi] about to upload robonix-api version: $VERSION"
echo "[publish_pypi] artefacts:"
ls -la "$PKG_ROOT"/dist/
echo ""
read -r -p "Push to PyPI (irreversible)? [y/N] " confirm
if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    echo "[publish_pypi] aborted."
    exit 0
fi

echo "[publish_pypi] twine check"
"$PYTHON" -m twine check "$PKG_ROOT"/dist/*

echo "[publish_pypi] uploading to PyPI"
"$PYTHON" -m twine upload "$PKG_ROOT"/dist/*

echo "[publish_pypi] done. Smoke-test:"
echo "    pip install robonix-api==$VERSION"
