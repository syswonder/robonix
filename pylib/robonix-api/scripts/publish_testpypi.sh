#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Upload the wheels in dist/ to Test PyPI. Run scripts/build_dist.sh first
# to populate dist/. Test PyPI is the recommended smoke target before a
# real PyPI push — same publish flow, identical metadata validation, no
# user-visible blast radius if a release is malformed.
#
# Auth: set TWINE_PASSWORD to a Test PyPI API token
#   (https://test.pypi.org/manage/account/token/).
# TWINE_USERNAME defaults to `__token__` per Test PyPI's token spec.
#
#   export TWINE_PASSWORD='pypi-AgEIcHl…'
#   bash scripts/publish_testpypi.sh
#
# After upload, smoke-test install from Test PyPI:
#   pip install --index-url https://test.pypi.org/simple/ \
#       --extra-index-url https://pypi.org/simple/ \
#       robonix-api

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
    echo "     https://test.pypi.org/manage/account/token/" >&2
    exit 1
fi
export TWINE_USERNAME TWINE_PASSWORD

echo "[publish_testpypi] twine check"
"$PYTHON" -m twine check "$PKG_ROOT"/dist/*

echo "[publish_testpypi] uploading to Test PyPI"
"$PYTHON" -m twine upload \
    --repository-url https://test.pypi.org/legacy/ \
    "$PKG_ROOT"/dist/*

echo "[publish_testpypi] done. Smoke-test:"
echo "    pip install --index-url https://test.pypi.org/simple/ \\"
echo "                --extra-index-url https://pypi.org/simple/ robonix-api"
