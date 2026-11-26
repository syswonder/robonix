#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0

set -euo pipefail

example_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$example_dir/../.." && pwd)"

exec python3 "$repo_root/scripts/fetch_model.py" \
  --manifest "$example_dir/model/model-assets.json" \
  --output "$example_dir/model" \
  "$@"
