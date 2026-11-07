#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
EXAMPLE_ROOT="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$EXAMPLE_ROOT/../.." && pwd)"
[[ -f "$EXAMPLE_ROOT/settings.json" ]] || { echo 'Copy settings.example.json to settings.json and configure the hardware paths.' >&2; exit 1; }
configured="$(python3 - "$REPO_ROOT" "$EXAMPLE_ROOT/settings.json" <<'PY'
import json
import os
import sys

repo, settings_file = sys.argv[1:]
settings = json.load(open(settings_file, encoding="utf-8"))
for key in ("calibration_root", "camera_python", "runtime_python", "atlas_binary"):
    print(os.path.abspath(os.path.join(repo, settings[key])))
PY
)"
mapfile -t paths <<< "$configured"
export NOVA5_EXAMPLE_ROOT="$EXAMPLE_ROOT"
export NOVA5_REPO_ROOT="$REPO_ROOT"
export NOVA5_CALIBRATION_ROOT="${paths[0]}"
export NOVA5_CAMERA_PYTHON="${paths[1]}"
export NOVA5_RUNTIME_PYTHON="${paths[2]}"
"$NOVA5_RUNTIME_PYTHON" -c 'import numpy, cv2, scipy, grpc, grpc_tools' || {
  echo 'Runtime dependencies are missing. See the runtime_python requirements in README.md.' >&2
  exit 1
}
: "${VLM_BASE_URL:?Set VLM_BASE_URL to the OpenAI-compatible API base URL}"
: "${VLM_API_KEY:?Set VLM_API_KEY for the model API}"
export VLM_MODEL=gpt-6-astra
export VLM_REASONING_EFFORT=high
export ROBONIX_HOME="${ROBONIX_HOME:-$REPO_ROOT/.artifacts/rbnx-config}"
export PATH="$REPO_ROOT/target/debug:$(dirname "${paths[3]}"):$PATH"
cd "$REPO_ROOT"
cargo build -p robonix-cli -p robonix-pilot -p robonix-atlas
rbnx setup "$REPO_ROOT"
cd "$EXAMPLE_ROOT"
exec rbnx boot --no-update-check
