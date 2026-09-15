#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -eo pipefail
export UV_INDEX_URL="${UV_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
mkdir -p rbnx-build/data
uv venv --python /usr/bin/python3 --system-site-packages --allow-existing rbnx-build/venv
VIRTUAL_ENV="$PKG/rbnx-build/venv" uv sync --active --no-managed-python
# Codegen must use this checkout, independently of the user's global rbnx setup.
mkdir -p rbnx-build/cli
ROOT="$(cd "$PKG/../../.." && pwd)"
printf 'package_storage_path: "%s/rbnx-build/packages"\nrobonix_source_path: "%s"\n' "$PKG" "$ROOT" > rbnx-build/cli/config.yaml
ROBONIX_HOME="$PKG/rbnx-build/cli" RBNX_CODEGEN_PYTHON="$PKG/rbnx-build/venv/bin/python" PATH="$PKG/rbnx-build/venv/bin:$PATH" rbnx codegen -p "$PKG" --mcp
PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}" \
  rbnx-build/venv/bin/python -c 'import rclpy, PIL, verifier_mcp; import vlm_verifier.service'
