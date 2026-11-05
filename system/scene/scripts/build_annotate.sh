#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
PY="$PKG/rbnx-build/annotate-venv/bin/python"
if [[ ! -x "$PY" ]]; then
  uv venv --system-site-packages --python "${ROS_PYTHON:-/usr/bin/python3}" "$PKG/rbnx-build/annotate-venv"
fi
uv pip install --python "$PY" -e "$(rbnx path robonix-api)" \
  'grpcio-tools==1.76.0' 'numpy<2' scipy Pillow httpx uvicorn 'starlette<1' 'setuptools<81' 'packaging>=24'
rbnx codegen -p "$PKG" --mcp --ros2 --python "$PY"
set +u
source "${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
set -u
cd "$PKG/rbnx-build/codegen/ros2_idl"
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build --packages-select map \
  --cmake-args "-DPython3_EXECUTABLE=$PY" "-DPYTHON_EXECUTABLE=$PY"
touch "$PKG/rbnx-build/.rbnx-built"
