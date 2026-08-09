#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/nvidia_host.sh"

TMP_ROOT=$(mktemp -d)
trap 'rm -rf "$TMP_ROOT"' EXIT
mkdir -p "$TMP_ROOT/bin" "$TMP_ROOT/lib/nvidia/xorg"

cat >"$TMP_ROOT/bin/nvidia-smi" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
args=" $* "
if [[ "$args" == *" --query-gpu=index,memory.free "* ]]; then
  printf '0, 1024\n1, 8192\n'
elif [[ "$args" == *" --id=invalid "* ]]; then
  exit 1
elif [[ "$args" == *" --query-gpu=index,uuid,name,driver_version "* ]]; then
  printf '1, GPU-test, Test GPU, 595.71.05\n'
elif [[ "$args" == *" --query-gpu=driver_version "* ]]; then
  printf '595.71.05\n'
else
  exit 2
fi
EOF
chmod +x "$TMP_ROOT/bin/nvidia-smi"

touch "$TMP_ROOT/lib/nvidia/xorg/nvidia_drv.so"
touch "$TMP_ROOT/lib/nvidia/xorg/libglxserver_nvidia.so.570.0"
touch "$TMP_ROOT/lib/nvidia/xorg/libglxserver_nvidia.so.595.71.05"
ln -s libglxserver_nvidia.so.570.0 \
  "$TMP_ROOT/lib/nvidia/xorg/libglxserver_nvidia.so"

export PATH="$TMP_ROOT/bin:$PATH"
export ROBONIX_NVIDIA_LIBRARY_ROOTS="$TMP_ROOT/lib"
unset ROBONIX_GPU_ID ROBONIX_NVIDIA_XORG_DRIVER ROBONIX_NVIDIA_GLX_SERVER

[[ "$(robonix_effective_webots_headless_mode "" 0)" == "host" ]]
[[ "$(robonix_effective_webots_headless_mode "" 1)" == "auto" ]]
[[ "$(robonix_effective_webots_headless_mode nvidia 0)" == "nvidia" ]]
if robonix_effective_webots_headless_mode invalid 0 >/dev/null 2>&1; then
  echo "invalid headless mode was accepted" >&2
  exit 1
fi

robonix_select_nvidia_gpu "" >/dev/null
[[ "$ROBONIX_GPU_ID" == "1" ]]
if robonix_select_nvidia_gpu invalid >/dev/null 2>&1; then
  echo "invalid GPU id was accepted" >&2
  exit 1
fi

robonix_resolve_nvidia_xorg_modules "$ROBONIX_GPU_ID" >/dev/null
[[ "$ROBONIX_NVIDIA_XORG_DRIVER" == "$TMP_ROOT/lib/nvidia/xorg/nvidia_drv.so" ]]
[[ "$ROBONIX_NVIDIA_GLX_SERVER" == "$TMP_ROOT/lib/nvidia/xorg/libglxserver_nvidia.so.595.71.05" ]]

rm "$TMP_ROOT/lib/nvidia/xorg/libglxserver_nvidia.so.595.71.05"
unset ROBONIX_NVIDIA_XORG_DRIVER ROBONIX_NVIDIA_GLX_SERVER
if robonix_resolve_nvidia_xorg_modules "$ROBONIX_GPU_ID" >/dev/null 2>&1; then
  echo "mismatched GLX library was accepted" >&2
  exit 1
fi

echo "nvidia host tests passed"
