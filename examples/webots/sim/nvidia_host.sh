#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# shellcheck shell=bash

# Resolve the backend that Webots will use after the stream overlay applies its
# default. The explicit WEBOTS_HEADLESS_MODE value always wins.
robonix_effective_webots_headless_mode() {
  local configured_mode="${1:-}"
  local stream_enabled="${2:-0}"
  if [[ -z "$configured_mode" ]]; then
    if [[ "$stream_enabled" == "1" ]]; then
      configured_mode="auto"
    else
      configured_mode="host"
    fi
  fi
  case "$configured_mode" in
    host|nvidia|xvfb|auto) printf '%s\n' "$configured_mode" ;;
    *)
      echo "[sim/nvidia] unsupported WEBOTS_HEADLESS_MODE=$configured_mode (choose host, nvidia, xvfb, or auto)" >&2
      return 2
      ;;
  esac
}

# Select one physical GPU, honoring an explicit index or UUID. Automatic
# selection uses free memory but does not inspect or depend on the GPU model.
robonix_select_nvidia_gpu() {
  local requested_id="${1:-}"
  local inventory selected_id identity
  if [[ -z "$requested_id" ]]; then
    if ! inventory=$(LC_ALL=C nvidia-smi \
      --query-gpu=index,memory.free \
      --format=csv,noheader,nounits); then
      echo "[sim/nvidia] failed to query NVIDIA GPU memory" >&2
      return 1
    fi
    selected_id=$(awk -F',' '
      {
        gsub(/[[:space:]]/, "", $1);
        gsub(/[[:space:]]/, "", $2);
        if (!seen || ($2 + 0) > most_free) {
          selected = $1;
          most_free = $2 + 0;
          seen = 1;
        }
      }
      END { if (seen) print selected }
    ' <<<"$inventory")
    if [[ -z "$selected_id" ]]; then
      echo "[sim/nvidia] nvidia-smi returned no selectable GPUs" >&2
      return 1
    fi
  else
    selected_id="$requested_id"
  fi

  if ! identity=$(LC_ALL=C nvidia-smi --id="$selected_id" \
    --query-gpu=index,uuid,name,driver_version \
    --format=csv,noheader); then
    echo "[sim/nvidia] invalid or unavailable ROBONIX_GPU_ID=$selected_id" >&2
    return 1
  fi
  export ROBONIX_GPU_ID="$selected_id"
  echo "[sim/nvidia] selected GPU $ROBONIX_GPU_ID: $identity"
}

# Print the canonical path of the first regular file. Symlinks are accepted
# only when their target exists and are resolved before the path is exported.
robonix_first_regular_file() {
  local candidate
  for candidate in "$@"; do
    if [[ -f "$candidate" ]]; then
      readlink -f "$candidate"
      return 0
    fi
  done
  return 1
}

# Discover the NVIDIA Xorg driver and GLX server library for the selected GPU's
# installed driver version. Explicit paths win; distro and architecture paths
# are searched next, with a bounded filesystem scan as the final fallback.
robonix_resolve_nvidia_xorg_modules() {
  local gpu_id="${1:-}"
  local driver_version multiarch driver_path glx_path driver_dir candidate root
  local -a roots candidates exact_glx_candidates fallback_glx_candidates
  if [[ -z "$gpu_id" ]]; then
    echo "[sim/nvidia] a GPU index or UUID is required to resolve Xorg modules" >&2
    return 1
  fi
  if ! driver_version=$(LC_ALL=C nvidia-smi --id="$gpu_id" \
    --query-gpu=driver_version --format=csv,noheader); then
    echo "[sim/nvidia] cannot query the driver version for GPU $gpu_id" >&2
    return 1
  fi
  driver_version=${driver_version//[[:space:]]/}
  if [[ -z "$driver_version" ]]; then
    echo "[sim/nvidia] GPU $gpu_id reported an empty driver version" >&2
    return 1
  fi

  IFS=':' read -r -a roots <<<"${ROBONIX_NVIDIA_LIBRARY_ROOTS:-/usr/lib:/usr/lib64}"
  multiarch=""
  if command -v dpkg-architecture >/dev/null 2>&1; then
    multiarch=$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || true)
  elif command -v gcc >/dev/null 2>&1; then
    multiarch=$(gcc -print-multiarch 2>/dev/null || true)
  fi
  if [[ -z "${ROBONIX_NVIDIA_LIBRARY_ROOTS:-}" && -n "$multiarch" && -d "/usr/lib/$multiarch" ]]; then
    roots+=("/usr/lib/$multiarch")
  fi

  driver_path="${ROBONIX_NVIDIA_XORG_DRIVER:-}"
  if [[ -z "$driver_path" ]]; then
    candidates=()
    for root in "${roots[@]}"; do
      candidates+=(
        "$root/nvidia/xorg/nvidia_drv.so"
        "$root/xorg/modules/drivers/nvidia_drv.so"
      )
    done
    driver_path=$(robonix_first_regular_file "${candidates[@]}" || true)
    if [[ -z "$driver_path" ]]; then
      for root in "${roots[@]}"; do
        [[ -d "$root" ]] || continue
        candidate=$(find "$root" -type f -name nvidia_drv.so -print -quit 2>/dev/null || true)
        if [[ -n "$candidate" ]]; then
          driver_path=$(readlink -f "$candidate")
          break
        fi
      done
    fi
  else
    driver_path=$(robonix_first_regular_file "$driver_path" || true)
  fi
  if [[ -z "$driver_path" ]]; then
    echo "[sim/nvidia] NVIDIA Xorg driver not found; set ROBONIX_NVIDIA_XORG_DRIVER" >&2
    return 1
  fi

  glx_path="${ROBONIX_NVIDIA_GLX_SERVER:-}"
  if [[ -z "$glx_path" ]]; then
    driver_dir=$(dirname "$driver_path")
    exact_glx_candidates=("$driver_dir/libglxserver_nvidia.so.$driver_version")
    fallback_glx_candidates=("$driver_dir/libglxserver_nvidia.so")
    for root in "${roots[@]}"; do
      exact_glx_candidates+=(
        "$root/nvidia/xorg/libglxserver_nvidia.so.$driver_version"
        "$root/xorg/modules/extensions/libglxserver_nvidia.so.$driver_version"
      )
      fallback_glx_candidates+=(
        "$root/nvidia/xorg/libglxserver_nvidia.so"
        "$root/xorg/modules/extensions/libglxserver_nvidia.so"
      )
    done
    glx_path=$(robonix_first_regular_file "${exact_glx_candidates[@]}" || true)
    if [[ -z "$glx_path" ]]; then
      glx_path=$(robonix_first_regular_file "${fallback_glx_candidates[@]}" || true)
    fi
    if [[ -z "$glx_path" ]]; then
      for root in "${roots[@]}"; do
        [[ -d "$root" ]] || continue
        candidate=$(find "$root" -type f \
          -name "libglxserver_nvidia.so.$driver_version" -print -quit 2>/dev/null || true)
        if [[ -n "$candidate" ]]; then
          glx_path=$(readlink -f "$candidate")
          break
        fi
      done
    fi
  else
    glx_path=$(robonix_first_regular_file "$glx_path" || true)
  fi
  if [[ -z "$glx_path" ]]; then
    echo "[sim/nvidia] NVIDIA GLX server library not found; set ROBONIX_NVIDIA_GLX_SERVER" >&2
    return 1
  fi
  case "$(basename "$glx_path")" in
    libglxserver_nvidia.so|"libglxserver_nvidia.so.$driver_version") ;;
    libglxserver_nvidia.so.*)
      echo "[sim/nvidia] GLX library $glx_path does not match driver $driver_version" >&2
      return 1
      ;;
  esac

  export ROBONIX_NVIDIA_XORG_DRIVER="$driver_path"
  export ROBONIX_NVIDIA_GLX_SERVER="$glx_path"
  echo "[sim/nvidia] NVIDIA Xorg driver: $ROBONIX_NVIDIA_XORG_DRIVER"
  echo "[sim/nvidia] NVIDIA GLX server: $ROBONIX_NVIDIA_GLX_SERVER"
}
