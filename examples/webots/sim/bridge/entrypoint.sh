#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Sim ENVIRONMENT only — Webots + eaios_webots controller. Nav2 lives in
# the tiago_nav2 service package (started by `rbnx boot`); robonix
# drivers (tiago_chassis / tiago_camera / tiago_lidar) live in their
# respective primitive packages and are exec'd into THIS container by
# `rbnx boot` via `docker exec`.
#
# Display backend is chosen by WEBOTS_HEADLESS_MODE:
#   unset / host    use the host DISPLAY bind-mounted in via /tmp/.X11-unix
#                   (legacy behaviour — local workstation with an X server)
#   nvidia          start an NVIDIA-backed Xorg on :48 inside the container,
#                   bound to the GPU with the most free memory (avoids
#                   disturbing peers on a shared multi-GPU box)
#   xvfb            start Xvfb on :99 (software llvmpipe — slow but
#                   needs no GPU, useful for CI / quick smoke)
#   auto            nvidia if /dev/nvidia0 is present, else xvfb
#
# Display :48 is intentionally outside the host's typical X allocator
# range (:0–:12 physical + :1001–:1099 xrdp) so the X socket that leaks
# into the bind-mounted /tmp/.X11-unix won't collide with any host user.
#
# When WEBOTS_STREAM=1, Webots keeps its raw stream on :1234 while a
# configurable proxy and viewer expose the browser-facing endpoints.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash
set -u

if [ "$#" -gt 0 ]; then
  exec "$@"
fi

# Display number is overridable via ROBONIX_SIM_XDISPLAY so two sim containers
# can run on one host without colliding on the same Xorg socket / logfile (a CI
# runner alongside an interactive user). Default :48 preserves single-tenant
# behaviour. XNUM is the bare number used for the /tmp/.X11-unix/X<n> socket.
NVIDIA_DISPLAY="${ROBONIX_SIM_XDISPLAY:-:48}"
XNUM="${NVIDIA_DISPLAY#:}"
ZENOH_ROUTER_PID=""
_webots_launch_pid=""
_viewer_pid=""
_stream_proxy_pid=""
STREAM_READY_FILE="/tmp/webots-stream-ready"

cleanup() {
  rm -f "$STREAM_READY_FILE"
  for pid in "${_stream_proxy_pid:-}" "${_viewer_pid:-}"; do
    if [ -n "$pid" ]; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  if [ -n "${_webots_launch_pid:-}" ]; then
    kill -TERM "${_webots_launch_pid}" 2>/dev/null || true
  fi
  if [ -n "${ZENOH_ROUTER_PID:-}" ]; then
    kill -TERM "${ZENOH_ROUTER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Start both browser-stream helpers and fail unless their configured endpoints
# become reachable while the exact child processes remain alive.
start_stream_helpers() {
  local viewer_port="${WEBOTS_VIEWER_PORT:-8080}"
  local stream_port="${WEBOTS_FILTER_PORT:-1235}"

  rm -f "$STREAM_READY_FILE"
  python3 /viewer_server.py >/tmp/viewer-http.log 2>&1 &
  _viewer_pid=$!
  python3 /webots_stream_proxy.py >/tmp/webots-stream-proxy.log 2>&1 &
  _stream_proxy_pid=$!

  if ! python3 /streaming_healthcheck.py \
      --viewer-port "$viewer_port" \
      --stream-port "$stream_port" \
      --viewer-pid "$_viewer_pid" \
      --proxy-pid "$_stream_proxy_pid" \
      --timeout "${WEBOTS_HELPER_READY_TIMEOUT:-10}"; then
    echo "[entrypoint] browser-stream helpers failed to become ready" >&2
    tail -80 /tmp/viewer-http.log /tmp/webots-stream-proxy.log 2>&1 || true
    return 1
  fi
  echo "[entrypoint] viewer HTTP on :${viewer_port}, optimized WebSocket on :${stream_port}, raw Webots stream on :1234"
}

# Wait for Webots and both required helpers. A helper exit is always fatal;
# Webots' own exit status is preserved when the launch process finishes first.
supervise_required_processes() {
  local status=0
  local required=("$_webots_launch_pid")
  if [ "${WEBOTS_STREAM:-0}" = "1" ]; then
    required+=("$_viewer_pid" "$_stream_proxy_pid")
  fi

  wait -n "${required[@]}" || status=$?
  if [ "${WEBOTS_STREAM:-0}" = "1" ]; then
    if ! kill -0 "$_viewer_pid" 2>/dev/null; then
      echo "[entrypoint] viewer helper exited unexpectedly; last 80 log lines:" >&2
      tail -80 /tmp/viewer-http.log 2>&1 || true
      return 1
    fi
    if ! kill -0 "$_stream_proxy_pid" 2>/dev/null; then
      echo "[entrypoint] stream proxy exited unexpectedly; last 80 log lines:" >&2
      tail -80 /tmp/webots-stream-proxy.log 2>&1 || true
      return 1
    fi
  fi
  if ! kill -0 "$_webots_launch_pid" 2>/dev/null; then
    return "$status"
  fi
  echo "[entrypoint] a required process exited unexpectedly" >&2
  return 1
}

start_zenoh_router() {
  if [ "${RMW_IMPLEMENTATION:-}" != "rmw_zenoh_cpp" ]; then
    return 0
  fi
  local router_bin="/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd"
  if [ ! -x "$router_bin" ]; then
    echo "[entrypoint] rmw_zenohd not found at $router_bin"
    return 1
  fi
  export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-20}"
  "$router_bin" >/tmp/rmw_zenohd.log 2>&1 &
  ZENOH_ROUTER_PID=$!
  echo "[entrypoint] rmw_zenohd pid=${ZENOH_ROUTER_PID}"
  local i
  for i in $(seq 1 20); do
    if ! kill -0 "$ZENOH_ROUTER_PID" 2>/dev/null; then
      echo "[entrypoint] rmw_zenohd exited early; last 80 lines:"
      tail -80 /tmp/rmw_zenohd.log 2>&1 || true
      return 1
    fi
    if python3 - <<PY >/dev/null 2>&1
import socket
with socket.create_connection(("127.0.0.1", 7447), timeout=0.2):
    pass
PY
    then
      echo "[entrypoint] rmw_zenohd listening on tcp/127.0.0.1:7447"
      return 0
    fi
    sleep 0.25
  done
  echo "[entrypoint] rmw_zenohd did not listen on :7447; last 80 lines:"
  tail -80 /tmp/rmw_zenohd.log 2>&1 || true
  return 1
}

start_nvidia_xorg() {
  # Compose limits visibility to the host-selected GPU. Pick from that visible
  # set and translate its PCI BusID into the Xorg ServerLayout form.
  local pick gpu_idx free_mib busid_full bus_hex_full bus_hex seg dev_str func busid
  pick=$(LC_ALL=C nvidia-smi --query-gpu=index,memory.free,pci.bus_id \
                    --format=csv,noheader,nounits 2>/dev/null \
         | sort -t',' -k2,2nr | head -1)
  if [ -z "$pick" ]; then
    echo "[entrypoint] nvidia-smi returned no GPUs"
    return 1
  fi
  gpu_idx=$(echo "$pick"  | awk -F',' '{gsub(/ /,"",$1); print $1}')
  free_mib=$(echo "$pick" | awk -F',' '{gsub(/ /,"",$2); print $2}')
  busid_full=$(echo "$pick" | awk -F',' '{gsub(/ /,"",$3); print $3}')
  bus_hex_full=${busid_full#*:}
  bus_hex=${bus_hex_full%%:*}
  seg=${bus_hex_full#*:}
  dev_str=${seg%.*}
  func=${seg#*.}
  busid="PCI:$((16#$bus_hex)):$((10#$dev_str)):$func"
  echo "[entrypoint] picked GPU $gpu_idx (free=${free_mib} MiB, $busid_full) -> Xorg BusID=$busid"

  cat >/tmp/xorg-nvidia.conf <<XCONF
Section "ServerLayout"
  Identifier "L0"
  Screen 0 "S0"
EndSection
Section "Device"
  Identifier "D0"
  Driver "nvidia"
  BusID  "$busid"
EndSection
Section "Screen"
  Identifier "S0"
  Device "D0"
  Option "AllowEmptyInitialConfiguration" "true"
  Option "UseDisplayDevice" "none"
  SubSection "Display"
    Virtual 1920 1080
    Depth 24
  EndSubSection
EndSection
XCONF

  Xorg "$NVIDIA_DISPLAY" -config /tmp/xorg-nvidia.conf \
       -noreset -novtswitch -sharevts -nolisten tcp \
       -logfile "/tmp/Xorg.${XNUM}.log" &
  local i
  for i in $(seq 1 30); do
    [ -S "/tmp/.X11-unix/X${XNUM}" ] && break
    sleep 0.5
  done
  if ! [ -S "/tmp/.X11-unix/X${XNUM}" ]; then
    echo "[entrypoint] Xorg ${NVIDIA_DISPLAY} failed; last 40 lines of /tmp/Xorg.${XNUM}.log:"
    tail -40 "/tmp/Xorg.${XNUM}.log" 2>&1 || true
    return 1
  fi
  export DISPLAY=$NVIDIA_DISPLAY
  local renderer
  renderer=$(glxinfo -B 2>/dev/null | awk -F'string: ' '/OpenGL renderer/ {print $2; exit}')
  echo "[entrypoint] Xorg ${NVIDIA_DISPLAY} up, renderer=$renderer"
  if ! echo "$renderer" | grep -qi nvidia; then
    echo "[entrypoint] WARN: renderer is not NVIDIA — webots will still be slow"
    return 1
  fi
  return 0
}

start_xvfb() {
  Xvfb "$NVIDIA_DISPLAY" -screen 0 1920x1080x24 -nolisten tcp &
  export DISPLAY="$NVIDIA_DISPLAY"
  sleep 1
  echo "[entrypoint] Xvfb ${NVIDIA_DISPLAY} (CPU render)"
}

prepare_office_webots_seed() {
  local seed_id expected_sha url mirror fetch_url cache_root marker
  local archive stage count actual_sha entry
  seed_id="webots-office-seed-v3"
  expected_sha="f98f3e27a58ca432b5faced2f4d2e5d7fd12dd1992202a59ee55332a510d5110"
  url="${ROBONIX_WEBOTS_SEED_URL:-https://github.com/syswonder/robonix-assets/releases/download/${seed_id}/${seed_id}.tar.gz}"
  mirror="${ROBONIX_WEBOTS_SEED_MIRROR-https://ghfast.top/}"
  cache_root="${ROBONIX_WEBOTS_CACHE_ROOT:-/root/.cache/Cyberbotics/Webots}"
  marker="${cache_root}/.${seed_id}-${expected_sha}.ok"

  if [ -f "$marker" ]; then
    echo "[entrypoint] Webots office seed already present (${seed_id})"
    return 0
  fi

  fetch_url="$url"
  if [ -n "$mirror" ]; then
    case "$url" in
      https://github.com/*)
        fetch_url="${mirror%/}/${url}"
        ;;
    esac
  fi

  archive=$(mktemp "/tmp/${seed_id}.XXXXXX.tar.gz")
  stage=$(mktemp -d "/tmp/${seed_id}.XXXXXX")
  echo "[entrypoint] downloading Webots office seed: ${fetch_url}"
  if ! wget --tries=3 --timeout=30 --progress=dot:giga -O "$archive" "$fetch_url"; then
    rm -rf "$archive" "$stage"
    echo "[entrypoint] failed to download Webots office seed" >&2
    return 1
  fi

  actual_sha=$(sha256sum "$archive" | awk '{print $1}')
  if [ "$actual_sha" != "$expected_sha" ]; then
    rm -rf "$archive" "$stage"
    echo "[entrypoint] Webots office seed checksum mismatch: expected=${expected_sha} actual=${actual_sha}" >&2
    return 1
  fi

  while IFS= read -r entry; do
    case "$entry" in
      assets|assets/|assets/*) ;;
      *)
        rm -rf "$archive" "$stage"
        echo "[entrypoint] unsafe Webots office seed path: ${entry}" >&2
        return 1
        ;;
    esac
    case "$entry" in
      ""|/*|*//*|../*|*/../*|*/..)
        rm -rf "$archive" "$stage"
        echo "[entrypoint] unsafe Webots office seed path: ${entry}" >&2
        return 1
        ;;
    esac
  done < <(tar -tzf "$archive")

  tar -xzf "$archive" -C "$stage"
  count=$(find "$stage/assets" -maxdepth 1 -type f | wc -l)
  if [ "$count" -ne 192 ]; then
    rm -rf "$archive" "$stage"
    echo "[entrypoint] Webots office seed file count mismatch: expected=192 actual=${count}" >&2
    return 1
  fi

  mkdir -p "$cache_root/assets"
  cp -a "$stage/assets/." "$cache_root/assets/"
  touch "$marker"
  rm -rf "$archive" "$stage"
  echo "[entrypoint] Webots office seed ready: ${count} verified files"
}

prepare_full_webots_assets() {
  if [ "${ROBONIX_WEBOTS_DOWNLOAD_ALL_ASSETS:-0}" != "1" ]; then
    return 0
  fi

  local version url mirror fetch_url cache_dir marker tmp_zip
  version="${ROBONIX_WEBOTS_ASSETS_VERSION:-R2025a}"
  url="${ROBONIX_WEBOTS_ASSETS_URL:-https://github.com/cyberbotics/webots/releases/download/${version}/assets-${version}.zip}"
  mirror="${ROBONIX_WEBOTS_ASSETS_MIRROR:-https://ghfast.top/}"
  fetch_url="$url"
  if [ -n "$mirror" ]; then
    case "$url" in
      https://github.com/*)
        fetch_url="${mirror%/}/${url}"
        ;;
    esac
  fi

  cache_dir="${ROBONIX_WEBOTS_ASSET_CACHE_DIR:-/root/.cache/Cyberbotics/Webots/assets}"
  marker="${cache_dir}/.robonix-full-assets-${version}.ok"
  if [ -f "$marker" ]; then
    echo "[entrypoint] Webots full asset cache already present (${version})"
    return 0
  fi

  mkdir -p "$cache_dir"
  tmp_zip="/tmp/webots-assets-${version}.zip"
  echo "[entrypoint] downloading Webots full asset library: ${fetch_url}"
  wget -S --progress=dot:giga -O "$tmp_zip" "$fetch_url"
  echo "[entrypoint] extracting Webots full asset library to ${cache_dir}"
  unzip -q -o "$tmp_zip" -d "$cache_dir"
  rm -f "$tmp_zip"
  touch "$marker"
  echo "[entrypoint] Webots full asset library ready: $(find "$cache_dir" -maxdepth 1 -type f | wc -l) cached files"
}

case "${WEBOTS_HEADLESS_MODE:-host}" in
  host)   : ;;                                # legacy: keep $DISPLAY from compose env
  nvidia) start_nvidia_xorg || exit 1 ;;
  xvfb)   start_xvfb ;;
  auto)
    if [ -e /dev/nvidia0 ] && command -v Xorg >/dev/null 2>&1 && start_nvidia_xorg; then
      :
    else
      echo "[entrypoint] auto: falling back to Xvfb :99"
      start_xvfb
    fi
    ;;
  *) echo "[entrypoint] unknown WEBOTS_HEADLESS_MODE=$WEBOTS_HEADLESS_MODE"; exit 2 ;;
esac

start_zenoh_router
prepare_office_webots_seed
prepare_full_webots_assets

if [ "${WEBOTS_STREAM:-0}" = "1" ]; then
  start_stream_helpers
fi

WEBOTS_WARMUP_SEC="${WEBOTS_WARMUP_SEC:-25}"

ROBONIX_WEBOTS_WORLD="${ROBONIX_WEBOTS_WORLD:-office.wbt}"
ROBONIX_TIAGO_VARIANT="${ROBONIX_TIAGO_VARIANT:-lite}"
ROBONIX_WEBOTS_ROBOT="${ROBONIX_WEBOTS_ROBOT:-}"

case "$ROBONIX_TIAGO_VARIANT" in
  lite)
    ROBONIX_WEBOTS_ROBOT="${ROBONIX_WEBOTS_ROBOT:-tiago_webots.urdf}"
    ;;
  full)
    ROBONIX_WEBOTS_ROBOT="${ROBONIX_WEBOTS_ROBOT:-tiago_full_webots.urdf}"
    ;;
  *)
    echo "[entrypoint] unsupported ROBONIX_TIAGO_VARIANT=$ROBONIX_TIAGO_VARIANT (choose lite or full)" >&2
    exit 2
    ;;
esac

PACKAGE_SHARE="$(ros2 pkg prefix eaios_webots)/share/eaios_webots"
if [[ "$ROBONIX_WEBOTS_WORLD" = /* ]]; then
  WORLD_SOURCE="$ROBONIX_WEBOTS_WORLD"
else
  WORLD_SOURCE="$PACKAGE_SHARE/worlds/$ROBONIX_WEBOTS_WORLD"
fi
export ROBONIX_WEBOTS_WORLD_PATH
ROBONIX_WEBOTS_WORLD_PATH="$(
  python3 -m eaios_webots.world_variant \
    "$WORLD_SOURCE" \
    --variant "$ROBONIX_TIAGO_VARIANT"
)"

echo "[entrypoint] Webots world: ${ROBONIX_WEBOTS_WORLD}"
echo "[entrypoint] TIAGo variant: ${ROBONIX_TIAGO_VARIANT}"
echo "[entrypoint] materialized world: ${ROBONIX_WEBOTS_WORLD_PATH}"
echo "[entrypoint] robot URDF: ${ROBONIX_WEBOTS_ROBOT}"

ros2 launch eaios_webots robot_launch.py \
  use_sim_time:=true \
  world:="${ROBONIX_WEBOTS_WORLD}" \
  robot:="${ROBONIX_WEBOTS_ROBOT}" &

_webots_launch_pid=$!
echo "[entrypoint] eaios_webots pid=${_webots_launch_pid}"
sleep "${WEBOTS_WARMUP_SEC}"

if ! kill -0 "$_webots_launch_pid" 2>/dev/null; then
  echo "[entrypoint] Webots exited during warmup" >&2
  exit 1
fi

if [ "${WEBOTS_STREAM:-0}" = "1" ]; then
  if ! python3 /streaming_healthcheck.py \
      --viewer-port "${WEBOTS_VIEWER_PORT:-8080}" \
      --stream-port "${WEBOTS_FILTER_PORT:-1235}" \
      --viewer-pid "$_viewer_pid" \
      --proxy-pid "$_stream_proxy_pid"; then
    echo "[entrypoint] browser-stream helper failed during Webots warmup" >&2
    exit 1
  fi
  touch "$STREAM_READY_FILE"
fi

# Stay alive so `docker exec` from rbnx-driven driver packages can land inside
# this container, while treating every browser-stream helper as required.
supervise_required_processes
