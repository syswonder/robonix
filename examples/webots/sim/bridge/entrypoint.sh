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
# When WEBOTS_STREAM=1, the webots stream WebSocket is exposed on :1234
# and a tiny HTTP server on :8080 serves the streaming-viewer assets
# (Webots ships the viewer HTML at
# /usr/local/webots/resources/web/streaming_viewer). Browse to
# http://<server>:8080/ from any machine that can reach the host.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash
set -u

# Display number is overridable via ROBONIX_SIM_XDISPLAY so two sim containers
# can run on one host without colliding on the same Xorg socket / logfile (a CI
# runner alongside an interactive user). Default :48 preserves single-tenant
# behaviour. XNUM is the bare number used for the /tmp/.X11-unix/X<n> socket.
NVIDIA_DISPLAY="${ROBONIX_SIM_XDISPLAY:-:48}"
XNUM="${NVIDIA_DISPLAY#:}"
ZENOH_ROUTER_PID=""
_webots_launch_pid=""

cleanup() {
  if [ -n "${_webots_launch_pid:-}" ]; then
    kill -TERM "${_webots_launch_pid}" 2>/dev/null || true
  fi
  if [ -n "${ZENOH_ROUTER_PID:-}" ]; then
    kill -TERM "${ZENOH_ROUTER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

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
  # Pick the GPU with the most free memory and translate its PCI BusID
  # ("00000000:9D:00.0") into the Xorg ServerLayout form ("PCI:157:0:0").
  local pick gpu_idx free_mib busid_full bus_hex_full bus_hex seg dev_str func busid
  pick=$(nvidia-smi --query-gpu=index,memory.free,pci.bus_id \
                    --format=csv,noheader,nounits 2>/dev/null \
         | sort -t',' -k2 -nr | head -1)
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

if [ "${WEBOTS_STREAM:-0}" = "1" ]; then
  # Serve the streaming-viewer assets from a separate port; the viewer
  # HTML connects back to the WS server on :1234 (port hard-coded by
  # webots).
  (cd /usr/local/webots/resources/web/streaming_viewer \
     && python3 -m http.server 8080 --bind 0.0.0.0) \
       >/tmp/viewer-http.log 2>&1 &
  echo "[entrypoint] viewer HTTP on :8080  ws on :1234"
fi

start_zenoh_router

WEBOTS_WARMUP_SEC="${WEBOTS_WARMUP_SEC:-25}"

ROBONIX_WEBOTS_WORLD="${ROBONIX_WEBOTS_WORLD:-office.wbt}"
ROBONIX_WEBOTS_ROBOT="${ROBONIX_WEBOTS_ROBOT:-tiago_webots.urdf}"

echo "[entrypoint] Webots world: ${ROBONIX_WEBOTS_WORLD}"
echo "[entrypoint] robot URDF: ${ROBONIX_WEBOTS_ROBOT}"

ros2 launch eaios_webots robot_launch.py \
  use_sim_time:=true \
  world:="${ROBONIX_WEBOTS_WORLD}" \
  robot:="${ROBONIX_WEBOTS_ROBOT}" &

_webots_launch_pid=$!
echo "[entrypoint] eaios_webots pid=${_webots_launch_pid}"
sleep "${WEBOTS_WARMUP_SEC}"

# Stay alive so `docker exec` from rbnx-driven driver packages can land
# inside this container. Ctrl-C (compose down) propagates SIGTERM here
# which `wait` will then forward to the launch process.
wait ${_webots_launch_pid}
