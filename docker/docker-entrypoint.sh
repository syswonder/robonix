#!/bin/bash
set -e

mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
export XDG_RUNTIME_DIR=/tmp/runtime-root

# Set up OSMesa for headless OpenGL rendering
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/osmesa:${LD_LIBRARY_PATH}
export OSMESA_LIB=/usr/lib/x86_64-linux-gnu/libOSMesa.so

echo "[*] Starting tailscaled..."
# Kill any existing tailscaled process in container
killall tailscaled 2>/dev/null || true
sleep 1

# Check if tailscale0 exists and remove it
if ip link show tailscale0 &>/dev/null; then
    echo "[*] Removing existing tailscale0 interface..."
    ip link set tailscale0 down 2>/dev/null || true
    ip link delete tailscale0 2>/dev/null || true
    sleep 1
fi

nohup tailscaled >/var/log/tailscaled.log 2>&1 &
sleep 2

echo "[*] Checking tailscaled status..."
tailscale status -json >/dev/null 2>&1 && echo "  OK" || echo "  Starting..."

echo "[*] Checking auth key..."
if [ -z "$TS_AUTHKEY" ]; then
  echo "[!] Error: TS_AUTHKEY is empty"
  echo "[!] Please set DOCKERKEY_PERM environment variable"
  exit 1
else
  echo "[*] Auth key is set (length: ${#TS_AUTHKEY})"
fi

echo "[*] Testing network connectivity..."
echo "[*] Checking internet connectivity..."
if ping -c 1 -W 2 8.8.8.8 >/dev/null 2>&1; then
  echo "[*] Network connectivity OK"
else
  echo "[!] Warning: Network connectivity test failed"
  ip -4 -o addr show | awk '{print $2}' | tr '\n' ' ' || true
  echo ""
fi

echo "[*] Checking connectivity to Tailscale control plane..."
if curl -s -m 5 https://login.tailscale.com >/dev/null 2>&1; then
  echo "[*] Can reach Tailscale login server"
else
  echo "[!] Cannot reach Tailscale login server"
  ip route | head -3 | tr '\n' '; ' || true
  echo ""
fi

echo "[*] Bringing up Tailscale..."
echo "[*] This may take a moment (connecting to Tailscale servers)..."

# Try with extended timeout and verbose output
tailscale up --authkey "$TS_AUTHKEY" --hostname "$TS_HOSTNAME" --accept-routes 2>&1 || {
  echo "[!] Failed to bring up Tailscale"
  echo "[*] ========== Debugging Info =========="
  echo "[*] Recent Tailscale logs:"
  tail -20 /var/log/tailscaled.log
  echo ""
  ip -4 -o addr show | awk '{print $2}' | tr '\n' ' '
  echo ""
  ip route | head -5 | tr '\n' '; '
  echo ""
  echo "[*] ====================================="
  exit 1
}

tailscale_ip=$(hostname -I | tr ' ' '\n' | grep '^100\.' | head -1)
if [ -n "$tailscale_ip" ]; then
  tailscale_status=$(tailscale status)
  tailscale_name=$(echo "$tailscale_status" | grep "^${tailscale_ip}" | awk '{print $2}')
  tailscale_login=$(echo "$tailscale_status" | grep "^${tailscale_ip}" | awk '{print $3}')
  echo "[*] Tailscale connected, this machine IP is: ${tailscale_ip}, name is ${tailscale_name} (${tailscale_login})"
else
  echo "[*] Tailscale connected, but no tailscale IP found"
fi

# some post apt install
apt install -qq -y iputils-ping screenfetch >/dev/null 2>&1

# source the setup.bash
source /opt/ros/humble/setup.bash

echo -e "[*] \033[1mWelcome to robonix docker environment!\033[0m Distro is: \033[33m$(lsb_release -ds 2>/dev/null || echo "Linux")\033[0m with ROS2 \033[33m$(echo $ROS_DISTRO)\033[0m"
exec bash

