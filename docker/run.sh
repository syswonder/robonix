#!/usr/bin/env bash
set -e

# set your workspace target here
WORKSPACE_TARGET=..

if [ -f .env ]; then
    echo "[*] Loading environment variables from .env..."
    export $(cat .env | grep -v '^#' | xargs)
fi

IMAGE_NAME=tailscale_ros
CONTAINER_NAME=tailscale_ros_dev

if [ "$1" == "-b" ]; then
    echo "[*] Building Docker image..."
    docker build -t $IMAGE_NAME .
fi

if ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
    echo "[*] Building Docker image..."
    docker build -t $IMAGE_NAME .
fi

if [ "$1" == "-d" ]; then
    echo "[*] Deleting Docker container..."
    docker rm -f $CONTAINER_NAME
fi

if [ "$1" == "-c" ]; then
    echo "[*] To clear Tailscale state, run: sudo rm -rf ./lib/tailscale"
    echo "[*] Then run ./run.sh again"
    exit 0
fi

GPU_ARGS=""
if command -v nvidia-smi &> /dev/null; then
    echo "[*] GPU detected, enabling GPU support..."
    GPU_ARGS="--gpus all --runtime nvidia"
fi

# Check if user wants to use host network
# Use host network so container has full network access
NETWORK_ARGS="--net host"
# No SYSCTL_ARGS needed for host network (not allowed in host network namespace)

# Stop host tailscaled if running (to avoid TUN device conflicts)
if pgrep tailscaled > /dev/null; then
    echo "[*] Stopping host tailscaled to avoid TUN device conflicts..."
    sudo systemctl stop tailscaled 2>/dev/null || sudo pkill tailscaled || true
    sleep 2
fi

# Set up X11 forwarding permissions
echo "[*] Setting up X11 forwarding..."
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi
if [ -z "$XAUTHORITY" ]; then
    export XAUTHORITY=$HOME/.Xauthority
fi
echo "[*] DISPLAY=$DISPLAY"
echo "[*] XAUTHORITY=$XAUTHORITY"
xhost +local:docker 2>/dev/null || xhost + 2>/dev/null || echo "[*] Warning: Could not set xhost permissions"

docker run -it --rm \
  --name $CONTAINER_NAME \
  --hostname docker-ub \
  --cap-add NET_ADMIN \
  --cap-add NET_RAW \
  --cap-add SYS_MODULE \
  $NETWORK_ARGS \
  $GPU_ARGS \
  -v ./shared:/root/shared \
  -v $WORKSPACE_TARGET:/root/workspace \
  -v ./tailscale:/var/lib/tailscale \
  -v ./docker-entrypoint.sh:/docker-entrypoint.sh \
  -v /dev/net/tun:/dev/net/tun \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR" \
  -v /dev/dri:/dev/dri \
  -e TS_AUTHKEY="${DOCKERKEY_PERM}" \
  -e TS_ROUTES="10.0.0.0/8" \
  -e TS_USERSPACE=0 \
  -e TS_STATE_DIR=/var/lib/tailscale \
  -e TS_HOSTNAME=docker1 \
  -e DISPLAY=${DISPLAY:-:0} \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=${XAUTHORITY} \
  -e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
  -e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all} \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  $IMAGE_NAME \
  bash /docker-entrypoint.sh