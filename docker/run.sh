#!/usr/bin/env bash
set -e

# set your workspace target here
WORKSPACE_TARGET=..

if [ -f .env ]; then
    echo "[*] Loading environment variables from .env..."
    export $(cat .env | grep -v '^#' | xargs)
fi

# Default: use remote image from Docker Hub
USE_LOCAL=false
REMOTE_IMAGE="docker.io/enkerewpo/robonix_ros:latest"
LOCAL_IMAGE="robonix_ros"
CONTAINER_NAME=robonix_ros_dev

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--build)
            USE_LOCAL=true
            shift
            ;;
        -d|--delete)
            echo "[*] Deleting Docker container..."
            docker rm -f $CONTAINER_NAME 2>/dev/null || true
            exit 0
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -b, --build    Build and use local image (default: use remote image from Docker Hub)"
            echo "  -d, --delete   Delete existing container"
            echo "  -h, --help     Show this help message"
            echo ""
            echo "By default, pulls and uses: docker.io/enkerewpo/robonix_ros:latest"
            echo "Use -b to build and use local image instead"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Determine which image to use
if [ "$USE_LOCAL" = true ]; then
    IMAGE_NAME=$LOCAL_IMAGE
    echo "[*] Using local image: $IMAGE_NAME"
    
    # Build local image if it doesn't exist
    if ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
        echo "[*] Local image not found, building..."
        docker build -t $IMAGE_NAME .
    else
        echo "[*] Local image found, skipping build"
    fi
else
    IMAGE_NAME=$REMOTE_IMAGE
    echo "[*] Using remote image: $IMAGE_NAME"
    
    # Pull remote image
    echo "[*] Pulling remote image..."
    docker pull $IMAGE_NAME || {
        echo "[!] Failed to pull remote image. Falling back to local build..."
        IMAGE_NAME=$LOCAL_IMAGE
        if ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
            echo "[*] Building local image as fallback..."
            docker build -t $IMAGE_NAME .
        fi
    }
fi

GPU_ARGS=""
if command -v nvidia-smi &> /dev/null; then
    echo "[*] GPU detected, enabling GPU support..."
    GPU_ARGS="--gpus all --runtime nvidia"
fi

# Check if user wants to use host network
# Use host network so container has full network access
NETWORK_ARGS="--net host"

# Set up X11 forwarding permissions
echo "[*] Setting up X11 forwarding..."
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi
if [ -z "$XAUTHORITY" ]; then
    export XAUTHORITY=$HOME/.Xauthority
fi
if [ -z "$XDG_RUNTIME_DIR" ]; then
    export XDG_RUNTIME_DIR=/tmp
fi
echo "[*] DISPLAY=$DISPLAY"
echo "[*] XAUTHORITY=$XAUTHORITY"
echo "[*] XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
xhost +local:docker 2>/dev/null || xhost + 2>/dev/null || echo "[*] Warning: Could not set xhost permissions"

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[*] Container ${CONTAINER_NAME} already exists"
    
    # Check if container is running
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "[*] Container is already running, attaching..."
        docker exec -it $CONTAINER_NAME bash /docker-entrypoint.sh
    else
        echo "[*] Container exists but is stopped, starting and attaching..."
        docker start $CONTAINER_NAME
        sleep 2
        docker exec -it $CONTAINER_NAME bash /docker-entrypoint.sh
    fi
    exit 0
fi

# If container doesn't exist, create new one
echo "[*] Creating new container..."

docker run -it \
  --name $CONTAINER_NAME \
  --hostname docker-ub \
  $NETWORK_ARGS \
  $GPU_ARGS \
  -v ./shared:/root/shared \
  -v $WORKSPACE_TARGET:/root/workspace \
  -v ./docker-entrypoint.sh:/docker-entrypoint.sh \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR" \
  -v /dev/dri:/dev/dri \
  -e DISPLAY=${DISPLAY:-:0} \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=${XAUTHORITY} \
  -e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
  -e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all} \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  $IMAGE_NAME \
  bash /docker-entrypoint.sh