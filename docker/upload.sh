#!/usr/bin/env bash
set -e

# Script to build and upload Docker image to cloud registry
# Usage: ./upload.sh [registry] [tag]
# Example: ./upload.sh docker.io/username/robonix_ros latest

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/docker_base_image.sh"
ROBONIX_ROS_DEV_BASE_IMAGE="${ROBONIX_ROS_DEV_BASE_IMAGE:-robonix-osrf-ros:humble-desktop}"

# Default values
DEFAULT_REGISTRY="docker.io"
DEFAULT_IMAGE_NAME="robonix_ros"
DEFAULT_TAG="latest"

# Load environment variables from .env if exists
if [ -f .env ]; then
    echo "[*] Loading environment variables from .env..."
    export $(cat .env | grep -v '^#' | xargs)
fi

# Parse arguments
REGISTRY=${1:-${DOCKER_REGISTRY:-$DEFAULT_REGISTRY}}
IMAGE_NAME=${2:-${DOCKER_IMAGE_NAME:-$DEFAULT_IMAGE_NAME}}
TAG=${3:-${DOCKER_TAG:-$DEFAULT_TAG}}

# Construct full image name
if [[ "$REGISTRY" == *"/"* ]]; then
    # Full path provided (e.g., ghcr.io/username/robonix_ros)
    FULL_IMAGE_NAME="${REGISTRY}:${TAG}"
else
    # Only registry provided, use default image name
    FULL_IMAGE_NAME="${REGISTRY}/${IMAGE_NAME}:${TAG}"
fi

echo "[*] Building Docker image..."
echo "[*] Image: $FULL_IMAGE_NAME"

# Build the image
robonix_ensure_local_base_image "$ROBONIX_ROS_DEV_BASE_IMAGE" "osrf/ros:humble-desktop"
docker build --pull=false --build-arg "ROS_BASE_IMAGE=$ROBONIX_ROS_DEV_BASE_IMAGE" -t "$FULL_IMAGE_NAME" "$SCRIPT_DIR"

# Also tag as latest if a specific tag was provided
if [ "$TAG" != "latest" ]; then
    LATEST_TAG="${FULL_IMAGE_NAME%:*}:latest"
    echo "[*] Tagging as latest: $LATEST_TAG"
    docker tag $FULL_IMAGE_NAME $LATEST_TAG
fi

# Check if we should push to registry
if [ "${DOCKER_PUSH:-true}" == "true" ]; then
    echo "[*] Pushing image to registry..."
    
    # Check if we need to login
    if [ -n "${DOCKER_USERNAME}" ] && [ -n "${DOCKER_PASSWORD}" ]; then
        echo "[*] Logging in to registry..."
        if [[ "$REGISTRY" == *"ghcr.io"* ]]; then
            echo "${DOCKER_PASSWORD}" | docker login ghcr.io -u "${DOCKER_USERNAME}" --password-stdin
        elif [[ "$REGISTRY" == *"docker.io"* ]] || [[ "$REGISTRY" == *"dockerhub"* ]]; then
            echo "${DOCKER_PASSWORD}" | docker login docker.io -u "${DOCKER_USERNAME}" --password-stdin
        else
            echo "${DOCKER_PASSWORD}" | docker login "$REGISTRY" -u "${DOCKER_USERNAME}" --password-stdin
        fi
    fi
    
    # Push the image
    docker push $FULL_IMAGE_NAME
    
    # Push latest tag if created
    if [ "$TAG" != "latest" ] && [ -n "$LATEST_TAG" ]; then
        docker push $LATEST_TAG
    fi
    
    echo "[*] Image pushed successfully: $FULL_IMAGE_NAME"
else
    echo "[*] Skipping push (set DOCKER_PUSH=false to disable)"
fi

echo "[*] Done! Image: $FULL_IMAGE_NAME"

