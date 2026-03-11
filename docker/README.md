# Docker Setup for Robonix

This directory contains Docker configuration files for building and running Robonix in a containerized environment.

## Files

- `Dockerfile`: Docker image definition based on ROS2 Humble
- `run.sh`: Script to build and run the Docker container locally
- `upload.sh`: Script to build and upload Docker image to cloud registry
- `docker-entrypoint.sh`: Container entrypoint script

## Local Usage

### Build and Run Container

```bash
cd docker
./run.sh          # Run container (builds if needed)
./run.sh -b       # Force rebuild image
./run.sh -d       # Delete existing container
```

### Environment Variables

Create a `.env` file in the `docker/` directory to set custom variables:

```bash
# Optional: Custom image name
DOCKER_IMAGE_NAME=robonix_ros

# Optional: Custom registry
DOCKER_REGISTRY=ghcr.io/username

# Optional: Custom tag
DOCKER_TAG=latest

# Optional: Disable push (default: true)
DOCKER_PUSH=false

# Optional: Docker credentials (for upload.sh)
DOCKER_USERNAME=your_username
DOCKER_PASSWORD=your_token_or_password
```

## Upload to Cloud Registry

### Docker Hub (Default)

```bash
cd docker

# Set credentials (or use .env file)
export DOCKER_USERNAME=your_dockerhub_username
export DOCKER_PASSWORD=your_dockerhub_password

# Upload with default settings (uses docker.io)
./upload.sh docker.io/username/robonix_ros latest

# Or use environment variables
export DOCKER_REGISTRY=docker.io/username/robonix_ros
export DOCKER_TAG=v1.0.0
./upload.sh
```

### Using .env File

1. Copy the template file:
```bash
cd docker
cp dotenv.template .env
```

2. Edit `.env` and fill in your values:

```bash
# For Docker Hub (default)
DOCKER_REGISTRY=docker.io/your_dockerhub_username/robonix_ros
DOCKER_TAG=latest
DOCKER_USERNAME=your_dockerhub_username
DOCKER_PASSWORD=your_dockerhub_password_or_token
DOCKER_PUSH=true
```

**Important Notes:**
For Docker Hub: Use your Docker Hub username and password (or access token). Create access token at: https://hub.docker.com/settings/security.

3. Run the upload script:
```bash
./upload.sh
```

The script will automatically use values from `.env` if no command-line arguments are provided.

## Notes

- The Docker image includes ROS2 Humble, Rust, gRPC Python tooling, and Zenoh RMW support
- GPU support is enabled if NVIDIA drivers are available on the host
- The container uses host networking for ROS2 communication
- X11 forwarding is set up for GUI applications

## gRPC And Zenoh RMW

The image now includes the pieces needed by the current Robonix stack:

- `ros-humble-rmw-zenoh-cpp` for ROS 2 over Zenoh
- `grpcio-tools`, `python3-grpcio`, `python3-protobuf`, and `protobuf-compiler` for gRPC and protobuf workflows
- Default runtime environment:
  - `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
  - `ROBONIX_META_GRPC_ADDR=0.0.0.0:50051`
  - `ROBONIX_META_GRPC_ENDPOINT=127.0.0.1:50051`

You can override these before running `./run.sh`, for example:

```bash
export ZENOH_ROUTER_AUTO_START=1
export ROBONIX_META_GRPC_ENDPOINT=192.168.1.10:50051
./run.sh -b
```

Inside the container, if you do not auto-start the router, you can start it manually with:

```bash
start_zenoh_router
```

