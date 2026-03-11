#!/bin/bash
set -e

mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
export XDG_RUNTIME_DIR=/tmp/runtime-root

# Set up OSMesa for headless OpenGL rendering
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/osmesa:${LD_LIBRARY_PATH}
export OSMESA_LIB=/usr/lib/x86_64-linux-gnu/libOSMesa.so

# some post apt install
apt install -qq -y iputils-ping screenfetch >/dev/null 2>&1

# source the setup.bash
source /opt/ros/humble/setup.bash

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROBONIX_META_GRPC_ADDR="${ROBONIX_META_GRPC_ADDR:-0.0.0.0:50051}"
export ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-127.0.0.1:50051}"

start_zenoh_router() {
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "[!] ros2 not found in container"
        return 1
    fi
    if ! ros2 pkg executables rmw_zenoh_cpp >/dev/null 2>&1; then
        echo "[!] rmw_zenoh_cpp is not installed"
        return 1
    fi
    echo "[*] Starting zenoh router in background..."
    nohup ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/zenoh-router.log 2>&1 &
    echo "[*] Zenoh router log: /tmp/zenoh-router.log"
}

if [ "${ZENOH_ROUTER_AUTO_START:-0}" = "1" ]; then
    start_zenoh_router
fi

if ! grep -q "### ROBONIX_DOCKER_ENV ###" ~/.bashrc 2>/dev/null; then
cat <<EOF >> ~/.bashrc
### ROBONIX_DOCKER_ENV ###
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
export ROBONIX_META_GRPC_ADDR=${ROBONIX_META_GRPC_ADDR}
export ROBONIX_META_GRPC_ENDPOINT=${ROBONIX_META_GRPC_ENDPOINT}
EOF
cat <<'EOF' >> ~/.bashrc
start_zenoh_router() {
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "[!] ros2 not found in container"
        return 1
    fi
    if ! ros2 pkg executables rmw_zenoh_cpp >/dev/null 2>&1; then
        echo "[!] rmw_zenoh_cpp is not installed"
        return 1
    fi
    echo "[*] Starting zenoh router in background..."
    nohup ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/zenoh-router.log 2>&1 &
    echo "[*] Zenoh router log: /tmp/zenoh-router.log"
}
EOF
fi

# Enable bash completion
if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
    if ! grep -q "### ROBONIX_BASH_COMPLETION ###" ~/.bashrc 2>/dev/null; then
        cat <<'EOF' >> ~/.bashrc
### ROBONIX_BASH_COMPLETION ###
if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
fi
EOF
    fi
elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
    if ! grep -q "### ROBONIX_BASH_COMPLETION ###" ~/.bashrc 2>/dev/null; then
        cat <<'EOF' >> ~/.bashrc
### ROBONIX_BASH_COMPLETION ###
if [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
fi
EOF
    fi
fi

echo -e "[*] \033[1mWelcome to robonix docker environment!\033[0m Distro is: \033[33m$(lsb_release -ds 2>/dev/null || echo "Linux")\033[0m with ROS2 \033[33m$(echo $ROS_DISTRO)\033[0m"
echo "[*] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[*] ROBONIX_META_GRPC_ADDR=$ROBONIX_META_GRPC_ADDR"
echo "[*] ROBONIX_META_GRPC_ENDPOINT=$ROBONIX_META_GRPC_ENDPOINT"
echo "[*] gRPC Python codegen: python3 -m grpc_tools.protoc"
echo "[*] Use start_zenoh_router to launch zenohd manually"
exec bash

