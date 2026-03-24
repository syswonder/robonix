#!/bin/bash
set -e

mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
export XDG_RUNTIME_DIR=/tmp/runtime-root

export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/osmesa:${LD_LIBRARY_PATH}
export OSMESA_LIB=/usr/lib/x86_64-linux-gnu/libOSMesa.so

apt install -qq -y iputils-ping screenfetch >/dev/null 2>&1

source /opt/ros/humble/setup.bash

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROBONIX_META_GRPC_ADDR="${ROBONIX_META_GRPC_ADDR:-0.0.0.0:50051}"
export ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-127.0.0.1:50051}"

if ! grep -q "### ROBONIX_DOCKER_ENV ###" ~/.bashrc 2>/dev/null; then
cat <<EOF >> ~/.bashrc
### ROBONIX_DOCKER_ENV ###
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
export ROBONIX_META_GRPC_ADDR=${ROBONIX_META_GRPC_ADDR}
export ROBONIX_META_GRPC_ENDPOINT=${ROBONIX_META_GRPC_ENDPOINT}
EOF
fi

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

echo -e "[*] \033[1mRobonix docker\033[0m ROS2 \033[33m${ROS_DISTRO}\033[0m"
echo "[*] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[*] ROBONIX_META_GRPC_ADDR=$ROBONIX_META_GRPC_ADDR"
exec bash
