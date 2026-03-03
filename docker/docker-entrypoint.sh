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

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Enable bash completion
if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
    echo "# Enable bash completion" >> ~/.bashrc
    echo "if [ -f /usr/share/bash-completion/bash_completion ]; then" >> ~/.bashrc
    echo "    . /usr/share/bash-completion/bash_completion" >> ~/.bashrc
    echo "fi" >> ~/.bashrc
elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
    echo "# Enable bash completion" >> ~/.bashrc
    echo "if [ -f /etc/bash_completion ]; then" >> ~/.bashrc
    echo "    . /etc/bash_completion" >> ~/.bashrc
    echo "fi" >> ~/.bashrc
fi

echo -e "[*] \033[1mWelcome to robonix docker environment!\033[0m Distro is: \033[33m$(lsb_release -ds 2>/dev/null || echo "Linux")\033[0m with ROS2 \033[33m$(echo $ROS_DISTRO)\033[0m"
exec bash

