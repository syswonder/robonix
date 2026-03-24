FROM osrf/ros:humble-desktop
    
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

RUN apt-get update && apt-get install -y \
    ca-certificates \
    wget gnupg && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get install -y cuda-toolkit-12-4 && \
    rm cuda-keyring_1.1-1_all.deb

# RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
#     bash Miniconda3-latest-Linux-x86_64.sh -b -p /opt/miniconda3 && \
#     rm Miniconda3-latest-Linux-x86_64.sh

RUN apt update && apt install -y \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-zenoh-cpp \
    python3-pip \
    python3-grpcio \
    python3-protobuf \
    python3-dev \
    build-essential \
    git \
    curl \
    wget \
    protobuf-compiler \
    libeigen3-dev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    bzip2 \
    libosmesa6 \
    libglu1-mesa \
    libglfw3 \
    libglfw3-dev \
    mesa-utils \
    libclang-dev python3-vcstool \
    bash-completion \
    fonts-lmodern \
    ros-humble-test-msgs \
    python-is-python3

RUN pip3 install --no-cache-dir \
    grpcio-tools openai pyarrow "eclipse-zenoh>=1.2,<2" \
    mujoco>=3.3.0 \
    robosuite \
    Pillow

RUN pip3 install --no-cache-dir \
    "lerobot[smolvla]" \
    libero

RUN GRPCURL_VERSION=1.9.3 && \
    curl -sSL "https://github.com/fullstorydev/grpcurl/releases/download/v${GRPCURL_VERSION}/grpcurl_${GRPCURL_VERSION}_linux_x86_64.tar.gz" \
    | tar -xz -C /usr/local/bin grpcurl

# RUN apt update && apt install -y curl && \
# curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.noarmor.gpg | tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null && \
# curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.tailscale-keyring.list | tee /etc/apt/sources.list.d/tailscale.list && \
# apt update && \
# apt install -y tailscale && \
# pip3 install --upgrade pip

# ENV PATH=/opt/miniconda3/bin:$PATH

# RUN conda tos accept || true && \
#     conda create -n env1 python=3.10 -y && \
#     conda clean -ya

# RUN conda config --set solver classic && \
#     conda config --set channel_priority flexible

RUN pip3 install --no-cache-dir \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124

# RUN conda run -n env1 pip install git+https://github.com/colcon/colcon-cargo.git && \
#     conda run -n env1 pip install git+https://github.com/colcon/colcon-ros-cargo.git

ENV CUDA_HOME=/usr/local/cuda
ENV CUDA_PATH=/usr/local/cuda
ENV PATH=/usr/local/cuda/bin:$PATH
RUN echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/compat:${LD_LIBRARY_PATH:-}' >> ~/.bashrc
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

# install rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

WORKDIR /root/workspace