# pointcloud related skills

wheatfox

## spatiallm

input: *.ply point cloud 3D model file
output: plaintext that marked each object with its bounding box and label

sptaillm also support "replay" data so there's visualization for the detection result.

### installation

https://github.com/manycore-research/SpatialLM

however installing spatiallm natively on jeston orin os seems to be a problem:

https://github.com/manycore-research/SpatialLM/issues/93

```
In file included from torchsparse/backend/hashmap/hashmap_cpu.cpp:1:
torchsparse/backend/hashmap/hashmap_cpu.hpp:7:10: fatal error: google/dense_hash_map: No such file or directory
    7 | #include <google/dense_hash_map>
    |          ^~~~~~~~~~~~~~~~~~~~~~~
```

```bash
sudo apt-get install libsparsehash-dev
```

https://github.com/rusty1s/pytorch_scatter/issues/428

see requirements.txt for some "normal" dependencies that do not cause much trouble.

```bash
pip install git+https://github.com/mit-han-lab/torchsparse.git # this may take forever :(, and requires a lot of space because we are building C++ code
pip install ninja flash-attn --no-build-isolation
pip install torch-scatter -f https://data.pyg.org/whl/torch-2.7.0+cu126.html # jeston orin uses cuda 12.6
pip install timm spconv-cu120
```

And also, spatiallm will download some LLM models, so there should be at least some dozens of GB to run this... and around 6 GB graphics memory is required - wheatfox.

### notes

# wheatfox 2025.6

# mkdir -p ~/miniconda3
# wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -O ~/miniconda3/miniconda.sh
# bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
# rm ~/miniconda3/miniconda.sh

# source ~/miniconda3/bin/activate
# conda create -n stt python=3.10
# conda activate stt

# sudo apt install ffmpeg

### INSTALL NVIDIA PYTORCH INTO DEFEULT ROS2 PYTHON ENVIRONMENT

sudo apt-cache show nvidia-jetpack | grep "Version"

wget https://pypi.jetson-ai-lab.dev/jp6/cu126/+f/6ef/f643c0a7acda9/torch-2.7.0-cp310-cp310-linux_aarch64.whl#sha256=6eff643c0a7acda92734cc798338f733ff35c7df1a4434576f5ff7c66fc97319 -O torch-2.7.0-cp310-cp310-linux_aarch64.whl

wget https://pypi.jetson-ai-lab.dev/jp6/cu126/+f/c59/026d500c57366/torchaudio-2.7.0-cp310-cp310-linux_aarch64.whl#sha256=c59026d500c573666ae0437c4202ac312ac8ebe38fa12dbb37250a07c1e826f9 -O torchaudio-2.7.0-cp310-cp310-linux_aarch64.whl

wget https://pypi.jetson-ai-lab.dev/jp6/cu126/+f/daa/bff3a07259968/torchvision-0.22.0-cp310-cp310-linux_aarch64.whl#sha256=daabff3a0725996886b92e4b5dd143f5750ef4b181b5c7d01371a9185e8f0402 -O torchvision-0.22.0-cp310-cp310-linux_aarch64.whl

pip install torch-2.7.0-cp310-cp310-linux_aarch64.whl torchaudio-2.7.0-cp310-cp310-linux_aarch64.whl torchvision-0.22.0-cp310-cp310-linux_aarch64.whl

sudo apt install libcudnn9-cuda-12 

https://pypi.jetson-ai-lab.io/

### for RTX 5090

pytorch should be at least 2.7 ! or 5090 cannot use it - wheatfox

https://github.com/lllyasviel/Fooocus/issues/3862

pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128

```bash
#!/usr/bin/env bash
set -euo pipefail
# NOTE: This script assumes Linux + Python 3.11 + CUDA 12.8 toolkit.

# 0) Create and activate a clean env
conda create -n spatiallm python=3.11 -y
conda activate spatiallm

# 1) Install CUDA 12.8 toolkit for building native extensions (nvcc)
conda install -y -c nvidia/label/cuda-12.8.0 cuda-toolkit conda-forge::sparsehash

# 2) Go to your project root (adjust path if needed)
cd spatiallm

# IMPORTANT: Ensure pyproject.toml does NOT pin torch/vision/audio to cu124 and has no poe tasks.

# 3) Install PyTorch stack first (supports RTX 5090 / sm_120)
#    Choose one of the following two lines:
#    Stable 2.8 (recommended if available):
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
#    Or pin to 2.7 specifically:
# pip install torch==2.7.* torchvision==0.22.* torchaudio==2.7.* --index-url https://download.pytorch.org/whl/cu128

# 4) Install project (pure Python deps) via Poetry without creating its own venv
pip install poetry
poetry config virtualenvs.create false --local
poetry install

# 5) Native extensions that previously used poe (install manually)

# 5.1 TorchSparse (build from source; needs nvcc from CUDA 12.8)
#     If you hit build errors, export CUDA_HOME and (optionally) TORCH_CUDA_ARCH_LIST.
export CUDA_HOME="${CONDA_PREFIX}"
# export TORCH_CUDA_ARCH_LIST="12.0"  # Uncomment to force Blackwell build only
pip install --no-cache-dir git+https://github.com/mit-han-lab/torchsparse.git

# 5.2 torch-scatter (pick the wheel index matching the installed torch version)
#     If you installed torch==2.8.* above, use 2.8.0+cu128; for 2.7.*, change the URL accordingly.
pip install --no-cache-dir torch-scatter -f https://data.pyg.org/whl/torch-2.8.0+cu128.html
# Alternative for torch 2.7.*:
# pip install --no-cache-dir torch-scatter -f https://data.pyg.org/whl/torch-2.7.0+cu128.html

# 5.3 FlashAttention (recent versions support torch>=2.7; build from source if no wheel)
pip install --no-cache-dir ninja
pip install --no-build-isolation --no-cache-dir flash-attn

# 5.4 spconv (try prebuilt CUDA12 wheels first; fall back to source if needed)
pip install --no-cache-dir timm spconv-cu120

# 6) (Optional) Training extras previously defined in poe install-training
pip install --no-cache-dir "omegaconf" "datasets<=3.6.0" "accelerate<=1.7.0" "wandb"

# 7) Sanity check (GPU, versions)
python - <<'PY'
import torch, torchvision, torchaudio
print("torch:", torch.__version__)
print("torchvision:", torchvision.__version__)
print("torchaudio:", torchaudio.__version__)
print("cuda available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("device:", torch.cuda.get_device_name(0))
    a = torch.randn(1024, 1024, device="cuda"); b = torch.randn(1024, 1024, device="cuda")
    c = a @ b
    print("matmul ok:", c.is_cuda, c.shape)
PY
```