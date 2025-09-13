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