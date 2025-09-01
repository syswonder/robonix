# examples

wheatfox <wheatfox17@icloud.com>

## 环境要求

下面是针对 python 3.12 的安装命令，适用于运行 genesis 模拟器以及运行 example 程序，其中包括模拟器相关的 driver 所使用的包（如 gRPC）：

```bash

# make sure you installed nvidia drivers and cuda toolkit
sudo apt-get install freeglut3 freeglut3-dev mesa-utils
export __GLX_VENDOR_LIBRARY_NAME=nvidia # 强制让 NVIDIA 显卡作为 OpenGL 的渲染后端，否则启动 genesis 会卡在 build visualizer

conda create -n genesis python=3.12
conda activate genesis

# -> then install pytorch according to https://pytorch.org/get-started/locally/ <-
# pip3 install torch torchvision (for example, on linux CUDA 12.8)

pip install rich loguru mcp pyyaml argparse grpcio grpcio-tools ultralytics genesis-world pynput openai python-dotenv opencv-python
```

## 如何运行 genesis action demo

修改模拟器运行的 IP 地址，在 `driver/sim_genesis_ranger/driver.py` 中，将 `TARGET_SERVER_IP` 修改为模拟器运行的 IP 地址（远程服务器或本地）。

1. 先运行 `python simulator/genesis/robot1.py` 启动模拟器，等待出现渲染窗口

2. 运行 `python manager/eaios_decorators.py --config config/include/simulator.yml`，导出 `skill/__init__.py`（如果要在物理小车上运行，请使用 `config/include/ranger_test.yml`，并参见 [examples/demo2/README.md](/examples/demo2/README.md)）

3. 下载并保存 yoloe 模型文件以供 `sim_vision` 相关 skill 使用：

```
cd skill/sim_vision/models
wget https://huggingface.co/jameslahm/yoloe/resolve/main/yoloe-11l-seg-pf.pt
```

4. 运行 `python examples/demo1/simple_demo.py --mode auto` 或 `--mode manual`，其中 auto 是用yolo辅助生成实体图

auto 模式下小车目前会自动识别物体，并自动绑定 action 参数，action 会让小车移动到识别到的物体位置。