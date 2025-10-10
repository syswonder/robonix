Simple Demo (Genesis)
============

本章通过 ``examples/demo1`` 中的完整示例，演示如何使用Robonix OS框架开发具身应用应用。

环境配置
-------

运行示例前，需要完成以下环境配置步骤：

**系统要求**

- Python 3.12
- NVIDIA 显卡驱动和 CUDA 工具包
- OpenGL 支持

**安装系统依赖**

.. code-block:: bash

   # Install OpenGL related dependencies
   sudo apt-get install freeglut3 freeglut3-dev mesa-utils
   
   # Configure NVIDIA as OpenGL rendering backend (Important!)
   export __GLX_VENDOR_LIBRARY_NAME=nvidia

**创建 Python 环境**

.. code-block:: bash

   # Create and activate conda environment
   conda create -n genesis python=3.12
   conda activate genesis

**安装依赖包**

.. code-block:: bash

   # First install PyTorch according to https://pytorch.org/get-started/locally/
   # For example: pip3 install torch torchvision (Linux CUDA 12.8)
   
   # Install other dependencies
   pip install rich loguru mcp pyyaml argparse grpcio grpcio-tools ultraimport \
               ultralytics genesis-world pynput openai python-dotenv opencv-python

Genesis 模拟器启动
-----------------

**1. 配置模拟器连接**

在运行示例前，需要修改模拟器连接配置。编辑 ``driver/sim_genesis_ranger/driver.py`` 文件，将 ``TARGET_SERVER_IP`` 修改为模拟器运行的 IP 地址：

- 本地运行：使用 ``localhost`` 或 ``127.0.0.1``
- 远程服务器：使用服务器的实际 IP 地址

**2. 启动模拟器**

.. code-block:: bash

   # Start Genesis simulator in robonix root directory
   python start_genesis.py

等待模拟器启动完成，直到出现渲染窗口。

**3. 导出技能系统**

.. code-block:: bash

   # Export skill system configuration in robonix root directory (simulator mode)
   python manager/eaios_decorators.py --config config/include/simulator.yml

此命令会生成 ``skill/__init__.py`` 文件，用于技能系统的初始化。

.. note::
   如果要在物理小车上运行，请使用 ``config/include/ranger_test.yml`` 配置文件。

**4. 下载视觉模型**

为了使用视觉相关技能，需要下载 YOLO 模型：

.. code-block:: bash

   # Execute in robonix root directory
   mkdir -p skill/sim_vision/models
   wget -P skill/sim_vision/models https://github.com/ultralytics/assets/releases/download/v8.3.0/yoloe-11l-seg-pf.pt


示例概述
-------

``simple_demo.py`` 展示了一个完整的具身应用应用开发流程，包括系统初始化、实体图构建、技能绑定和动作执行。该示例支持两种运行模式，适合不同的使用场景。

运行示例
-------

完成环境配置和模拟器启动后，可以运行示例程序。

其中加载的 action 程序为 ``examples/demo1/simple.action``。 

**手动模式**

.. code-block:: bash

   # Run in robonix root directory
   python examples/demo1/simple_demo.py --mode manual

手动模式下，用户需要手动指定目标物体和动作参数。

**自动模式**

.. code-block:: bash

   # Run in robonix root directory
   python examples/demo1/simple_demo.py --mode auto

自动模式下，系统会：

- 使用 YOLO 模型自动识别场景中的物体
- 自动生成实体图
- 自动绑定动作参数
- 让小车自动移动到识别到的物体位置

**导出场景信息**

.. code-block:: bash

   # Run in robonix root directory
   python examples/demo1/simple_demo.py --mode manual --export-scene scene_info.json

此命令可以将当前场景信息导出为 JSON 文件，便于后续分析和调试。
