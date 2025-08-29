# Genesis Robot Simulator

基于 Genesis 物理引擎的机器人仿真环境，支持键盘控制、gRPC 远程控制和相机采集系统。

## 安装

```bash
pip install grpcio grpcio-tools genesis-world pynput opencv-python numpy scipy
```

## 运行

```bash
python simulator/genesis/robot1.py
```

## 控制

**移动**: 方向键控制前进/后退/左移/右移
**旋转**: `[` 左转, `]` 右转
**重置**: `R` 键让小车瞬移回初始位置（TODO：其他物品也支持 reset）
**退出**: `ESC`

## 架构

**robot1.py**: 主程序
**scene_manager.py**: 3D场景管理，创建房间、机器人、相机等
**car_controller.py**: 机器人运动控制，处理输入和运动学
**camera_manager.py**: 相机图像采集和保存
**keyboard_device.py**: 键盘事件监听
**main_loop.py**: 主控制 loop
**grpc_service.py**: gRPC远程控制服务

## 通信

gRPC 服务端口 50051，支持 Move、Rotate、Stop、GetPose 等接口，具体见 `robot_control.proto`。

之后配好服务器之后，应该可以指定服务器的 IP 作为 gRPC 的 server。

## 开发

添加新功能时修改对应模块，更新 proto 文件后运行`./gen_grpc.sh`重新生成 gRPC 代码。
