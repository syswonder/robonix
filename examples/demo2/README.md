# 在物理小车上运行 action

## 环境要求

如果是 ubuntu 22，请通过官方方法直接在系统 python 环境安装 ROS2 Humble

如果不是 ubuntu 22，请使用虚拟机，请注意，此处的环境仅为保证 ROS2 相关的导入存在，如果需要实际控制 ranger，必须在小车的 Jeston Orin AGX 系统中运行。

## 仅导出 skill 文件

```bash
python manager/eaios_decorators.py --config config/include/ranger_test.yaml
```

## 小车 Jeston Orin Only：启动小车 ROS2 节点和相关驱动

```bash
python manager/boot.py --config config/include/ranger_test.yaml
```