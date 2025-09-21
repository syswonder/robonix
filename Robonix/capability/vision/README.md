# vision使用说明

该capability使用[yoloe](https://docs.ultralytics.com/models/yoloe/#introduction)算法以及云端VLM，实现对物体的识别和位置定位，以及语义地图的创建和更新。

## 1. 安装依赖

首先，确保您已经更新了系统并且安装了必要的依赖。以下是一些安装步骤，其中`$ROS_DISTRO` 是您的ROS2发行版（例如：`foxy`、`galactic`）：

```bash
sudo apt update
sudo apt install python3-pip ros-$ROS_DISTRO-vision-msgs
pip install -U ultralytics # 务必保证ultralytics处于最新版
```

## 2. 运行节点

yolo和VLM两个ROS节点在系统启动时自动运行。其中yolo节点各输入参数含义如下：

- **`device:=cuda:0`**:

  指定计算设备，默认为 `cpu`。这里设置为 `cuda:0`，表示使用第一个 GPU 进行计算。如果使用 CPU，可以设置为 `cpu`。

- **`interest:="person"`**:

  指定要定位位置的目标对象，默认为 `"bottle"`。这里设置为 `"person"`，表示检测场景中的“人”。你可以根据需要更改为其他目标，如 `"bottle"`、`"cup"` 等。被检测到的对象会从话题`/piper_vision/target_point`发布。

- **`depth_threshold:=15.0`**:

  设置深度阈值，单位为米，默认为 `2.0`。超过深度阈值的物体会被视为背景，背景会在yolo进行推理前从图片中被移除。该参数仅在 `bg_removal` 为 `True` 时有效。

- **`model:=yoloe-11l-seg-pf`**:

  指定使用的 YOLO 模型，默认为 `"yoloe-11l-seg-pf"`。这里使用 `"yoloe-11l-seg-pf"` 模型进行目标检测和分割。你可以根据需要选择其他模型。

- **`bg_removal:=False`**:

  是否启用背景移除功能，默认为 `True`。这里设置为 `False`，表示不进行背景移除。如果设置为 `True`，系统会根据 `depth_threshold` 移除背景。

- **`conf_threshold:=0.7`**:
  设置置信度阈值，默认为 `0.7`。检测到的目标物体的置信度低于该值时，会被过滤掉，不会发布到话题中。可以根据需要调整该值以平衡检测精度和召回率。

- **`target_frame_id:=map`**:
  设置坐标转换的目标坐标系，用于`/piper_vision/all_object_points`。

- `tf_translation:=[0.3, 0.0, 0.1, -1.5708, 0.0, -1.5708]`：

  设置从camera_link到base_link的坐标转换矩阵：`[x y z roll pitch yaw]`，含义为**camera_link** 相对于 **base_link**：

  - 位置平移了 `(x, y, z)`
  - 并且绕 base_link 的 XYZ 轴（注意这是不动的）分别进行了 `(roll, pitch, yaw)` 的旋转

yolo_ros2节点会在以下的topic中发布消息：

- `/piper_vision/all_object_points` 用于发布看到的所有目标物体的位置（相对于target_frame_id的坐标）和尺寸信息。
- `/piper_vision/target_point` 仅发布名为参数`interest`的目标物体的位置和尺寸信息。
- `/piper_vision/pred_image` 用于发布经过 YOLO 检测和标注后的图像，便于可视化检测结果。可以用rviz2查看。

## 3. 有关yoloe的介绍

yoloe的[论文](https://arxiv.org/html/2503.07465v1)介绍：

> In light of these, in this paper, we introduce YOLOE(ye), a highly **efficient**, **unified**, and **open** object detection and segmentation model, like human eye, under different prompt mechanisms, like texts, visual inputs, and prompt-free paradigm.

简单来说，使用yoloe可以识别更多的物体，还可以用text prompt指定识别图片中的哪个物体。目前我们可以用到两种模型，对应的模型请到这里[下载](https://docs.ultralytics.com/models/yoloe/#introduction)：

1. prompt-free yoloe

YOLOE also includes prompt-free variants that come with a built-in vocabulary. These models don't require any prompts and work like traditional YOLO models. Instead of relying on user-provided labels or visual examples, they detect objects from a [predefined list of 4,585 classes](https://github.com/xinyu1205/recognize-anything/blob/main/ram/data/ram_tag_list.txt) based on the tag set used by the [Recognize Anything Model Plus (RAM++)](https://arxiv.org/abs/2310.15200).

2. Text prompt

Text prompts allow you to specify the classes that you wish to detect through textual descriptions. The following code shows how you can use YOLOE to detect people and buses in an image.

## 4. 语义地图构建指引

在piper_vision下的piper_vision_api.py有各种函数，供上层调用。