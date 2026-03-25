# `prm_camera`

ROS 包名 **`prm_camera`**：承载与 **`robonix/prm/camera/*`** 相关的 **gRPC 生成用** `.srv`（ROS 2 运行时未必实现为同名 service）。

## `srv/SubscribeRgb.srv`

- 首行指令 `# @robonix.grpc stream_server sensor_msgs/msg/Image` 仅由 **ridlc** 识别，用于生成 **server streaming** RPC（模拟 **ROS publisher** 对外推流）。
- 生成结果：`robonix_proto/prm_camera.proto` → `PrmCameraService.SubscribeRgb` → `returns (stream robonix.sensor_msgs.Image)`。

普通 **topic** 载荷仍使用 **`lib/common_interfaces/sensor_msgs/msg/Image.msg`**，无需在本包重复定义 `.msg`。详见仓库根 `rust/robonix-interfaces/README.md` 中 **prm / camera** 表与手册 [interface-spec.md](../../../../docs/src/chapter3-developer-guide/interface-spec.md)。
