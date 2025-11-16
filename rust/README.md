# Robonix Quick Start

wheatfox

## workflow

Robonix系统的工作流程如下：
1. **准备环境**：安装ROS2和Rust，构建消息包
2. **启动robonix-core**：启动核心服务
3. **注册大模型**：注册用于DSL生成的AI模型
4. **注册能力技能**：通过recipe注册机器人的能力和技能
5. **构建语义地图**：添加实体（包括机器人）到语义地图，实体包含其支持的skills
6. **感知模块持续更新**：通过update_map skill持续更新语义地图和空间地图
7. **创建任务**：用户发送自然语言任务
8. **自动执行**：系统生成DSL代码并执行

## 准备环境

首先需要ROS2环境和Rust。

```bash
cd rust # at robonix src root folder
cd robonix-cli
cargo build
# in robonix-cli, run:
mkdir -p ~/.robonix; rm -rf ~/.robonix/packages; ln -s "$(realpath ../provider/)" ~/.robonix/packages;
export FASTRTPS_DEFAULT_PROFILES_FILE=

# then build the robonix-msg
cd robonix-msg
./build_ros2.sh
```

## 启动 robonix-core

在开始一切之前，需要先启动 robonix-core（在单独的终端中）。

```bash
cd robonix-core
cargo run --
```

robonix-core 会启动以下服务：
- **管理模块服务** (`/rbnx/srv/mgmt/*`)：能力技能注册查询、大模型注册查询
- **感知模块服务** (`/rbnx/srv/perception/*`)：语义地图和空间地图操作
- **规划模块服务** (`/rbnx/srv/planning/*`)：任务创建、查询、DSL生成
- **行动模块服务** (`/rbnx/srv/action/*`)：DSL执行

然后可以在另一个终端使用 `robonix-cli`。

## 配置 robonix-cli

```bash
cd robonix-cli
# 查看当前配置
cargo run -- config -s

# 设置 robonix-msg 路径
cargo run -- config --set-msg-path ../robonix-msg
cargo run -- config -s
```

## 注册大模型（用于DSL生成）

系统需要一个大模型来将自然语言任务转换为DSL代码。支持OpenAI兼容的API格式。

```bash
cd robonix-cli
export ROBONIX_MODEL_API_KEY=sk-xxxx # 或使用 --api-key 参数

# 注册DeepSeek模型示例
cargo run -- model register \
  --model-id deepseek-chat \
  --model-name "deepseek-chat" \
  --model-type llm \
  --provider deepseek \
  --api-endpoint https://api.deepseek.com/v1/chat/completions \
  --description "DeepSeek Chat model via DeepSeek API" \
  --capabilities "planning,reasoning,general"

# 查询已注册的模型
cargo run -- model query
```

## 安装和注册包（能力技能）

### 安装包

```bash
cd robonix-cli

# 从GitHub安装包
cargo run -- package install --github https://github.com/enkerewpo/demo-package-01-robonix
cargo run -- package list
cargo run -- package info demo_package_01_github

# 构建所有包，也可以指定包名称
cargo run -- package build all
```

### 注册包到 robonix-core（使用recipe）

使用recipe文件注册包，recipe会指定哪些能力技能注册到哪个实体（机器人）。

```bash
cd robonix-cli

# 构建所有包
cargo build && cargo run -- daemon restart # 如果修改了daemon代码
cargo run -- package build all

# 注册recipe（会自动注册recipe中指定的能力技能）
cargo run -- deploy register demo_recipe.yaml

# 启动所有注册的包
cargo run -- deploy start

# 查看状态
cargo run -- deploy status

# 重启/停止
cargo run -- deploy restart
cargo run -- deploy stop

# 清理所有ROS2进程
# pkill -9 -f "ros2|robonix|rclpy|rclcpp|demo_rgb_provider"
```

## 构建语义地图

语义地图包含环境中的实体（物体、机器人等）及其关系。机器人本身也是语义地图中的一个entity，包含其支持的skills信息。

`robonix-msg` ROS2 包提供了封装好的调用 robonix service 的接口代码，在 source 之后可以直接使用。

当注册了`skl::update_map`技能后，感知模块将处于“持续更新”状态，监控相关技能的执行。

```bash
# 确保update_map skill已注册并运行
cargo run -- deploy status

# 检查地图更新状态
# 通过ROS2服务调用 /rbnx/srv/perception/get_map_status
```

## 创建和执行任务

### 创建任务（自然语言输入）

```bash
cd robonix-cli

# 创建任务
cargo run -- task create "Pick up the red box on the table"

# 查看任务列表
cargo run -- task list

# 查看任务详情
cargo run -- task get task_0
```

### 任务执行流程

创建任务后，系统会自动执行以下流程：

1. **任务创建**：任务状态为 `Pending`
2. **DSL生成**：系统调用注册的大模型，根据任务描述和语义地图生成DSL代码
   - 大模型会看到：
     - 语义地图中的所有实体及其支持的skills
     - 所有注册的skills列表
     - 可用的数据类型（Robonix自定义类型和ROS2标准类型）
   - 任务状态变为 `Generating` → `Parsing`
3. **DSL执行**：action模块解析DSL代码，按顺序执行每个skill调用
   - 向skill的input topic发送参数
   - 收集skill的output
   - 任务状态变为 `Running`
4. **任务完成**：所有skill执行完成后，任务状态变为 `Completed` 或 `Failed`

### 查看任务状态

```bash
# 列出所有任务
cargo run -- task list

# 获取任务详情（包括生成的DSL代码）
cargo run -- task get <task_id>

# 取消任务
cargo run -- task cancel <task_id>
```

## 注意事项

1. **robonix-msg设置**：robonix-msg的setup会自动被启动脚本source。CLI会：
   - 首先检查配置文件（通过`rbnx config --set-msg-path`设置）
   - 然后检查`ROBONIX_MSG_PATH`环境变量

2. **实体和Skills**：
   - 机器人本身是语义地图中的一个entity
   - 实体的`registered_skills`字段包含该实体（机器人）支持的所有skills
   - 规划模块在生成DSL时会考虑实体支持的skills

3. **DSL格式**：
   - 当前的DSL格式是简单的指令列表，每行一个skill调用。后续会持续优化（如基于 PLEXIL）
   - 格式：`skill_name(param1=value1, param2=value2)`
   - 按顺序执行

4. **数据类型**：
   - 系统支持Robonix自定义消息类型（Point3D, Entity, BoundingBox等）
   - 也支持标准ROS2消息类型（geometry_msgs, sensor_msgs, std_msgs等）
   - 大模型在生成DSL时会知道这些数据类型

## 故障排查

```bash
# 检查robonix-core是否运行
ros2 service list | grep rbnx

# 检查服务是否可用
ros2 service call /rbnx/srv/mgmt/ping robonix_core/Ping "{sequence: 1}"
```
