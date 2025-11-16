# Robonix Package Specification

## 概述

本文档定义了 robonix provider package 的规范。一个 package 可以提供多个 capabilities (cap) 和 skills (skl)。

**重要**: Robonix 不关心开发者如何实现 capabilities 和 skills。开发者可以使用任何技术栈（Python、C++、Rust、ROS2等）。Robonix 只要求 package 提供标准的 manifest 文件和必要的控制脚本，并提供 ROS2 通信接口。

## Package 目录结构

一个标准的 robonix provider package 应包含以下结构：

```
provider_package/
├── rbnx_manifest.yaml    # Robonix package manifest (必需)
└── [source code]         # 源代码 (实现相关，任意技术栈)
```

## 必需配置文件

### 1. rbnx_manifest.yaml

这是 robonix package 的核心配置文件，定义了 package 提供的所有 capabilities 和 skills。

#### 格式规范

```yaml
package:
  name: string              # Package 名称
  version: string           # 版本号 (语义化版本)
  description: string       # 描述
  maintainer: string        # 维护者
  maintainer_email: string  # 维护者邮箱
  license: string           # 许可证
  build_script: string      # 可选：构建脚本路径 (相对于 package 根目录，如 "rbnx/build.sh")。如果省略，默认查找 "rbnx/build.sh"

capabilities:
  - name: string            # 标准能力名称 (如 cap::grasp.move)
    impl_id: string         # 可选：实现标识符 (如 "algo01", "algo02")。如果省略，默认为 "default"
    start_script: string    # 启动脚本路径 (相对于 package 根目录，如 "rbnx/start_cap.sh")
    stop_script: string     # 停止脚本路径 (相对于 package 根目录，如 "rbnx/stop_cap.sh")
    inputs:                 # 输入参数通道映射 (字典格式: {参数名: topic通道})
      parameter_name: topic_channel
    outputs:                # 输出参数通道映射 (字典格式: {参数名: topic通道})
      parameter_name: topic_channel
    configs: {}             # 配置服务映射 (字典格式，通常为空)

skills:
  - name: string            # 标准技能名称 (如 skl::pick)
    impl_id: string         # 可选：实现标识符 (如 "algo01", "algo02")。如果省略，默认为 "default"
    start_script: string    # 启动脚本路径 (相对于 package 根目录，如 "rbnx/start_skill.sh")
    stop_script: string     # 停止脚本路径 (相对于 package 根目录，如 "rbnx/stop_skill.sh")
    inputs:                 # 输入参数通道映射 (字典格式: {参数名: topic通道})
      parameter_name: topic_channel
    outputs:                # 输出参数通道映射 (字典格式: {参数名: topic通道})
      parameter_name: topic_channel
    configs: {}             # 配置服务映射 (字典格式，通常为空)
```

**重要说明：**
- **capability/skill 的 `name` 字段是必需的**：必须在 manifest 中指定标准名称（如 `cap::vision.capture_rgb`、`skl::pick`），用于标识该 capability/skill 符合哪个标准规范
- **参数的 ROS 类型无需指定**：参数的类型（如 `sensor_msgs/msg/Image`、`geometry_msgs/msg/PoseStamped`）会从 spec 中自动获取，无需在 manifest 中重复指定
- **参数的名称必须指定**：在 `inputs` 和 `outputs` 字典中，key 必须是参数名（如 `image`、`target_pose`），且必须与 spec 中定义的标准参数名完全一致
- **参数到通道的映射**：只需提供参数名到 ROS2 topic 通道的映射关系，格式为字典：`{参数名: topic通道}`
- `impl_id` 用于区分同一标准名称下的不同实现。如果省略，系统会自动使用 "default" 作为默认值
- 一个 package 可以为同一个标准名称提供多个实现（通过不同的 `impl_id` 区分）
- `code_path`、`package_id`、`description` 等字段由 CLI 在注册时自动填充

#### 字段说明

**Package 字段：**
- `name`: Package 名称
- `version`: 语义化版本号 (如 1.0.0)
- `description`: Package 功能描述
- `maintainer`: 维护者名称
- `maintainer_email`: 维护者邮箱
- `license`: 许可证类型 (如 Apache-2.0)
- `build_script`: 构建脚本路径（可选），相对于 package 根目录的路径，如 `rbnx/build.sh`。如果省略，CLI 会默认查找 `rbnx/build.sh`。如果都不存在，build 命令会跳过该 package

**Capability 字段：**
- `name`: 标准能力名称，必须以 `cap::` 开头，格式为 `cap::category.action`。名称和参数定义由标准规范 (spec) 定义
- `impl_id`: 实现标识符（可选），用于区分同一标准名称下的不同实现。例如：`"algo01"`、`"algo02"`、`"fast"`、`"accurate"` 等。如果省略，系统会自动使用 `"default"` 作为默认值。**重要**：同一个 package 可以为同一个标准名称提供多个实现，只需使用不同的 `impl_id` 即可
- `start_script`: 启动脚本路径（必需），相对于 package 根目录的路径，如 `rbnx/start_cap.sh`。CLI 会在执行 `rbnx deploy start` 时使用此脚本启动对应的 capability 进程。每个 capability 必须有自己独立的启动脚本
- `stop_script`: 停止脚本路径（必需），相对于 package 根目录的路径，如 `rbnx/stop_cap.sh`。CLI 会在执行 `rbnx deploy stop` 时使用此脚本停止对应的 capability 进程。每个 capability 必须有自己独立的停止脚本
- `inputs`: 输入参数的通道映射，字典格式 `{参数名: topic通道}`。参数名必须与 spec 中定义的标准参数名一致
- `outputs`: 输出参数的通道映射，字典格式 `{参数名: topic通道}`。参数名必须与 spec 中定义的标准参数名一致
- `configs`: 配置服务的映射，字典格式（通常为空）

**Skill 字段：**
- `name`: 标准技能名称，必须以 `skl::` 开头。名称和参数定义由标准规范 (spec) 定义
- `impl_id`: 实现标识符（可选），用于区分同一标准名称下的不同实现。例如：`"algo01"`、`"algo02"`、`"fast"`、`"accurate"` 等。如果省略，系统会自动使用 `"default"` 作为默认值。**重要**：同一个 package 可以为同一个标准名称提供多个实现，只需使用不同的 `impl_id` 即可
- `start_script`: 启动脚本路径（必需），相对于 package 根目录的路径，如 `rbnx/start_skill.sh`。CLI 会在执行 `rbnx deploy start` 时使用此脚本启动对应的 skill 进程。每个 skill 必须有自己独立的启动脚本
- `stop_script`: 停止脚本路径（必需），相对于 package 根目录的路径，如 `rbnx/stop_skill.sh`。CLI 会在执行 `rbnx deploy stop` 时使用此脚本停止对应的 skill 进程。每个 skill 必须有自己独立的停止脚本
- `inputs`: 输入参数的通道映射，字典格式 `{参数名: topic通道}`。参数名必须与 spec 中定义的标准参数名一致
- `outputs`: 输出参数的通道映射，字典格式 `{参数名: topic通道}`。参数名必须与 spec 中定义的标准参数名一致
- `configs`: 配置服务的映射，字典格式（通常为空）

**自动填充字段（无需在 manifest 中指定）：**
- `package_id`: 由 CLI 根据 package 名称自动生成
- `code_path`: 由 CLI 自动设置为 package 的安装路径
- `description`: capability/skill 的描述从 spec 中自动获取
- 参数的 ROS 类型：从 spec 中自动获取，无需在 manifest 的 `inputs`/`outputs` 中指定

**示例：**

```yaml
capabilities:
  - name: cap::vision.capture_rgb
    # Spec 定义: OUTPUT: ["image" => "sensor_msgs/msg/Image"]
    start_script: rbnx/start_capture_rgb.sh
    stop_script: rbnx/stop_capture_rgb.sh
    outputs:
      image: /demo_rgb/image

  - name: cap::grasp.move
    # Spec 定义:
    #   INPUT: ["target_pose" => "geometry_msgs/msg/PoseStamped"]
    #   OUTPUT: ["status" => "boolean"]
    start_script: rbnx/start_grasp_move.sh
    stop_script: rbnx/stop_grasp_move.sh
    inputs:
      target_pose: /demo_grasp/pose_goal
    outputs:
      status: /demo_grasp/pose_status

skills:
  - name: skl::pick
    # Spec 定义:
    #   INPUT: ["target_label" => "string"]
    #   OUTPUT: ["status" => "boolean"]
    # impl_id 省略，将使用默认值 "default"
    start_script: rbnx/start_pick.sh
    stop_script: rbnx/stop_pick.sh
    inputs:
      target_label: /demo_pick/target_label
    outputs:
      status: /demo_pick/status

  # 示例：同一标准名称的多个实现
  - name: skl::pick
    impl_id: algo01  # 第一个实现
    start_script: rbnx/start_pick_algo01.sh
    stop_script: rbnx/stop_pick_algo01.sh
    inputs:
      target_label: /demo_pick_algo01/target_label
    outputs:
      status: /demo_pick_algo01/status

  - name: skl::pick
    impl_id: algo02  # 第二个实现（不同的算法）
    start_script: rbnx/start_pick_algo02.sh
    stop_script: rbnx/stop_pick_algo02.sh
    inputs:
      target_label: /demo_pick_algo02/target_label
    outputs:
      status: /demo_pick_algo02/status
```

**多实现说明：**
- 同一个 package 可以为同一个标准名称（如 `skl::pick`）提供多个实现
- 每个实现必须使用不同的 `impl_id` 来区分
- 每个实现可以有自己独立的启动/停止脚本和 ROS2 topic 通道
- 查询时可以通过指定 `impl_id` 来选择特定的实现，或者不指定 `impl_id` 来获取第一个匹配的实现（以及所有可用的 `impl_id` 列表）

### 2. rbnx/ 目录

（可选）推荐使用 `rbnx/` 目录包含所有 robonix 特定的配置和脚本。

**重要**: 
- 每个 capability 和 skill 都需要有自己独立的启动和停止脚本。这些脚本在 manifest 中通过 `start_script` 和 `stop_script` 字段指定。
- Package 可以有一个可选的构建脚本 `rbnx/build.sh`（或在 manifest 中通过 `build_script` 字段指定），用于编译、安装依赖等构建操作。

#### 构建脚本 (build_script)

Package 的构建脚本用于编译、安装依赖等构建操作。

脚本要求：
- 必须是可执行文件 (`chmod +x`)
- 应该在 package 根目录下执行
- 应该处理错误情况并返回适当的退出码（0 表示成功，非 0 表示失败）
- 脚本路径相对于 package 根目录
- 如果脚本不存在，build 命令会跳过该 package（不会报错）

**默认位置**: 如果 manifest 中没有指定 `build_script`，CLI 会默认查找 `rbnx/build.sh`。

#### 启动脚本 (start_script)

每个 capability/skill 的启动脚本用于启动对应的进程。

脚本要求：
- 必须是可执行文件 (`chmod +x`)
- 应该使用 `exec` 启动目标进程，这样 CLI 可以正确管理进程
- 应该设置必要的环境变量（ROS2、Python 路径等）
- 应该处理错误情况并返回适当的退出码
- 脚本路径相对于 package 根目录

#### 停止脚本 (stop_script)

每个 capability/skill 的停止脚本用于停止对应的进程。

脚本要求：
- 必须是可执行文件 (`chmod +x`)
- 应该停止对应的进程（发送 SIGTERM，必要时使用 SIGKILL）
- 应该清理临时文件和资源
- 脚本路径相对于 package 根目录

**注意**: 虽然 CLI 目前通过 PID 直接管理进程，但 `stop_script` 字段保留以备将来使用，或者用于执行额外的清理工作。

## 标准规范验证

所有 capabilities 和 skills 必须符合 `robonix-core` 中定义的标准规范。系统会在注册时进行验证：

1. 能力/技能名称必须匹配标准规范
2. 输入/输出参数名称必须与规范完全一致
3. 参数类型由 spec 自动提供，无需在 manifest 中指定
4. 配置服务必须与规范完全一致

CLI 在注册时会自动：
- 从 spec 中获取参数的类型信息
- 验证 manifest 中的参数名是否与 spec 定义一致
- 自动填充 `code_path`、`package_id`、`description` 等字段

## 注册流程

Package 在安装后，需要通过 CLI 命令 `rbnx deploy register <recipe_file>` 注册其提供的 capabilities 和 skills。

### Recipe 文件格式

Recipe 文件定义了要注册的 packages 和对应的 capabilities/skills，以及挂载的 entity 名称：

```yaml
name: recipe_name
description: Optional description

packages:
  - name: package_name
    entity_name: agilex_robot  # Entity 名称，用于在 entity tree 中挂载
    capabilities:              # 可选，如果不指定则注册所有 capabilities
      - cap::vision.capture_rgb
    skills:                    # 可选，如果不指定则注册所有 skills
      - skl::pick
```

### 部署工作流程

部署流程分为多个步骤，按顺序执行：

1. **注册阶段** (`rbnx deploy register <recipe_file>`): 向 robonix-core 注册每个 cap/skill
   - 每个注册请求包含 `hostname`（当前主机名）和 `entity_name`（从 recipe 中获取）
   - 这确保了 entity tree 和 graph 中的节点能够正确对应到 cap/skill
   - **注意**：注册时不会启动进程，只是将 cap/skill 信息注册到系统中

2. **构建阶段** (`rbnx deploy build [target]`): 构建 packages（编译、安装依赖等）
   - 可以指定 `target` 参数来构建特定的 package（如 `"demo_rgb_provider"`），或使用 `"all"` 构建所有 packages（默认）
   - 执行每个 package 的构建脚本（`rbnx/build.sh` 或在 manifest 中指定的 `build_script`）
   - 如果构建脚本不存在，会跳过该 package（不会报错）
   - **注意**：构建是可选的，如果 package 不需要构建（如纯 Python 脚本），可以跳过此步骤

3. **启动进程阶段** (`rbnx deploy start [target]`): 根据 manifest 中每个 cap/skill 的 `start_script` 启动对应的进程
   - 所有进程的 stdout/stderr 会被重定向到日志文件（存储在 `{package_storage_path}/logs/`）
   - CLI 会记录本机启动的所有 cap/skill 及其进程信息
   - 可以指定 `target` 参数来启动特定的 cap/skill，或使用 `"all"` 启动所有（默认）

4. **停止进程阶段** (`rbnx deploy stop [target]`): 停止正在运行的 cap/skill 进程
   - 可以指定 `target` 参数来停止特定的 cap/skill，或使用 `"all"` 停止所有（默认）

5. **重启进程阶段** (`rbnx deploy restart [target]`): 重启 cap/skill 进程
   - 相当于先执行 `stop` 再执行 `start`

6. **查看状态** (`rbnx deploy status`): 查看所有正在运行的 cap/skill 进程状态

7. **注销阶段** (`rbnx deploy unregister <target>`): 从系统中注销 cap/skill
   - 需要先停止所有相关进程
   - 可以注销整个 recipe、package、或特定的 cap/skill

**完整工作流程示例：**
```bash
# 1. 注册 recipe
rbnx deploy register demo_recipe.yaml

# 2. 构建所有 packages（可选，如果需要编译等操作）
rbnx deploy build

# 3. 启动所有进程
rbnx deploy start

# 4. 查看状态
rbnx deploy status

# 5. 停止所有进程
rbnx deploy stop

# 6. 注销 recipe
rbnx deploy unregister demo_recipe.yaml
```

标准 capabilities 和 skills 的规范定义参见 [`robonix-core/src/specs_table.rs`](robonix-core/src/specs_table.rs)。

## 示例

完整示例请参考：
- Package 示例：`rust/provider/demo_package/` 目录（包含完整的 `rbnx_manifest.yaml` 和启动/停止脚本）
- Recipe 示例：`rust/robonix-cli/demo_recipe.yaml` 文件
